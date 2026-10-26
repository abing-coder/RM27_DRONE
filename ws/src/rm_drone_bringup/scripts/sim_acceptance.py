#!/usr/bin/env python3
"""Isolated simulation acceptance run; truth is recorded only for evaluation.

Source ROS/workspace first. Does not run sim.sh or stop any existing simulation.
Use --scenario return for a short takeoff, EGO cruise, visual return/landing test.
"""
import argparse
import collections
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET


def validate_description(scratch):
    """Check URDF→SDF semantics; custom collision names can lose contact data."""
    from ament_index_python.packages import get_package_share_directory
    xacro = Path(get_package_share_directory('rm_drone_description'))/'urdf/rm_drone.urdf.xacro'
    urdf = subprocess.check_output(['xacro',str(xacro),'use_depth:=true'],text=True)
    source = ET.fromstring(urdf)
    path = Path(scratch)/'robot.urdf'
    path.write_text(urdf)
    converted = ET.fromstring(subprocess.check_output(['gz','sdf','-p',str(path)],text=True))
    collision = converted.find("model/link[@name='base_link']/collision")
    if collision is None:
        raise RuntimeError('Converted robot has no base collision')
    contact = {}
    for tag, target in [('kp','contact/ode/kp'),('kd','contact/ode/kd'),('mu1','friction/ode/mu'),('mu2','friction/ode/mu2')]:
        requested = source.findtext("gazebo[@reference='base_link']/"+tag)
        actual = collision.findtext('surface/'+target)
        if requested is None or actual is None or not math.isfinite(float(actual)) or not math.isclose(float(actual),float(requested),rel_tol=1e-5):
            raise RuntimeError(f'URDF→SDF lost contact parameter {tag}: requested={requested}, actual={actual}')
        contact[tag]=float(actual)
    return contact


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--profile', choices=['gt', 'vision'], default='vision')
    ap.add_argument('--team', choices=['red', 'blue'], default='red')
    ap.add_argument('--scenario', choices=['mission', 'return', 'depth-loss', 'takeoff-depth-loss'], default='mission')
    ap.add_argument('--detector', choices=['yolo', 'traditional'], default='yolo')
    ap.add_argument('--allow-blind-attack', action='store_true')
    ap.add_argument('--domain', type=int, default=156)
    ap.add_argument('--port', type=int, default=23518)
    ap.add_argument('--timeout', type=float, default=480)
    ap.add_argument('--model', default='')
    ap.add_argument('--target', default='auto')
    ap.add_argument('--lockstep', choices=['auto','true','false'], default='auto')
    args = ap.parse_args()
    if not 1 <= args.port <= 65535:
        ap.error('Gazebo port must be between 1 and 65535')
    try:
        with socket.create_connection(('127.0.0.1',args.port),timeout=.3):
            pass
    except ConnectionRefusedError:
        pass
    except OSError as error:
        ap.error(f'Cannot verify isolated Gazebo port: {error}')
    else:
        ap.error('Gazebo port is occupied; choose a separate --port and --domain')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.port}')
    import cv2
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from rm_drone_msgs.msg import ArmorDetectionArray, EstimatorStatus, ImpactEvent, MissionState, PlannerStatus
    from sensor_msgs.msg import Image, PointCloud2
    from std_msgs.msg import Bool, Empty
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init()
    node = rclpy.create_node('sim_acceptance', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    bridge = CvBridge()
    counts, last, states, errors, hits, detections = collections.Counter(), {}, [], [], [], []
    home, final_position, touchdown_position = None, None, None
    max_angular_rate = 0.0
    vio_alignment, vio_origin = None, None
    vio_errors = []
    debug_saved = False
    drop_start = None
    drop_window = []
    return_sent = False
    cruise_started = None
    takeoff_started = None
    fault_scenario = args.scenario in ['depth-loss','takeoff-depth-loss']
    subs = []
    def callback(topic, msg):
        nonlocal home, final_position, debug_saved, vio_alignment, vio_origin, touchdown_position, max_angular_rate
        counts[topic] += 1
        last[topic] = msg
        sim = node.get_clock().now().nanoseconds*1e-9
        if isinstance(msg, MissionState):
            if not states or states[-1]['state'] != msg.state or states[-1].get('returning') != ('returning=true' in msg.info):
                states.append(dict(t=sim, state=msg.state, info=msg.info, returning='returning=true' in msg.info))
                print('STATE', round(sim, 2), msg.state, msg.info, flush=True)
        if topic == '/drone/ground_truth/odom':
            angular = msg.twist.twist.angular
            max_angular_rate=max(max_angular_rate,math.sqrt(angular.x**2+angular.y**2+angular.z**2))
            p = msg.pose.pose.position
            final_position = np.array([p.x, p.y, p.z])
            if home is None:
                home = final_position.copy()
            ms=last.get('/drone/mission_state');ground=last.get('/drone/grounded');arm=last.get('/drone/arm')
            if ms and ms.state=='LAND' and ground and ground.data and arm and arm.data:
                touchdown_position=final_position.copy()
            est = last.get('/drone/odom')
            if est:
                dt = abs((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)-(est.header.stamp.sec+est.header.stamp.nanosec*1e-9))
                if dt < .02:
                    pe = est.pose.pose.position
                    error = np.array([pe.x,pe.y,pe.z])-final_position
                    errors.append(dict(t=sim, error=error.tolist()))
            raw = last.get('/ov_msckf/odomimu')
            if raw:
                rp = raw.pose.pose.position
                vp = np.array([rp.x,rp.y,rp.z])
                def rotation(q):
                    v = np.array([q.x,q.y,q.z]); w=q.w
                    skew=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
                    return np.eye(3)+2*w*skew+2*skew@skew
                if vio_alignment is None:
                    # Evaluation alignment only; never published to the robot.
                    vio_alignment=rotation(msg.pose.pose.orientation)@rotation(raw.pose.pose.orientation).T
                    vio_origin=final_position-vio_alignment@vp
                dt=abs((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)-(raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9))
                if dt<.02:
                    error=vio_alignment@vp+vio_origin-final_position
                    vio_errors.append(dict(t=sim,error=error.tolist()))
        if isinstance(msg, ImpactEvent):
            hits.append(dict(t=sim, armor=msg.armor_name, striker=msg.striker_name, approach_speed=msg.approach_speed, duration_ms=msg.duration*1000,
                             impulse=msg.impulse, peak_force=msg.peak_force))
        if isinstance(msg, ArmorDetectionArray) and msg.armors:
            detections.append(dict(t=sim, count=len(msg.armors), map_count=sum(a.has_map_pose for a in msg.armors)))
        if topic == '/drone/vision/debug_image' and detections and not debug_saved:
            cv2.imwrite(str(output/'armor-detection.jpg'), bridge.imgmsg_to_cv2(msg, 'bgr8'))
            debug_saved = True
    topics = [('/ov_msckf/odomimu',Odometry),('/drone/odom',Odometry),('/drone/ground_truth/odom',Odometry),('/drone/armors',ArmorDetectionArray),
              ('/drone/estimator_status',EstimatorStatus),('/drone/mission_state',MissionState),('/referee/impact',ImpactEvent),
              ('/drone/docked',Bool),('/drone/grounded',Bool),('/drone/arm',Bool),('/drone/ego/status',PlannerStatus),
              ('/drone/vision/debug_image',Image),('/drone/landing/debug_image',Image)]
    for topic, typ in topics:
        subs.append(node.create_subscription(typ, topic, lambda msg,t=topic:callback(t,msg), qos_profile_sensor_data))
    return_pub = node.create_publisher(Empty, '/drone/mission/return', 10)
    if fault_scenario:
        relay = node.create_publisher(PointCloud2, '/drone/acceptance/depth', qos_profile_sensor_data)
        def relay_depth(message):
            sim = node.get_clock().now().nanoseconds*1e-9
            if not drop_window or not (drop_window[0] <= sim < drop_window[0]+2):
                relay.publish(message)
        subs.append(node.create_subscription(PointCloud2, '/drone/depth/points', relay_depth, qos_profile_sensor_data))
    launch = ['ros2','launch','rm_drone_bringup','sim.launch.py',f'profile:={args.profile}',f'team:={args.team}',
              f'detector:={args.detector}',f'target:={args.target}',('vio:=stereo' if args.profile=='vision' else 'vio:=off'),'gui:=false','rviz:=false','imshow:=false',
              'render_backend:=native',f'lockstep:={args.lockstep}','allow_blind_attack:='+str(args.allow_blind_attack).lower()]
    if fault_scenario:
        launch.append('depth_topic:=/drone/acceptance/depth')
    if args.model:
        launch.append('yolo_model:='+str(Path(args.model).resolve()))
    result = {}
    with tempfile.TemporaryDirectory(prefix='rm27-acceptance-') as scratch:
        result['converted_contact'] = validate_description(scratch)
        print('DESCRIPTION',result['converted_contact'],flush=True)
        os.environ['ROS_LOG_DIR'] = str(Path(scratch)/'ros')
        log = (output/'launch.log').open('w')
        proc = subprocess.Popen(launch, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline, next_report = time.monotonic()+args.timeout, time.monotonic()+20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if proc.poll() is not None:
                    result['failure'] = 'launch_exit'
                    break
                mission = last.get('/drone/mission_state')
                if mission and mission.state == 'CRUISE' and cruise_started is None:
                    cruise_started = node.get_clock().now().nanoseconds*1e-9
                sim = node.get_clock().now().nanoseconds*1e-9
                if mission and mission.state=='TAKEOFF' and takeoff_started is None:
                    takeoff_started=sim
                if fault_scenario:
                    trigger = (takeoff_started is not None and mission and mission.state=='TAKEOFF' and sim-takeoff_started>.35) if args.scenario=='takeoff-depth-loss' else (cruise_started is not None and sim-cruise_started>2)
                    if drop_start is None and trigger:
                        drop_start = sim
                        drop_window.append(sim)
                        print('FAULT depth_drop', sim, flush=True)
                    if drop_start is not None and sim-drop_start > 3 and mission and mission.state == 'CRUISE' and not return_sent:
                        return_pub.publish(Empty())
                        return_sent = True
                if args.scenario == 'return' and cruise_started is not None and node.get_clock().now().nanoseconds*1e-9-cruise_started > 2 and not return_sent:
                    return_pub.publish(Empty())
                    return_sent = True
                if mission and mission.state in ['DONE', 'ESTOP']:
                    # Allow dock/disarm feedback and final image to arrive.
                    end = time.monotonic()+1.5
                    while time.monotonic()<end:rclpy.spin_once(node, timeout_sec=.03)
                    break
                if time.monotonic()>next_report:
                    health=last.get('/drone/estimator_status')
                    print('PROGRESS', {k:v for k,v in counts.items() if 'image' not in k}, 'health=',health.source if health else None, flush=True)
                    next_report=time.monotonic()+20
            else:
                result['failure']='timeout'
            mission = last.get('/drone/mission_state')
            landed = bool(mission and mission.state == 'DONE' and not last.get('/drone/arm', Bool(data=True)).data)
            returned = bool(home is not None and touchdown_position is not None and np.linalg.norm((touchdown_position-home)[:2]) < .08)
            physical_hit = any(h['striker']=='rm_drone' and h['approach_speed']>1 and mission and h['armor']==mission.target for h in hits)
            confirmed = bool(detections and any(d['map_count']>0 for d in detections))
            fault_brake = bool(drop_start is not None and any(x['state']=='BRAKE' and drop_start <= x['t'] < drop_start+2 for x in states))
            fault_recovered = bool(drop_start is not None and any(x['state']=='CRUISE' and x['t'] > drop_start+2 for x in states))
            takeoff_resumed = bool(drop_start is not None and any(x['state']=='TAKEOFF' and x['t']>drop_start+2 for x in states))
            outcome = mission.info.split('result=',1)[1].split()[0] if mission and 'result=' in mission.info else 'unknown'
            expected_outcome = 'contact_completed' if args.scenario == 'mission' else 'operator_return'
            result.update(passed='failure' not in result and outcome == expected_outcome and landed and returned and (args.scenario=='return' or (fault_scenario and fault_brake and fault_recovered and (args.scenario!='takeoff-depth-loss' or takeoff_resumed)) or (args.scenario=='mission' and physical_hit and (confirmed or args.allow_blind_attack))),
                          outcome=outcome,
                          profile=args.profile,team=args.team,scenario=args.scenario,detector=args.detector,
                          depth_drop_start=drop_start,depth_fault_brake=fault_brake,depth_fault_recovered=fault_recovered,takeoff_resumed=takeoff_resumed,
                          blind_attack=args.allow_blind_attack,states=states,counts=dict(counts),hits=hits,max_angular_rate_rad_s=max_angular_rate,
                          detection_frames=len(detections),vision_confirmed=confirmed,returned=returned,disarmed_landing=landed,
                          final_position=final_position.tolist() if final_position is not None else None,
                          touchdown_position=touchdown_position.tolist() if touchdown_position is not None else None,
                          touchdown_xy_error_m=float(np.linalg.norm((touchdown_position-home)[:2])) if touchdown_position is not None and home is not None else None,
                          home=home.tolist() if home is not None else None)
            if errors:
                e=np.array([e['error'] for e in errors]);result['position_rmse_m']=np.sqrt(np.mean(e*e,axis=0)).tolist()
                result['position_max_error_m']=np.max(np.linalg.norm(e,axis=1)).item()
                (output/'estimator-errors.json').write_text(json.dumps(errors))
            if vio_errors:
                e=np.array([e['error'] for e in vio_errors]);result['raw_vio_position_rmse_m']=np.sqrt(np.mean(e*e,axis=0)).tolist()
                result['raw_vio_position_max_error_m']=float(np.max(np.linalg.norm(e,axis=1)))
                (output/'raw-vio-errors.json').write_text(json.dumps(vio_errors))
            for topic,name in [('/drone/landing/debug_image','landing.jpg'),('/drone/vision/debug_image','final-vision.jpg')]:
                if topic in last:cv2.imwrite(str(output/name),bridge.imgmsg_to_cv2(last[topic],'bgr8'))
        finally:
            try:os.killpg(proc.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                try:os.killpg(proc.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                proc.wait()
            log.close()
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
    (output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    raise SystemExit(0 if result.get('passed') else 1)


if __name__ == '__main__':
    main()
