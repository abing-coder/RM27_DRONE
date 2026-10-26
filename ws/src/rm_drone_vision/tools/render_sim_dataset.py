#!/usr/bin/env python3
"""Render the installed CAD field into a pose dataset; no simulator truth at inference.

Run after sourcing ROS and ws/install. Output must be an empty directory outside
source/model folders. Train/validation split holds out armor index 2 entirely.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--views', type=int, default=800)
    parser.add_argument('--seed', type=int, default=2701)
    parser.add_argument('--domain', type=int, default=154)
    parser.add_argument('--port', type=int, default=23516)
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists() and any(out.iterdir()):
        raise SystemExit('Output must be empty; existing data is never overwritten')
    out.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.port}',
                      GAZEBO_MODEL_DATABASE_URI='')
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from cv_bridge import CvBridge
    from gazebo_msgs.srv import SetEntityState
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    share = Path(get_package_share_directory('rm_field_gazebo'))
    os.environ['GAZEBO_MODEL_PATH'] = str(share/'models')+':'+os.environ.get('GAZEBO_MODEL_PATH', '')
    world_file = share/'worlds/rmuc2027_official.world'
    map_file = share/'config/field_map_official.yaml'
    field = yaml.load(map_file.read_text(), Loader=yaml.CSafeLoader)
    targets = field['targets']
    for split in ['train', 'val']:
        for kind in ['images', 'labels']:
            (out/kind/split).mkdir(parents=True)
    width, height, hfov = 640, 480, 1.6
    focal = width/(2*math.tan(hfov/2))
    rng = np.random.default_rng(args.seed)
    rclpy.init()
    node = rclpy.create_node('sim_dataset_renderer')
    bridge, frames = CvBridge(), {}
    def on_image(key, msg):
        frames[key] = (msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                       bridge.imgmsg_to_cv2(msg, 'bgr8' if key == 'rgb' else '32FC1'))
    subs = [node.create_subscription(Image, '/dataset/render/'+topic,
             lambda msg, key=key: on_image(key, msg), qos_profile_sensor_data)
            for key, topic in [('rgb', 'image_raw'), ('depth', 'depth/image_raw')]]
    client = node.create_client(SetEntityState, '/dataset/set_entity_state')
    records = []
    with tempfile.TemporaryDirectory(prefix='rm27-dataset-') as scratch:
        scratch = Path(scratch)
        os.environ['ROS_LOG_DIR'] = str(scratch/'ros')
        tree = ET.parse(world_file)
        world = tree.getroot().find('world')
        plugin = ET.SubElement(world, 'plugin', name='dataset_state', filename='libgazebo_ros_state.so')
        ET.SubElement(ET.SubElement(plugin, 'ros'), 'namespace').text = '/dataset'
        ET.SubElement(plugin, 'update_rate').text = '30'
        model = ET.SubElement(world, 'model', name='dataset_camera')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='camera')
        sensor = ET.SubElement(link, 'sensor', name='render', type='depth')
        ET.SubElement(sensor, 'always_on').text = 'true'
        ET.SubElement(sensor, 'update_rate').text = '15'
        cam = ET.SubElement(sensor, 'camera')
        ET.SubElement(cam, 'horizontal_fov').text = str(hfov)
        image = ET.SubElement(cam, 'image')
        for k, v in [('width', str(width)), ('height', str(height)), ('format', 'R8G8B8')]:
            ET.SubElement(image, k).text = v
        clip = ET.SubElement(cam, 'clip')
        ET.SubElement(clip, 'near').text = '.025'
        ET.SubElement(clip, 'far').text = '35'
        plug = ET.SubElement(sensor, 'plugin', name='render', filename='libgazebo_ros_camera.so')
        ET.SubElement(ET.SubElement(plug, 'ros'), 'namespace').text = '/dataset'
        ET.SubElement(plug, 'camera_name').text = 'render'
        ET.SubElement(plug, 'frame_name').text = 'render_optical'
        tree.write(scratch/'render.world')
        log = (scratch/'gazebo.log').open('w')
        proc = subprocess.Popen(['gzserver', '-s', 'libgazebo_ros_init.so', str(scratch/'render.world')],
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            if not client.wait_for_service(timeout_sec=60):
                raise RuntimeError('Camera pose service unavailable: '+(scratch/'gazebo.log').read_text()[-2000:])
            for i in range(args.views):
                target = targets[i % len(targets)]
                split = 'val' if target['name'].endswith('_2') else 'train'
                center = np.array(target['position'], float)
                normal = np.array(target['normal'], float)
                az = math.atan2(normal[1], normal[0])+rng.uniform(-.8, .8)
                elevation = rng.uniform(-.1, 1.15) if target['kind'] == 'base' else rng.uniform(-.6, .6)
                distance = math.exp(rng.uniform(math.log(.32), math.log(3.3)))
                eye = center+distance*np.array([math.cos(elevation)*math.cos(az),
                                                math.cos(elevation)*math.sin(az), math.sin(elevation)])
                look = center+rng.normal(0, distance*.12, 3)
                forward = look-eye
                forward /= np.linalg.norm(forward)
                left = np.cross([0, 0, 1], forward)
                left /= np.linalg.norm(left)
                up = np.cross(forward, left)
                roll = rng.uniform(-.18, .18)
                left, up = math.cos(roll)*left+math.sin(roll)*up, -math.sin(roll)*left+math.cos(roll)*up
                rotation = np.column_stack([forward, left, up])
                # Quaternion from a right-handed camera model rotation matrix.
                from scipy.spatial.transform import Rotation
                quat = Rotation.from_matrix(rotation).as_quat()
                req = SetEntityState.Request()
                req.state.name = 'dataset_camera'
                req.state.reference_frame = 'world'
                req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = map(float, eye)
                req.state.pose.orientation.x, req.state.pose.orientation.y, req.state.pose.orientation.z, req.state.pose.orientation.w = map(float, quat)
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                if future.result() is None or not future.result().success:
                    raise RuntimeError('Failed to set rendering camera pose')
                # Drain the old frame and require multiple subsequent synchronized renders.
                before = frames.get('rgb', (-1,))[0]
                deadline = time.monotonic()+8
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.02)
                    if 'rgb' in frames and 'depth' in frames and frames['rgb'][0] > before+.15 and abs(frames['rgb'][0]-frames['depth'][0]) < .001:
                        break
                else:
                    raise RuntimeError('Timed out waiting for a fresh RGB/depth pair')
                rgb, depth = frames['rgb'][1], frames['depth'][1]
                labels = []
                for obj in targets:
                    # Hold-out armor instances never appear as labels in the training set.
                    # Frames containing another split's visible armor are excluded below.
                    corners = np.array(obj['light_bar_corners'], float)
                    rel = corners-eye
                    z = rel@forward
                    if np.any(z <= .03) or np.dot(np.array(obj['normal']), eye-np.array(obj['position'])) <= .01:
                        continue
                    uv = np.column_stack([width/2-focal*(rel@left)/z, height/2-focal*(rel@up)/z])
                    if np.any(uv[:, 0] < 2) or np.any(uv[:, 0] >= width-2) or np.any(uv[:, 1] < 2) or np.any(uv[:, 1] >= height-2):
                        continue
                    if cv2.contourArea(uv.astype(np.float32)) < 12:
                        continue
                    # Real rendered depth checks occlusion at the four bar endpoints.
                    sampled = [depth[int(v), int(u)] for u, v in uv]
                    if any(not np.isfinite(d) or abs(float(d)-float(zz)) > .045 for d, zz in zip(sampled, z)):
                        continue
                    if (obj['name'].endswith('_2')) != (split == 'val'):
                        labels = None
                        break
                    lo, hi = uv.min(axis=0), uv.max(axis=0)
                    margin = np.maximum((hi-lo)*.12, 2)
                    lo, hi = np.maximum(lo-margin, [0, 0]), np.minimum(hi+margin, [width-1, height-1])
                    cls = (0 if obj['team'] == 'red' else 1)+(2 if obj['kind'] == 'outpost' else 0)
                    values = [cls, *((lo+hi)/2/[width, height]), *((hi-lo)/[width, height]), *(uv/[width, height]).reshape(-1)]
                    labels.append(' '.join(f'{v:.8g}' for v in values))
                if labels is None:
                    continue
                # Keep a bounded share of genuinely empty/background frames.
                if not labels and rng.random() > .15:
                    continue
                name = f'view_{i:05d}'
                cv2.imwrite(str(out/'images'/split/(name+'.jpg')), rgb, [cv2.IMWRITE_JPEG_QUALITY, 95])
                (out/'labels'/split/(name+'.txt')).write_text('\n'.join(labels)+'\n' if labels else '')
                records.append(dict(image=f'images/{split}/{name}.jpg', target=target['name'], eye=eye.tolist(), rotation=rotation.tolist(), objects=len(labels)))
                if i % 50 == 0:
                    print(f'{i}/{args.views}: saved {len(records)} frames', flush=True)
        finally:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=12)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            log.close()
            node.destroy_node()
            rclpy.shutdown()
    (out/'data.yaml').write_text(yaml.safe_dump(dict(path=str(out), train='images/train', val='images/val',
        kpt_shape=[4, 2], flip_idx=[3, 2, 1, 0], names={0:'red_base',1:'blue_base',2:'red_outpost',3:'blue_outpost'})))
    manifest = dict(seed=args.seed, requested_views=args.views, hfov=hfov, image_size=[width,height],
                    world_sha256=hashlib.sha256(world_file.read_bytes()).hexdigest(),
                    map_sha256=hashlib.sha256(map_file.read_bytes()).hexdigest(), records=records)
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
    print('Saved', len(records), 'independent camera views to', out, flush=True)


if __name__ == '__main__':
    main()
