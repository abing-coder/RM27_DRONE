#!/usr/bin/env python3
"""RMUC 2027 空中机器人仿真启动。

profile:=gt       真值调试，只验证控制和规划
profile:=vision   开 VIO、关真值、要求健康估计
profile:=off      ESKF，不用 VIO，也不用真值
profile:=real     在本仿真入口中会直接失败

显式传入与 profile 矛盾的 estimator / vio / run_vio / use_sim_truth / require_vio 会报错，
不会静默退回真值。取值 auto 表示交给 profile。
"""
import os
import subprocess
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def _flag(text):
    return str(text).lower() in ("1", "true", "yes", "on")


def resolve_profile(profile, estimator, vio, run_vio, use_sim_truth, require_vio, runtime):
    """把 profile 展开成现有参数。auto 跟随 profile，显式冲突则失败。"""
    if profile == "gt":
        want = dict(estimator="gt", vio="off", run_vio=False, use_sim_truth=False,
                    require_vio=False, use_vio=False, allow_blind_attack=True)
    elif profile == "vision":
        want = dict(estimator="eskf", vio="mono", run_vio=True, use_sim_truth=False,
                    require_vio=True, use_vio=True, allow_blind_attack=True)
    elif profile == "off":
        want = dict(estimator="eskf", vio="off", run_vio=False, use_sim_truth=False,
                    require_vio=False, use_vio=False, allow_blind_attack=True)
    elif profile == "real":
        want = dict(estimator="eskf", vio="mono", run_vio=True, use_sim_truth=False,
                    require_vio=True, use_vio=True, allow_blind_attack=False)
    else:
        raise RuntimeError(f"未知 profile: {profile}")

    def take(name, value, parse):
        if value in ("", "auto"):
            return
        parsed = parse(value)
        if parsed != want[name]:
            raise RuntimeError(f"profile:={profile} 与 {name}:={value} 矛盾，拒绝启动")
        want[name] = parsed

    take("estimator", estimator, str)
    if profile == "vision" and vio in ("mono", "stereo"):
        want["vio"] = vio
    else:
        take("vio", vio, str)
    take("run_vio", run_vio, _flag)
    take("use_sim_truth", use_sim_truth, _flag)
    take("require_vio", require_vio, _flag)

    if profile == "vision" and want["vio"] not in ("mono", "stereo"):
        raise RuntimeError("视觉验收必须配置 mono 或 stereo")
    if want["require_vio"] and (not want["use_vio"] or not want["run_vio"] or want["vio"] not in ("mono", "stereo")):
        raise RuntimeError("要求 VIO，但未启动或未配置有效的 mono/stereo")
    if profile == "vision" and want["use_sim_truth"]:
        raise RuntimeError("视觉验收不能启用真值替代")
    if (runtime == "real" or profile == "real") and (want["use_sim_truth"] or want["estimator"] == "gt"):
        raise RuntimeError("实机配置不能使用仿真真值")
    if runtime == "sim" and profile == "real":
        raise RuntimeError("sim.launch 是仿真入口，不能使用 profile:=real")
    if want["use_sim_truth"]:
        raise RuntimeError("use_sim_truth 不能作为输出来源；真值调试用 profile:=gt")
    return want


def _hangar(team: str):
    if team == "blue":
        return 1.9, 1.0, 1.02, 3.1416
    return -1.9, -1.0, 1.02, 0.0


def resolve_render_backend(requested, display):
    """优先使用可用的桌面硬件 OpenGL；无显示/无硬件时使用 Xvfb。"""
    if requested != "auto":
        return requested, "explicit selection"
    if not display:
        return "software", "DISPLAY is unset"
    probe_env = dict(os.environ, LC_ALL="C", DISPLAY=display)
    probe_env.pop("LIBGL_ALWAYS_SOFTWARE", None)
    try:
        probe = subprocess.run(["glxinfo", "-B"], env=probe_env, capture_output=True,
                               text=True, timeout=3, check=False)
        renderer = next((line.split(":", 1)[1].strip() for line in probe.stdout.splitlines()
                         if line.startswith("OpenGL renderer string:")), "")
        software = any(name in renderer.lower() for name in
                       ("llvmpipe", "softpipe", "software rasterizer", "swiftshader"))
        if probe.returncode == 0 and renderer and not software and "direct rendering: Yes" in probe.stdout:
            return "native", renderer
        return "software", renderer or "hardware OpenGL unavailable"
    except (OSError, subprocess.TimeoutExpired):
        return "software", "OpenGL probe unavailable (install mesa-utils or select native explicitly)"


def launch_setup(context):
    team = LaunchConfiguration("team").perform(context)
    use_depth = LaunchConfiguration("use_depth").perform(context)
    target = LaunchConfiguration("target").perform(context)
    try:
        cfg = resolve_profile(
            LaunchConfiguration("profile").perform(context),
            LaunchConfiguration("estimator").perform(context),
            LaunchConfiguration("vio").perform(context),
            LaunchConfiguration("run_vio").perform(context),
            LaunchConfiguration("use_sim_truth").perform(context),
            LaunchConfiguration("require_vio").perform(context),
            LaunchConfiguration("runtime").perform(context),
        )
    except RuntimeError as exc:
        print(f"启动失败：{exc}", file=sys.stderr)
        raise
    vio = cfg["vio"]

    field_share = get_package_share_directory("rm_field_gazebo")
    desc_share = get_package_share_directory("rm_drone_description")
    ctrl_share = get_package_share_directory("rm_drone_control")
    est_share = get_package_share_directory("rm_drone_estimator")
    plan_share = get_package_share_directory("rm_drone_planner")
    bringup_share = get_package_share_directory("rm_drone_bringup")
    vision_share = get_package_share_directory("rm_drone_vision")
    detector = LaunchConfiguration("detector").perform(context)
    yolo_model = LaunchConfiguration("yolo_model").perform(context)
    if not yolo_model:
        yolo_model = os.path.join(vision_share, "models", "yolo11.xml")
    if detector == "yolo" and not os.path.isfile(yolo_model):
        raise RuntimeError(f"YOLO 模型不存在：{yolo_model}")
    plugin_lib = os.path.join(get_package_prefix("rm_drone_gazebo_plugins"), "lib")

    world = os.path.join(field_share, "worlds", "rmuc2027.world")
    field_map = os.path.join(field_share, "config", "field_map.yaml")
    xacro = os.path.join(desc_share, "urdf", "rm_drone.urdf.xacro")
    hx, hy, hz, hyaw = _hangar(team)
    enemy = "red" if team == "blue" else "blue"
    blind = LaunchConfiguration("allow_blind_attack").perform(context)
    if blind not in ("auto", "true", "false"):
        raise RuntimeError("allow_blind_attack 必须为 auto/true/false")
    allow_blind_attack = cfg["allow_blind_attack"] if blind == "auto" else blind == "true"
    lockstep_arg = LaunchConfiguration("lockstep").perform(context)
    lockstep = (cfg["use_vio"] or not allow_blind_attack) if lockstep_arg == "auto" else _flag(lockstep_arg)
    print(f"仿真调度：lockstep={lockstep}，VIO={vio}；起飞倒计时使用仿真时间", flush=True)

    robot_description = ParameterValue(
        Command(["xacro ", xacro, " ns:=drone use_depth:=", use_depth, " vio:=", vio]),
        value_type=str,
    )

    model_path = os.pathsep.join(
        [
            os.path.join(field_share, "models"),
            "/usr/share/gazebo-11/models",
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ]
    )
    plugin_path = os.pathsep.join(
        [plugin_lib, "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins", os.environ.get("GAZEBO_PLUGIN_PATH", "")]
    )
    resource_path = os.pathsep.join(
        [
            os.path.join(field_share, "models"),
            "/usr/share/gazebo-11",
            os.environ.get("GAZEBO_RESOURCE_PATH", ""),
        ]
    )

    user_display = os.environ.get("DISPLAY", "")
    render_backend, renderer = resolve_render_backend(
        LaunchConfiguration("render_backend").perform(context), user_display)
    print(f"相机渲染：{render_backend} ({renderer})；识别：{detector}/OpenVINO" if detector == "yolo"
          else f"相机渲染：{render_backend} ({renderer})；识别：traditional", flush=True)
    render_prefix = ["xvfb-run", "-a", "-s", "-screen 0 1280x720x24"] if render_backend == "software" else []
    gazebo_server = ExecuteProcess(
        cmd=[
            *render_prefix,
            # 视觉验收需要相机/物理锁步；真值控制调试不被软件渲染阻塞。
            "gzserver", "--verbose", *(["--lockstep"] if lockstep else []),
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world,
            # 使用参数文件，避免 -p 被 gzserver 解释成日志回放选项。
            "--ros-args", "--params-file", os.path.join(field_share, "config", "gazebo_ros.yaml"), "--",
        ],
        output="screen",
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1" if render_backend == "software" else "0"},
    )
    gazebo_client = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
        additional_env={"DISPLAY": user_display},
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "rm_drone",
            "-topic", "robot_description",
            "-x", str(hx), "-y", str(hy), "-z", str(hz), "-Y", str(hyaw),
        ],
        output="screen",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    ns = {"use_sim_time": True}
    nodes = [
        Node(package="rm_drone_control", executable="fc_sim_node", namespace="drone",
             parameters=[os.path.join(ctrl_share, "config", "control.yaml"), ns], output="screen"),
        Node(package="rm_drone_control", executable="position_controller_node", namespace="drone",
             parameters=[os.path.join(ctrl_share, "config", "control.yaml"), ns], output="screen"),
        Node(package="rm_drone_estimator", executable="eskf_node", namespace="drone",
             parameters=[os.path.join(plan_share, "config", "mission.yaml"), ns,
                         {"mode": cfg["estimator"], "team": team, "field_map": field_map,
                          "runtime": "sim", "use_vio": cfg["use_vio"], "use_sim_truth": False}],
             remappings=[("vio_odom", "/ov_msckf/odomimu")],
             output="screen"),
        Node(package="rm_drone_vision", executable="armor_detector_node", namespace="drone",
             parameters=[os.path.join(vision_share, "config", "vision.yaml"), ns,
                         {"enemy_color": enemy, "detector_backend": detector, "yolo_model": yolo_model,
                              "imshow": LaunchConfiguration("imshow").perform(context) == "true"}],
             on_exit=[Shutdown(reason="装甲识别节点退出，停止仿真")],
             output="screen"),
        Node(package="rm_drone_planner", executable="mission_node", namespace="drone",
             parameters=[os.path.join(plan_share, "config", "mission.yaml"), ns,
                         {"team": team, "target": target, "field_map": field_map,
                          "auto_start": LaunchConfiguration("auto_start").perform(context) == "true",
                          "require_vio": cfg["require_vio"],
                          "allow_blind_attack": allow_blind_attack}],
             output="screen"),
        Node(package="rm_drone_planner", executable="referee_sim_node",
             parameters=[ns, {"field_map": field_map, "phase": "match",
                              "flight_permitted": True, "hangar_released": True}],
             output="screen"),
    ]

    ov_nodes = []
    run_vio = cfg["run_vio"]
    if run_vio and vio in ("mono", "stereo"):
        ov_cfg = os.path.join(est_share, "config", "openvins", vio, "estimator_config.yaml")
        if not os.path.isfile(ov_cfg):
            raise RuntimeError(f"要求 VIO，但配置不存在：{ov_cfg}")
        ov_nodes.append(
            Node(
                package="ov_msckf",
                executable="run_subscribe_msckf",
                name="ov_msckf",
                namespace="ov_msckf",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "verbosity": "INFO",
                    "use_stereo": vio == "stereo",
                    "max_cameras": 2 if vio == "stereo" else 1,
                    "config_path": ov_cfg,
                }],
            )
        )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(bringup_share, "rviz", "sim.rviz")],
        parameters=[ns],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )
    color_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/drone/color/image_raw"],
        condition=IfCondition(LaunchConfiguration("image_view")),
    )
    debug_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/drone/vision/debug_image"],
        condition=IfCondition(LaunchConfiguration("image_view")),
    )
    depth_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=["/drone/depth/image_raw"],
        condition=IfCondition(LaunchConfiguration("image_view")),
    )

    return [
        SetEnvironmentVariable(
            "GAZEBO_MODEL_DATABASE_URI",
            "",  # 模型均由 GAZEBO_MODEL_PATH 查找，不将本地目录冒充在线模型数据库。
        ),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", plugin_path),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", resource_path),
        gazebo_server, gazebo_client, rsp, spawn,
        *nodes, *ov_nodes, rviz, color_view, debug_view, depth_view,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("team", default_value="red"),
        DeclareLaunchArgument("target", default_value="auto"),
        DeclareLaunchArgument("profile", default_value="gt"),
        DeclareLaunchArgument("runtime", default_value="sim"),
        DeclareLaunchArgument("estimator", default_value="auto"),
        DeclareLaunchArgument("vio", default_value="auto"),
        DeclareLaunchArgument("run_vio", default_value="auto"),
        DeclareLaunchArgument("require_vio", default_value="auto"),
        DeclareLaunchArgument("use_depth", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_sim_truth", default_value="auto"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("imshow", default_value="true"),
        DeclareLaunchArgument("image_view", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("lockstep", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("render_backend", default_value="auto", choices=["auto", "native", "software"]),
        DeclareLaunchArgument("detector", default_value="yolo", choices=["yolo", "traditional"]),
        DeclareLaunchArgument("yolo_model", default_value=""),
        DeclareLaunchArgument("allow_blind_attack", default_value="auto"),
        OpaqueFunction(function=launch_setup),
    ])
