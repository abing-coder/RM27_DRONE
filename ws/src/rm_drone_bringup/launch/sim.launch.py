#!/usr/bin/env python3
"""RMUC 2027 空中机器人仿真启动。

常用参数：
  team:=red|blue
  target:=auto|blue_base_armor_0|blue_outpost|...
  estimator:=gt|eskf          # gt 用仿真真值调试控制/规划；eskf 融合 OpenVINS+ToF
  vio:=mono|stereo|off
  use_depth:=true|false
  gui:=true|false             # Gazebo 窗口
  rviz:=true|false
  image_view:=true|false      # 机载彩色相机 + 装甲识别叠加画面
  auto_start:=true|false
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def _hangar(team: str):
    if team == "blue":
        return 1.9, 1.0, 1.02, 3.1416
    return -1.9, -1.0, 1.02, 0.0


def launch_setup(context):
    team = LaunchConfiguration("team").perform(context)
    vio = LaunchConfiguration("vio").perform(context)
    estimator = LaunchConfiguration("estimator").perform(context)
    use_depth = LaunchConfiguration("use_depth").perform(context)
    target = LaunchConfiguration("target").perform(context)

    field_share = get_package_share_directory("rm_field_gazebo")
    desc_share = get_package_share_directory("rm_drone_description")
    ctrl_share = get_package_share_directory("rm_drone_control")
    est_share = get_package_share_directory("rm_drone_estimator")
    plan_share = get_package_share_directory("rm_drone_planner")
    bringup_share = get_package_share_directory("rm_drone_bringup")
    plugin_lib = os.path.join(get_package_prefix("rm_drone_gazebo_plugins"), "lib")

    world = os.path.join(field_share, "worlds", "rmuc2027.world")
    field_map = os.path.join(field_share, "config", "field_map.yaml")
    xacro = os.path.join(desc_share, "urdf", "rm_drone.urdf.xacro")
    hx, hy, hz, hyaw = _hangar(team)
    enemy = "red" if team == "blue" else "blue"

    robot_description = ParameterValue(
        Command(["xacro ", xacro, " ns:=drone use_depth:=", use_depth]),
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

    # WSL 桌面的 OpenGL 会让 gzserver 的 OGRE 卡在着色器初始化。
    # 物理和相机渲染放在 Xvfb（llvmpipe）里；gzclient 仍用用户桌面，只负责看 3D。
    user_display = os.environ.get("DISPLAY", ":0")
    gazebo_server = ExecuteProcess(
        cmd=[
            "xvfb-run", "-a", "-s", "-screen 0 1280x720x24",
            "gzserver", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            world,
        ],
        output="screen",
        additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
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
                         {"mode": estimator, "team": team, "field_map": field_map,
                          "use_vio": False,
                          "use_sim_truth": LaunchConfiguration("use_sim_truth").perform(context) == "true"}],
             remappings=[("vio_odom", "/ov_msckf/odomimu")],
             output="screen"),
        Node(package="rm_drone_vision", executable="armor_detector_node", namespace="drone",
             parameters=[ns, {"enemy_color": enemy,
                              "imshow": LaunchConfiguration("imshow").perform(context) == "true"}],
             output="screen"),
        Node(package="rm_drone_planner", executable="mission_node", namespace="drone",
             parameters=[os.path.join(plan_share, "config", "mission.yaml"), ns,
                         {"team": team, "target": target, "field_map": field_map,
                          "auto_start": LaunchConfiguration("auto_start").perform(context) == "true",
                          "require_vio": False}],
             output="screen"),
        Node(package="rm_drone_planner", executable="referee_sim_node",
             parameters=[ns, {"field_map": field_map}], output="screen"),
    ]

    ov_nodes = []
    run_vio = LaunchConfiguration("run_vio").perform(context) == "true"
    if run_vio and vio in ("mono", "stereo"):
        ov_cfg = os.path.join(est_share, "config", "openvins", vio, "estimator_config.yaml")
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
            "file:///home/abing/RM27_DRONE/ws/src/rm_field_gazebo/models",
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
        DeclareLaunchArgument("estimator", default_value="gt"),
        DeclareLaunchArgument("vio", default_value="off"),
        DeclareLaunchArgument("run_vio", default_value="false"),
        DeclareLaunchArgument("use_depth", default_value="true"),
        DeclareLaunchArgument("use_sim_truth", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("imshow", default_value="true"),
        DeclareLaunchArgument("image_view", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
