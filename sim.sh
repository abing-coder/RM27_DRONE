#!/usr/bin/env bash
# RMUC 2027 空中机器人仿真。
#
#   ./sim.sh                 真值调试：只验证控制和规划，estimator=gt
#   ./sim.sh gt              同上
#   ./sim.sh mono            视觉验收：单目 OpenVINS，关闭真值，要求健康估计
#   ./sim.sh stereo          视觉验收：双目
#   ./sim.sh off             ESKF，不启 VIO，也不用真值
#   ./sim.sh detector:=yolo auto_start:=false  查看 YOLO11；默认保留传统灯条识别
#
# 起飞还要比赛阶段和机库放行，见 README。其余参数原样传给 launch。
# 矛盾组合（例如视觉验收再打开 use_sim_truth）会在 launch 里直接失败。
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f "$ROOT/ws/install/setup.bash" ]]; then
  echo "工作空间还没编译。先执行：" >&2
  echo "  source /opt/ros/humble/setup.bash && cd $ROOT/ws && colcon build --symlink-install" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo/setup.sh
elif [[ -f /usr/share/gazebo-11/setup.sh ]]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo-11/setup.sh
fi
# shellcheck disable=SC1091
source "$ROOT/ws/install/setup.bash"

unset LIBGL_ALWAYS_SOFTWARE
export GDK_BACKEND=x11
export GAZEBO_MODEL_DATABASE_URI=""

profile="gt"
vio="auto"
extra=()
for arg in "$@"; do
  case "$arg" in
    gt) profile="gt" ;;
    mono|stereo) profile="vision"; vio="$arg" ;;
    off) profile="off" ;;
    real)
      echo "sim.sh 只会启动 Gazebo。实机禁止仿真真值，不要用本脚本冒充实机。" >&2
      exit 1
      ;;
    vio:=mono|vio:=stereo)
      profile="vision"
      vio="${arg#vio:=}"
      ;;
    vio:=off) profile="off"; vio="off" ;;
    *) extra+=("$arg") ;;
  esac
done

if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  echo "结束残留的 Gazebo 进程..."
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  sleep 1
fi

args=("profile:=${profile}")
if [[ "$vio" != "auto" ]]; then
  args+=("vio:=${vio}")
fi
echo "启动仿真：profile:=${profile} vio:=${vio} ${extra[*]:-}"
exec ros2 launch rm_drone_bringup sim.launch.py "${args[@]}" "${extra[@]}"
