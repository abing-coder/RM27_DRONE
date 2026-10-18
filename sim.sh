#!/usr/bin/env bash
# RMUC 2027 空中机器人仿真一键启动
#
#   ./sim.sh                  真值位姿，先把控制/规划/撞击跑通
#   ./sim.sh                  自动起飞（不用 OpenVINS，避免定位漂移导致打转）
#   ./sim.sh run_vio:=true vio:=mono   额外打开 OpenVINS
#   ./sim.sh stereo           ESKF + 双目 OpenVINS
#   ./sim.sh gt gui:=false    其余参数原样传给 ros2 launch
#
# 常用覆盖：team:=blue  target:=blue_outpost  gui:=false  rviz:=false  auto_start:=false
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

# 只让 xvfb 里的 gzserver 用软件渲染。不要加到整个桌面进程上，否则 gzclient 会卡死。
unset LIBGL_ALWAYS_SOFTWARE
# OpenCV 的 imshow 走 GTK。WSL 上 GTK 默认会去找 Wayland，窗口就出不来；Gazebo/RViz 走的是 X11。
export GDK_BACKEND=x11
# 空字符串会变成 //database.config，gzclient 仍用 libcurl 去连，SSL 失败时窗口卡住
export GAZEBO_MODEL_DATABASE_URI="file:///home/abing/RM27_DRONE/ws/src/rm_field_gazebo/models"

estimator="eskf"
vio="off"
extra=()
for arg in "$@"; do
  case "$arg" in
    mono|stereo|off) vio="$arg" ;;
    gt|eskf) estimator="$arg" ;;
    *) extra+=("$arg") ;;
  esac
done
# 指定了 VIO 但没写估计器时，默认走 ESKF
if [[ "$vio" != "off" && "$estimator" == "gt" ]]; then
  for arg in "$@"; do
    if [[ "$arg" == "gt" ]]; then
      estimator="gt"
      break
    fi
    estimator="eskf"
  done
fi

if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
  echo "结束残留的 Gazebo 进程..."
  pkill -9 gzserver 2>/dev/null || true
  pkill -9 gzclient 2>/dev/null || true
  sleep 1
fi

echo "启动仿真：estimator:=${estimator} vio:=${vio} ${extra[*]:-}"
exec ros2 launch rm_drone_bringup sim.launch.py \
  "estimator:=${estimator}" \
  "vio:=${vio}" \
  "${extra[@]}"
