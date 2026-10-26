#!/usr/bin/env bash
# RMUC 2027 空中机器人仿真。
#
#   ./sim.sh                 新赛季 STEP 地图，真值调试 estimator=gt
#   ./sim.sh gt              同上
#   ./sim.sh mono            视觉验收：单目 OpenVINS，关闭真值，要求健康估计
#   ./sim.sh stereo          视觉验收：双目
#   ./sim.sh off             ESKF，不启 VIO，也不用真值
#   ./sim.sh auto_start:=false  查看 OpenVINO YOLO11 四关键点识别（默认）
#   ./sim.sh detector:=traditional  显式切换传统灯条识别作对照
#   ./sim.sh planner:=astar      对照原有静态地图规划；默认使用 EGO 在线避障
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
  echo "  cd $ROOT && ./build.sh --packages-skip ov_eval ov_data" >&2
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
    field:=official) ;;  # 唯一场地，由启动器统一传入。
    field:=*)
      echo "旧地图已停用，当前仅支持 2027 STEP 场地（默认，无需 field 参数）。" >&2
      exit 2
      ;;
    *) extra+=("$arg") ;;
  esac
done

# Never terminate another simulation. Refuse an occupied Gazebo master port.
if python3 - <<'PYPORT'
import os, socket
from urllib.parse import urlparse
master = urlparse(os.environ.get("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"))
try:
    with socket.create_connection((master.hostname or "127.0.0.1", master.port or 11345), timeout=.3):
        pass
except OSError:
    raise SystemExit(1)
PYPORT
then
  echo "Gazebo 主端口已被占用，请先退出对应仿真，或设置独立的 GAZEBO_MASTER_URI 和 ROS_DOMAIN_ID。" >&2
  exit 2
fi

args=("profile:=${profile}" "field:=official")
if [[ "$vio" != "auto" ]]; then
  args+=("vio:=${vio}")
fi
echo "启动仿真：field:=official profile:=${profile} vio:=${vio} ${extra[*]:-}"
exec ros2 launch rm_drone_bringup sim.launch.py "${args[@]}" "${extra[@]}"
