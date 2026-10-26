#!/usr/bin/env bash
# Reproducible ROS build with the project-maintained OpenVINS patch.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH="$ROOT/patches/openvins-static-zupt.patch"
VIO_SOURCE="$ROOT/ws/src/open_vins"
if [[ ! -f "$VIO_SOURCE/ov_msckf/src/core/VioManager.cpp" ]]; then
  echo '请先运行 git submodule update --init --recursive' >&2
  exit 1
fi
if git -C "$VIO_SOURCE" apply --reverse --check "$PATCH" 2>/dev/null; then
  : # Already applied. Preserve all unrelated work in the submodule.
elif git -C "$VIO_SOURCE" apply --check "$PATCH"; then
  git -C "$VIO_SOURCE" apply "$PATCH"
else
  echo 'OpenVINS 补丁与当前源码冲突，请检查 patches/openvins-static-zupt.patch；未覆盖源码。' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
cd "$ROOT/ws"
export MAKEFLAGS="${MAKEFLAGS:--j1}"
exec colcon build --symlink-install --executor sequential "$@"
