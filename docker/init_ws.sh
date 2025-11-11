#!/usr/bin/env bash
set -e

WS=${WS:-/home/dev/code/polarquest/ws}
REPO=${REPO:-https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git}

mkdir -p "$WS/src"

# Se src estiver vazia, clona
if [ ! -d "$WS/src/POLARIS_GEM_e2/.git" ]; then
  echo "[init_ws] Cloning repo in $WS/src ..."
  git clone --depth=1 "$REPO" "$WS/src/POLARIS_GEM_e2"
fi

# Build
echo "[init_ws] Running catking make in $WS ..."
source /opt/ros/$ROS_DISTRO/setup.bash
cd "$WS"
catkin_make -DCMAKE_BUILD_TYPE=Release

exec bash -lc "tail -f /dev/null"

