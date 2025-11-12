#!/usr/bin/env bash
set -e

roslaunch polaris_system_id gem_mpc_follower.launch "$@"

IMG="$(rospack find polaris_system_id)/src/analysis/data/cte_metrics/cte_over_time.png"
if [ -f "$IMG" ]; then
  xdg-open "$IMG" 2>/dev/null || eog "$IMG" 2>/dev/null || display "$IMG" 2>/dev/null || echo "RMSE: $IMG"
else
  echo "Imagem não encontrada: $IMG"
fi