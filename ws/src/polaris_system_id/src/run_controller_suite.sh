#!/usr/bin/env bash
set -euo pipefail

# Run from a sourced ROS/catkin shell after copying the supplied files into
# polaris_system_id/{scripts,launch,config} and rebuilding/sourcing the workspace.

PACKAGE="polaris_system_id"
LAUNCH="gem_controller_comparison.launch"
REPEATS="${REPEATS:-3}"
ROBUST_REPEATS="${ROBUST_REPEATS:-1}"
INCLUDE_PURE_PURSUIT="${INCLUDE_PURE_PURSUIT:-0}"
GUI="${GUI:-false}"
RVIZ="${RVIZ:-false}"
MODEL_CONFIG="${MODEL_CONFIG:-$(rospack find ${PACKAGE})/config/greybox_current_node.yaml}"
SUITE_ID="${SUITE_ID:-$(date +%Y%m%d_%H%M%S)}"

controllers=(greybox_mpc kinematic_mpc stanley)
if [[ "${INCLUDE_PURE_PURSUIT}" == "1" ]]; then
  controllers+=(pure_pursuit)
fi

# Nominal pose: x=33.4, y=-92.9, yaw=-0.7207 rad.
# Lateral offsets are +/-0.5 m along the initial left-normal.
scenario_names=(nominal lateral_plus_0p5 lateral_minus_0p5 yaw_plus_5deg yaw_minus_5deg)
scenario_x=(33.4 33.730 33.070 33.4 33.4)
scenario_y=(-92.9 -92.524 -93.276 -92.9 -92.9)
scenario_yaw=(-0.7207 -0.7207 -0.7207 -0.633434 -0.807966)

run_one() {
  local controller="$1" scenario="$2" repeat="$3" x="$4" y="$5" yaw="$6"
  echo "=== ${controller} | ${scenario} | repeat ${repeat} ==="
  roslaunch "${PACKAGE}" "${LAUNCH}" \
    controller:="${controller}" \
    scenario:="${scenario}" \
    repeat:="${repeat}" \
    run_id:="${SUITE_ID}_${scenario}_r${repeat}" \
    model_config:="${MODEL_CONFIG}" \
    x:="${x}" y:="${y}" yaw:="${yaw}" \
    gui:="${GUI}" rviz:="${RVIZ}"
  sleep 2
}

for controller in "${controllers[@]}"; do
  # Three nominal repetitions by default.
  for ((r=1; r<=REPEATS; r++)); do
    run_one "${controller}" nominal "${r}" 33.4 -92.9 -0.7207
  done

  # Robustness perturbations; one repetition each by default.
  for i in 1 2 3 4; do
    for ((r=1; r<=ROBUST_REPEATS; r++)); do
      run_one "${controller}" "${scenario_names[$i]}" "${r}" \
        "${scenario_x[$i]}" "${scenario_y[$i]}" "${scenario_yaw[$i]}"
    done
  done
done

echo "All runs completed. Aggregate them with aggregate_results.py."
