#!/usr/bin/env bash
# The image entrypoint supplies the ROS overlay and virtual environment.
set -Eeuo pipefail

mkdir -p Output/Gate1
run_dir="$(mktemp -d Output/Gate1/run_XXXXXXXX)"
printf 'Experiment artifacts: %s\n' "${run_dir}"

run_status=0
ros2 run hybrid_nav gate1_experiment \
    --duration "${EXPERIMENT_DURATION_SECONDS:-30}" \
    --output-dir "${run_dir}" || run_status=$?

# Retain the report for a failed safety gate too. A plot cannot make a failed
# experiment pass, and a report failure must also fail this command.
report_status=0
python /opt/gate1/generate_ros_validation_report.py \
    --input "${run_dir}/summary.json" \
    --output "${run_dir}/plots" || report_status=$?

if [[ "${run_status}" -ne 0 ]]; then
    exit "${run_status}"
fi
exit "${report_status}"
