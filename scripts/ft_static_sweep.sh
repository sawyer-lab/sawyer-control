#!/usr/bin/env bash

set -euo pipefail

# Defaults
DURATION="${1:-15}"
POLL_RATE="${2:-100}"
SENSOR_RATE="${3:-7000}"
POSE_ID="${4:-P001}"

# OUTPUT_DIR="results/static/${POSE_ID}-${DURATION}s"
OUTPUT_DIR="results/static"

mkdir -p "${OUTPUT_DIR}"

# Sensor-rate label used in filenames
case "${SENSOR_RATE}" in
    # 7000) RATE_LABEL="7000Hz" ;;
    # 1000) RATE_LABEL="1000Hz" ;;
    100)  RATE_LABEL="100Hz" ;;
    *)    RATE_LABEL="${SENSOR_RATE}Hz" ;;
esac

# ATI filter codes
FILTER_LABELS=(
    "no-filter"  # 0
    "838Hz"      # 1
    "326Hz"      # 2
    "152Hz"      # 3
    "73Hz"       # 4
    "35Hz"       # 5
    "18Hz"       # 6
    "8Hz"        # 7
    "5Hz"        # 8
)

# Select the filters to test.
# For all filters:
# FILTER_CODES=(0 1 2 3 4 5 6 7 8)

# For only one filter:
FILTER_CODES=(5)

POSE_FILE="${OUTPUT_DIR}/${POSE_ID}-cfg.txt"

# echo "Saving robot pose..."
# python scripts/demo/inspect_robot.py > "${POSE_FILE}"

# echo "Pose saved to:"
# echo "  ${POSE_FILE}"

# Allow everything to settle before the first acquisition.
sleep 5

for FILTER_CODE in "${FILTER_CODES[@]}"; do
    FILTER_LABEL="${FILTER_LABELS[$FILTER_CODE]}"

    OUTPUT_FILE="${OUTPUT_DIR}/${POSE_ID}-${RATE_LABEL}-${FILTER_LABEL}-${DURATION}s.jsonl"

    echo
    echo "========================================"
    echo "Running static acquisition"
    echo "  pose      = ${POSE_ID}"
    echo "  duration  = ${DURATION} s"
    echo "  poll rate = ${POLL_RATE} Hz"
    echo "  rate      = ${SENSOR_RATE} Hz"
    echo "  filter    = ${FILTER_CODE} (${FILTER_LABEL})"
    echo "  output    = ${OUTPUT_FILE}"
    echo "========================================"

    # Let the sensor/bridge settle before starting the next record.
    sleep 5

    python scripts/demo/ft_acquisition.py \
        --duration "${DURATION}" \
        --poll-rate "${POLL_RATE}" \
        --sensor-rate "${SENSOR_RATE}" \
        --filter-code "${FILTER_CODE}" \
        --output "${OUTPUT_FILE}"

    echo "Finished filter ${FILTER_CODE} (${FILTER_LABEL})."

    # Allow configuration/session changes to settle.
    sleep 5
done

echo
echo "Static sweep completed."
echo "Results:"
echo "  ${OUTPUT_DIR}"