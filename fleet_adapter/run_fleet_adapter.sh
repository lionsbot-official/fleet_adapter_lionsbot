#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ $# -ne 1 || ! "$1" =~ ^[0-9]+$ || "$1" -lt 1 ]]; then
    echo "Usage: $0 <fleet-number>" >&2
    echo "Example: $0 1" >&2
    exit 2
fi

FLEET_NUMBER="$1"

ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
    echo "Error: env file not found: $ENV_FILE" >&2
    echo "Copy .env.example to .env and fill in the required values." >&2
    exit 1
fi

set -a
source "$ENV_FILE"
set +a

: "${LIONSBOT_USER:?LIONSBOT_USER is required}"
: "${LIONSBOT_PASSWORD:?LIONSBOT_PASSWORD is required}"

CONFIG_VARIABLE="FLEET_${FLEET_NUMBER}_CONFIG"
NAV_GRAPH_VARIABLE="FLEET_${FLEET_NUMBER}_NAV_GRAPH"

FLEET_CONFIG="${!CONFIG_VARIABLE:-}"
NAV_GRAPH="${!NAV_GRAPH_VARIABLE:-}"

if [[ -z "$FLEET_CONFIG" || -z "$NAV_GRAPH" ]]; then
    echo "Error: .env must define both variables for fleet ${FLEET_NUMBER}:" >&2
    echo "  ${CONFIG_VARIABLE}=..." >&2
    echo "  ${NAV_GRAPH_VARIABLE}=..." >&2
    exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
else
    set -u
    echo "Error: ROS 2 Jazzy not found at /opt/ros/jazzy. This package requires Jazzy." >&2
    exit 1
fi
source "$SCRIPT_DIR/install/setup.bash"
set -u

FOLDER_NAME="${FOLDER_NAME:-office_new}"
SERVER_URI="${SERVER_URI:-ws://localhost:8000/_internal}"
USE_SIM_TIME="${USE_SIM_TIME:-false}"
SCHEDULE_TIMEOUT="${SCHEDULE_TIMEOUT:-60}"

CONFIG_DIR="$SCRIPT_DIR/configs/$FOLDER_NAME"
MAP_DIR="$SCRIPT_DIR/maps/$FOLDER_NAME"
CONFIG_FILE="$CONFIG_DIR/$FLEET_CONFIG"
NAV_GRAPH_FILE="$MAP_DIR/$NAV_GRAPH"
DOCK_SUMMARY_FILE="$MAP_DIR/dock_summary.yaml"

for path in "$CONFIG_FILE" "$NAV_GRAPH_FILE" "$DOCK_SUMMARY_FILE"; do
    if [[ ! -f "$path" ]]; then
        echo "Required file not found: $path" >&2
        exit 1
    fi
done

EXPANDED_CONFIG_FILE="$(mktemp /tmp/fleet_adapter_config.XXXXXX.yaml)"
cleanup() {
    rm -f "$EXPANDED_CONFIG_FILE"
}
trap cleanup EXIT INT TERM

python3 -c '
import os
import sys
import yaml

source_path, target_path = sys.argv[1], sys.argv[2]

def expand_env_vars(value):
    if isinstance(value, dict):
        return {key: expand_env_vars(child) for key, child in value.items()}
    if isinstance(value, list):
        return [expand_env_vars(item) for item in value]
    if isinstance(value, str):
        return os.path.expandvars(value)
    return value

with open(source_path, "r") as source_file:
    config = expand_env_vars(yaml.safe_load(source_file))
with open(target_path, "w") as target_file:
    yaml.safe_dump(config, target_file, sort_keys=False)
' "$CONFIG_FILE" "$EXPANDED_CONFIG_FILE"

echo "Waiting up to ${SCHEDULE_TIMEOUT}s for /rmf_traffic_schedule..."
for ((attempt = 1; attempt <= SCHEDULE_TIMEOUT; attempt++)); do
    if ros2 node list 2>/dev/null | grep -qx '/rmf_traffic_schedule'; then
        echo "Found /rmf_traffic_schedule."
        break
    fi
    if (( attempt == SCHEDULE_TIMEOUT )); then
        echo "Timed out waiting for /rmf_traffic_schedule." >&2
        exit 1
    fi
    sleep 1
done

echo "Starting fleet adapter..."
ros2 run fleet_adapter fleet_adapter \
    -c "$EXPANDED_CONFIG_FILE" \
    -n "$NAV_GRAPH_FILE" \
    -d "$DOCK_SUMMARY_FILE" \
    --server_uri "$SERVER_URI" \
    --ros-args -p use_sim_time:="$USE_SIM_TIME"
