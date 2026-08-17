#!/bin/bash
set -euo pipefail

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'

# ROS setup scripts may reference unset shell variables, so source them with
# nounset disabled. Restore strict variable checking before starting the node.
set +u
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

: "${LIONSBOT_USER:?LIONSBOT_USER is required}"
: "${LIONSBOT_PASSWORD:?LIONSBOT_PASSWORD is required}"
: "${SERVER_URI:?SERVER_URI is required}"

BASE="/ws/src/fleet_adapter"
FOLDER_NAME="${FOLDER_NAME:-office_new}"
FLEET_CONFIG="${FLEET_CONFIG:-config_r5.yaml}"
NAV_GRAPH="${NAV_GRAPH:-0.yaml}"

CONFIG_DIR="$BASE/configs/$FOLDER_NAME"
MAP_DIR="$BASE/maps/$FOLDER_NAME"
DOCK_SUMMARY_FILE="$MAP_DIR/dock_summary.yaml"

if [[ ! -d "$CONFIG_DIR" || ! -d "$MAP_DIR" ]]; then
  echo "Expected directories not found:" >&2
  echo "  $CONFIG_DIR" >&2
  echo "  $MAP_DIR" >&2
  exit 1
fi

if [[ ! -f "$DOCK_SUMMARY_FILE" ]]; then
  echo "Dock summary file not found: $DOCK_SUMMARY_FILE" >&2
  exit 1
fi

# Match run_fleet.sh: expand environment variables in each fleet YAML before
# passing the generated configuration to the adapter.
run_fleet_adapter() {
  local config_name="$1"
  local nav_graph_name="$2"
  local config_file="$CONFIG_DIR/$config_name"
  local nav_graph_file="$MAP_DIR/$nav_graph_name"
  local expanded_config_file

  if [[ ! -f "$config_file" ]]; then
    echo "Config file not found: $config_file" >&2
    return 1
  fi
  if [[ ! -f "$nav_graph_file" ]]; then
    echo "Nav graph file not found: $nav_graph_file" >&2
    return 1
  fi

  expanded_config_file="$(mktemp /tmp/fleet_adapter_config.XXXXXX.yaml)"
  trap 'rm -f "$expanded_config_file"' RETURN

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
' "$config_file" "$expanded_config_file"

  ros2 run fleet_adapter fleet_adapter \
    -c "$expanded_config_file" \
    -n "$nav_graph_file" \
    -d "$DOCK_SUMMARY_FILE" \
    --server_uri "$SERVER_URI" \
    --ros-args -p use_sim_time:="${USE_SIM_TIME:-false}"
}

# Adapter.make() requires the RMF schedule node to be ready when it starts.
SCHEDULE_NODE="${SCHEDULE_NODE:-/rmf_traffic_schedule}"
SCHEDULE_TIMEOUT="${SCHEDULE_TIMEOUT:-60}"
echo "Waiting up to ${SCHEDULE_TIMEOUT}s for ${SCHEDULE_NODE}..."
for ((attempt = 1; attempt <= SCHEDULE_TIMEOUT; attempt++)); do
  if ros2 node list 2>/dev/null | grep -qx "$SCHEDULE_NODE"; then
    echo "Found ${SCHEDULE_NODE}."
    break
  fi
  if (( attempt == SCHEDULE_TIMEOUT )); then
    echo "Timed out waiting for ${SCHEDULE_NODE}." >&2
    exit 1
  fi
  sleep 1
done

echo "Starting fleet adapters..."
run_fleet_adapter "$FLEET_CONFIG" "$NAV_GRAPH"
