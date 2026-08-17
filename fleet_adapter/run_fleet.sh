#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Load local settings and credentials. Keep this file out of Git.
ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
    echo "Error: env file not found: $ENV_FILE"
    echo "Copy .env.example to .env and fill in the required values."
    exit 1
fi
set -a
source "$ENV_FILE"
set +a

: "${LIONSBOT_USER:?Please export LIONSBOT_USER before running run_fleet.sh}"
: "${LIONSBOT_PASSWORD:?Please export LIONSBOT_PASSWORD before running run_fleet.sh}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOG_DIR="$SCRIPT_DIR/log/ros"
export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
mkdir -p "$ROS_LOG_DIR"

echo "=== 1. Cleaning up old build artifacts ==="
rm -rf build/ install/ .pytest_cache/ __pycache__/ *.egg-info/ src/*.egg-info

echo "=== 2. Sourcing ROS 2 environment ==="
set +u
if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
    source /opt/ros/jazzy/setup.bash
else
    set -u
    echo "Error: ROS 2 Jazzy not found at /opt/ros/jazzy. This package requires Jazzy."
    exit 1
fi
set -u


echo "=== 3. Updating dependencies ==="
rosdep install --from-paths . --ignore-src -r -y

echo "=== 4. Building the workspace ==="
colcon build --symlink-install

echo "=== 5. Sourcing the local workspace ==="
set +u
source install/setup.bash
set -u

echo "=== 6. Launching RMF ==="
RMF_LAUNCH_PID=""

ros2 launch fleet_adapter rmf.launch.xml \
    map_yaml_file:="$SCRIPT_DIR/maps/$FOLDER_NAME/rmf_${FOLDER_NAME}.building.yaml" \
    floor_name:=L8 &
RMF_LAUNCH_PID=$!

cleanup() {
    # Prevent EXIT from invoking cleanup again while cleanup is running.
    trap - EXIT INT TERM
    set +e

    # ros2 launch owns the RMF nodes and RViz, so stopping it stops its children.
    if [[ -n "$RMF_LAUNCH_PID" ]] && kill -0 "$RMF_LAUNCH_PID" 2>/dev/null; then
        echo "=== Shutting down RMF core and RViz ==="
        kill -INT "$RMF_LAUNCH_PID" 2>/dev/null
        wait "$RMF_LAUNCH_PID" 2>/dev/null
    fi

    echo "=== Removing RMF web containers ==="
    sudo docker compose -f "$SCRIPT_DIR/launch/docker-compose.yaml" down
}
trap cleanup EXIT INT TERM

if [[ -z "$RMF_LAUNCH_PID" ]]; then
    echo "Error: failed to start RMF launch."
    exit 1
fi

echo "=== 7. Starting Docker Compose ==="
sudo docker compose -f "$SCRIPT_DIR/launch/docker-compose.yaml" down
sudo docker compose -f "$SCRIPT_DIR/launch/docker-compose.yaml" up -d

echo "=== 8. Waiting for RMF Schedule Node ==="
schedule_ready=false
for attempt in {1..60}; do
    if ros2 node list 2>/dev/null | grep -q '^/rmf_traffic_schedule$'; then
        schedule_ready=true
        break
    fi
    if ! kill -0 "$RMF_LAUNCH_PID" 2>/dev/null; then
        echo "Error: the RMF launch process exited before rmf_traffic_schedule became ready."
        exit 1
    fi
    sleep 1
done

if [[ "$schedule_ready" != true ]]; then
    echo "Error: timed out waiting for /rmf_traffic_schedule."
    exit 1
fi

echo "=== RMF core and Web services are running ==="
echo "Start the fleet adapter separately with ./run_fleet_adapter.sh"
wait "$RMF_LAUNCH_PID"
