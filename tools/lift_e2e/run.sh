#!/bin/bash
# End-to-end test of a lift trip on one machine, with no robot and no lift.
#
# It starts RMF from rmf.launch.xml with the mock lift adapter, and the real
# fleet adapter with a fake robot in place of the LionsBot cloud (see
# fake_robot_adapter.py). Then it sends the robot to the other level and back
# and checks each step (see observe.py). It contacts no robot and no cloud.
#
# Usage: tools/lift_e2e/run.sh
# It uses ROS_DOMAIN_ID 58, or LIFT_E2E_DOMAIN_ID. Never give it the domain
# of an RMF that runs. It stops only the processes that it starts.

set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$(dirname "$HERE")")"
SETUP_TOOL="$REPO/tools/lb_rmf_setup"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    echo "Skipped: ROS 2 Jazzy is not installed."
    exit 0
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${LIFT_E2E_DOMAIN_ID:-58}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORK="$(mktemp -d)"
export ROS_LOG_DIR="$WORK/roslog"
echo "Logs: $WORK"
# rmf_traffic_schedule writes a file in the current directory.
cd "$WORK" || exit 1

echo "=== 1. Two-level building with a lift, from the setup tool ==="
mkdir site
cp "$SETUP_TOOL"/testdata/twofloor/* site/
python3 "$SETUP_TOOL/lb_rmf_setup.py" lifts export --all \
    --building site/twofloor.building.yaml --config site/config_r3.yaml \
    --from-info L1=site/L1.info --from-info L3=site/L3.info || exit 1
# What a person does in traffic editor: a start point on L1, a goal on L3,
# and lanes from them to the lift.
python3 - site/twofloor.building.yaml <<'EOF'
import sys
import yaml

building = yaml.safe_load(open(sys.argv[1]))
lane = {'bidirectional': [4, True], 'graph_idx': [2, 0]}
for name, level in building['levels'].items():
    del level['drawing']
    level['fiducials'] = [[50, 50, 'f1'], [350, 350, 'f2']]
    vertices = level['vertices']
    index = {vertex[3]: number for number, vertex in enumerate(vertices)}
    place = 'start_L1' if name == 'L1' else 'goal_L3'
    vertices.append([100, 240, 0, place, {
        'is_charger': [4, name == 'L1'], 'is_parking_spot': [4, True]}])
    level['lanes'] += [
        [len(vertices) - 1, index[f'lobby_1_{name}'], lane],
        [index[f'lobby_1_{name}'], index[f'lift_a_{name}_approach'], lane]]
yaml.safe_dump(building, open(sys.argv[1], 'w'))
EOF
ros2 run rmf_building_map_tools building_map_generator nav \
    site/twofloor.building.yaml site/ > nav_graph.log 2>&1 || exit 1
echo '{}' > site/dock_summary.yaml

echo "=== 2. Building the fleet_adapter package ==="
colcon build --base-paths "$REPO/fleet_adapter" > build.log 2>&1 || {
    cat build.log
    exit 1
}
set +u
source install/setup.bash
set -u

STARTED=()
cleanup() {
    trap - EXIT INT TERM
    for pid in "${STARTED[@]}"; do
        kill -INT -- -"$pid" 2>/dev/null
    done
    sleep 3
    for pid in "${STARTED[@]}"; do
        kill -KILL -- -"$pid" 2>/dev/null
    done
}
trap cleanup EXIT INT TERM

echo "=== 3. Starting RMF with the mock lift adapter ==="
setsid ros2 launch fleet_adapter rmf.launch.xml headless:=true \
    mock_lifts:=true floor_name:=L1 \
    map_yaml_file:="$WORK/site/twofloor.building.yaml" > rmf.log 2>&1 &
STARTED+=($!)
sleep 8

echo "=== 4. Starting the fleet adapter with a fake robot ==="
setsid python3 "$HERE/fake_robot_adapter.py" \
    -c "$HERE/config.yaml" -n site/0.yaml -d site/dock_summary.yaml \
    --server_uri ws://127.0.0.1:9/_internal \
    --debug_log_file "$WORK/adapter_debug.log" > adapter.log 2>&1 &
STARTED+=($!)

echo "=== 5. Lift trip to L3 and back to L1 ==="
python3 "$HERE/observe.py" goal_L3=L3 start_L1=L1 | tee observe.log
result=${PIPESTATUS[0]}
grep "FAKE ROBOT" adapter.log
for command in lift_enter_start lift_exit_start; do
    if [[ "$(grep -c "FAKE ROBOT.*$command" adapter.log)" != 2 ]]; then
        echo "FAIL: the robot did not get $command one time for each trip"
        result=1
    fi
done
if grep -q "FAKE ROBOT.*ABORT" adapter.log; then
    echo "FAIL: the adapter stopped a lift move"
    result=1
fi

[[ "$result" == 0 ]] && echo "=== PASSED ===" || echo "=== FAILED ==="
exit "$result"
