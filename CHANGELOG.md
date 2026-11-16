# CHANGELOG

## v2.1.0 (2026-11-16)

### Features

- feat: start the Open-RMF setup tool in `tools/lb_rmf_setup/`, a Python
  script with one entry point (`lb_rmf_setup.py`), its exit codes, and a CI
  gate on Python 3.10 and 3.12. Later changes add the commands.
- The repository root now has the Apache-2.0 `LICENSE` file.
- feat: the setup tool has the robot map frame and a standard-library copy
  of the adapter's map transform, tested against the adapter's own.
- feat: the setup tool reads the RMF building file and writes vertex
  changes in the traffic-editor style. Only changed vertex lines change.
- feat: the setup tool plans the vertex export for one level: match by name,
  append new vertices, keep hand-placed positions unless asked to move them,
  report the move distance and the dock heading, and write files through a
  temporary file and a rename.
- feat: the setup tool merges one fleet's cleaning paths into
  `dock_summary.yaml` by name and keeps every other fleet and entry.
- feat: the setup tool's first command, `export`, reads a robot map from a
  `.info` file (`--from-info`) or a `.geometry.json` file (`--from-geometry`)
  and updates the building file and `dock_summary.yaml` for one level.
  `--dry-run` prints the diff; `--check` exits 10 when the files would change.
- feat: the setup tool has a LionsBot API client: email and password login
  from environment variables, one new token after a
  401, retries on 429, 502, 503 and 504, and signed-link downloads without the
  Authorization header.
- feat: the setup tool's online commands: `auth check`, `robots list`,
  `maps list`, `maps fetch` (plain map image, ROS map yaml and geometry file)
  and `export` straight from the LionsBot API. `--env-file` reads unset
  variables from a dotenv file; `--verbose` prints each API call without
  secrets.
- feat: the setup tool's `lifts export` command writes the RMF `lifts:` block
  from the cabin point of each lift. On request it also adds cabin vertices
  (`--lift-cabins`), approach vertices with a lane to the cabin
  (`--lift-approach`) and lobby vertices (`--lobbies`); `--all` adds all
  three. It stops before any write when
  a cabin point is missing, when the cabin points do not line up within 0.1 m
  across levels, or when another vertex sits in a car.
- feat: `export` draws the lane from each dock's approach point to the dock.
  The approach point is the named point that the robot map keeps in front of
  the dock, facing it. When the map has none, `export` adds
  `<dock>_approach` 0.7 m in front of the dock. You no longer draw the dock
  lane or work out its heading.
- feat: `export` fills in `charger.waypoint` in the fleet config when it still
  has the template value. It uses the robot's home dock from the LionsBot API,
  or the only dock on the map.
- feat: new `init` command. From a robot serial and a building name it
  writes the fleet config from the template with the robot UUID and robot
  map from the LionsBot API, makes the site folders, and sets the site in
  `.env`.
- feat: `export` fills in the map transform while the config has the
  template value. It reads it from the traffic editor layer of the robot
  map, or from the measurements when the floor plan is the robot map.
- feat: `export` writes `level_transform` for a level that is not the
  reference level and has none in the config. It takes the scale and the
  shift from the fiducials the level shares with the reference level, the
  way RMF does. It stops when the two levels share fewer than 2 fiducials,
  when the fiducials turn the floor plan by more than 0.5 degrees, or when
  their scale is more than 1 % from the measurements of the level. A
  `level_transform` set by hand is never changed.
- fix: `export` no longer lists a "kept position" for a move under half a
  pixel. Those were rounding, not hand-placed changes.
- feat: new `check` command. It names each vertex that a lane misses, each
  named point with no lane, each stray vertex and each robot without a
  charger. It exits with code 9 when it finds a problem.
- feat: `check` names each waypoint name that is on more than one level.
  RMF refuses such a nav graph with "Duplicated waypoint name".
- docs: "Set up a new site" is one list of steps in order, with a short
  list of the words it uses (robot map, RMF building map, nav graph, map
  transform, dock and approach point). New FAQ entries cover a timeout
  waiting for `/rmf_traffic_schedule`, a robot that does not dock, and a
  waypoint that the dashboard does not show. The install notes ask for
  Docker Engine, not Docker Desktop.
- docs: README step 2 gives the setup script commands, environment variables
  and exit codes, and step 2b covers lifts. The old internal map generation
  script is removed.
- feat: every setup script command takes `--json` and prints one JSON result
  object, also on failure. `maps fetch` and `--layer-png` never replace an
  existing file without `--overwrite`. Output uses public words only.
- fix: `export` no longer adds a lift's cabin point or `<lift>_door` point as a
  vertex; `lifts export` places the lift vertices. Exporting every level and
  then running `lifts export` now works, and a second run changes nothing.
- fix: the setup script finds the robot map of each level the way the adapter
  does, from `rmf_config.robot_maps` as well as the start level, so one robot
  on two levels works with `export` and `lifts export`. The config template
  shows `robot_maps` and `robot_map_level`.
- fix: the config template no longer sets `level_transform` (its `scale: 1.0`
  made the setup script place every point about 20 times off). The setup
  script now stops with exit 3 when the floorplan scale is outside 0.001 to
  0.5 metres per pixel.
- fix: `run_fleet.sh` reads `RMF_MAP_NAME` (default `rmf_<FOLDER_NAME>`) and
  `RMF_FLOOR_NAME` (default `L8`) from `.env`, and stops before the build
  when the building file does not exist.
- fix: `package.xml` declares `python3-numpy`, `python3-requests` and
  `python3-websocket`, so `rosdep` installs them. The README says how to
  install `nudged`, which has no `rosdep` key.
- docs: README fixes from a clean-machine run: which release has the setup
  script, PyYAML without `pip` (or a virtual environment), starting the
  traffic editor after a binary install, fiducials for more than one level
  with a nav graph check, and the Docker Compose option with `.env` and
  `sudo -E`.
- fix: when the API says a robot map is not synced yet, `export` and
  `lifts export` stop with exit 7 and the message "map <name> is not synced to
  the robot yet: sync it in the app, then run again", not an API error.
- feat: the adapter reads `experimental.lift_commands` from the fleet config.
  The default is `false`. The adapter stops at start when the `experimental`
  section has an unknown key or a value that is not `true` or `false`.
- feat: the adapter's robot client can send the lift commands
  `lift_enter_start` and `lift_exit_start` on the robot status WebSocket and
  reads `lift_enter_end_status` and `lift_exit_end_status`. Each command gets
  a new `request_id`. The client reads the command error and its reason code,
  and it stops the robot when no end status comes in 6 minutes.
- feat: with `experimental.lift_commands: true`, the adapter moves a robot
  into a lift car with `lift_enter_start` and out of it with
  `lift_exit_start`. After the ride it changes the robot map and sets the
  robot pose at the cabin vertex. The adapter reads the RMF `lift_states`
  topic: it sends a lift command only when the car is at the robot's level
  with open doors, stopped, in this robot's lift session, and it stops the
  robot when the car moves or the session changes. A failed lift move is
  never reported to RMF as an arrival. The adapter never requests a lift.
  With the key at `false`, nothing changes. The package now depends on
  `rmf_lift_msgs`.
- feat: when a robot does not answer a lift command (reason code
  `RCSE00006`), the adapter fails the lift move with a message that says
  what to do, cancels the task, and sends that robot no more lift commands
  until the adapter starts again. Work on the robot's current level
  continues.
- feat: when a robot reports `Estop`, `Error` or `Offline`, the adapter opens
  an RMF issue on the robot and sends one RMF Web alert. A clean or dock
  action in progress is marked blocked, or delayed for `Offline`. `Offline`
  counts only after it lasts `navigation.timeout_grace_sec`. When the state
  ends, the issue is resolved. The adapter sends the robot no command because
  of these states.
- feat: a mock RMF lift adapter (`mock_lift_adapter`) for a test at a site
  with no real lift. `MOCK_LIFTS=true` in `.env` makes `run_fleet.sh` start
  it (launch argument `mock_lifts`). It reads the lifts from the building
  map, answers `lift_requests` and publishes `lift_states`. It moves no real
  lift.
- feat: `tools/lift_e2e/run.sh` tests a lift trip on one machine with no
  robot and no lift. It starts RMF with the mock lift adapter on
  `ROS_DOMAIN_ID` 58 and the real fleet adapter with a fake robot, sends the
  robot to another level and back, and checks each step. It ends with
  `Skipped` on a machine with no ROS 2 Jazzy.
- feat: RViz shows the whole building in 3D. `building_3d_visualizer` draws
  every level at its elevation (floor plan image, walls, doors, lanes, level
  name), each lift shaft with a car that follows `/lift_states`, and each
  robot from `/fleet_states` with a heading arrow, a name, a trail and a
  colour for its mode. It publishes a tf frame for each level, so the launch
  file has no static tf. `robot_marker_visualizer` no longer draws robots. It
  puts the schedule markers of the selected level at that level's elevation.
  The package now needs `python3-pil` and `tf2_ros`.

### Bug Fixes

- fix: when the adapter has an error while it checks if a move is a lift
  move, it no longer tells RMF that the robot arrived. The robot stays where
  it is and the adapter writes the error to its log and to the RMF task log.
- fix: with RMF running, every move of a robot on a map with a lift failed
  with `is_in_lift(): incompatible function arguments`, and the adapter then
  told RMF that the robot had arrived. RMF moved the lift car while the robot
  stood still. The adapter now gives the RMF lift check its second argument.
  A test now runs the check against the real RMF binding on a ROS machine.
- fix: `rmf.launch.xml` now starts the RMF `lift_supervisor`. Without it, the
  lift requests of a fleet adapter (`adapter_lift_requests`) never reached the
  lift adapter (`lift_requests`), so no robot could take a lift.
- fix: the setup tool now refuses a `level_transform` with `tx_pixels` or
  `ty_pixels` but no `scale`. The tool took the scale from the building file
  and the adapter took 1, so the robot was far from its waypoints on that
  level. The config template now says how to fill `level_transform`. A test
  runs `export` and RMF's `building_map_generator` on a second level and
  compares the nav graph with the adapter's position.
- fix: a robot sent to a dock by RMF now docks. The adapter sent
  `dock_start` with the heading the robot arrives at, facing into the
  charger. The robot expects the heading of the dock marker on its map,
  which points out of the charger, and refused the command with
  `Destination inaccessible`. The adapter now sends the reverse of the
  arrival heading.
- fix: the adapter finds an RMF traffic schedule by any node name, such as
  `rmf_traffic_schedule_primary` in rmf_demos. The adapter waits for the
  schedule itself, up to `SCHEDULE_TIMEOUT` seconds (default 60), and then
  stops with a message that names `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION`
  and says what to check. The scripts no longer wait by node name or tell you
  to run `run_fleet.sh`.
- fix: the scripts and the Docker setup keep a `CYCLONEDDS_URI` that you set,
  for example in `.env` to pick a network card. Before, they always replaced
  it. Without one, they use the same default as before.
- fix: RMF Web alerts go to the server of `SERVER_URI` when
  `RMF_WEB_ALERTS_URL` is not set. Before, they went to localhost. The adapter
  warns at start when it cannot reach `SERVER_URI` and when
  `RMF_WEB_API_TOKEN` is not set. The token is optional, also with Docker.
- fix: the adapter logs an error at start when `SERVER_URI` is `wss://`. The
  RMF websocket client in ROS 2 Jazzy fails with "endpoint not secure" and
  does not try again, so use `ws://`.
- fix: `maps fetch` writes the lifts and lift lobbies of the robot map into
  the `.geometry.json` file. Before, the file always had none, so
  `lifts export --from-geometry` added no lift.
- fix: `lifts export` no longer lists the docks and zones of a level as "not
  on this robot map". It reads only the lift points, so every other point
  looked missing.
- fix: `init` says to download the robot map image first, and `export`
  says to add the level to `rmf_config.robot_maps` when no robot is on it.
- fix: a `429` answer of the LionsBot API now says to wait a minute and run
  the command again. Dry-run diffs label the changed file `(after)`, not
  `(new)`.
- fix: **with `experimental.lift_commands: false`, the adapter no longer
  drives a robot into or out of a lift car with a point-to-point move.** A
  LionsBot robot enters and leaves a car only with `lift_enter_start` and
  `lift_exit_start`. The adapter fails that move, the robot does not move,
  and outside the car the task is cancelled.
- fix: the adapter no longer stops with `InvalidTopicNameError` when it adds a
  robot to RMF. The failure occurs with rmf_ros2 2.10.0 or later, which
  includes ROS 2 Kilted and RMF built from source. RMF now gets a robot name
  that is a valid ROS 2 topic segment: `R3SCP-B02357` becomes `R3SCP_B02357`.
  **The robot name in RMF Web, in task requests and in ROS messages changes
  for every robot whose name has a hyphen, on every RMF version.** The
  LionsBot API continues to use the LionsBot robot name. Set
  `robot_config.rmf_name` to choose the RMF name. The adapter stops at startup
  when `rmf_name` is not a valid topic segment or when two robots have the
  same RMF name. Thanks to @MinatoNami for the report and the fix.
- fix: a clean no longer times out while the robot reports `Cleaning` or
  `Cleaning Paused`. Before, a clean that ran longer than
  `navigation.max_timeout_sec` timed out and the adapter sent `clean_start`
  again, also to a robot that was still cleaning or already docking. The
  timeout now counts only the time after the robot last
  reported that it cleans. When a robot that was cleaning starts to dock, or
  is docked or charging, and sent no `clean_end_status`, the adapter tells
  RMF that the clean finished and sends the robot no more commands. When such
  a robot rests for `navigation.timeout_grace_sec` instead, the clean ends as
  with a good `clean_end_status`. A clean that never starts still times out
  and is tried again as before.

### CI

- ci: the leak-guard job fails when a published file holds an internal ticket
  key. Changelog entries name a change in plain words, or by its public
  release commit.

### Documentation

- docs: the README says what a site needs for lift trips (the RMF lift
  supervisor and what the lift adapter must publish) and how to test lift
  taking with the mock lift adapter.
- docs: the "Future improvements" in the README say what is done and what is
  left for network security and RMF Web scheduled tasks.
- docs: the README has an Experimental features section. It describes the
  `experimental.lift_commands` key of the fleet config.


## v2.0.1 (2026-10-23)

### Bug Fixes

- fix: the Docker image now installs the patched Ubuntu
  `ca-certificates` (20260816ubuntu0.24.04.1 or later), so the adapter no
  longer trusts the compromised eMudhra roots "emSign ECC Root CA - G3" and
  "AAA Certificate Services". The image build fails if the CA bundle still
  holds either root. Rebuild deployed images to take the fix. A host that runs
  the adapter without Docker uses its own CA bundle: run
  `sudo apt-get install --only-upgrade ca-certificates` on it.


## v2.0.0 (2026-08-17)

### Breaking Changes

- **LeoScrub robots are no longer supported.** The `fleet_adapter_leoscrub`
  package has been removed. Deployments running LeoScrub fleets should remain
  on v1.1.2.
- **The adapter is now a single `fleet_adapter` package.** It replaces the
  former `fleet_adapter_r3` package, so any installation or launch
  configuration referring to `fleet_adapter_r3` by name must be updated.
- **ROS 2 Jazzy is now required** (previously Foxy/Galactic), which in turn
  requires Ubuntu 24.04.

### Features

- Support for R3 (Vac, Scrub Pro) and R5 robot families. REX support is in
  testing.
- Docker Compose deployment and an RViz visualisation stack.
- A documented configuration template (`configs/config_template.yaml`) plus a
  worked `office_new` example.

### Bug Fixes

- Detect silent robot telemetry WebSocket failures and reconnect both sockets
  so robot status and pose subscriptions are restored automatically.


## v1.1.1 (2025-03-28)

### Bug Fixes

- Remove jwt token decode to obtain jwt expiry time
  ([`d680841`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/d680841acc29d184c9cde09d42fb89eaf0cf457d))


## v1.1.0 (2025-03-28)

### Features

- Add github workflow to increment release versions
  ([`620ae18`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/620ae18cdea46a42dbd95ad2a8ff500ad3a363da))


## v1.0.0 (2025-03-12)

### Bug Fixes

- Treat navigation as success when robot is near target position
  ([`5ef78ea`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/5ef78ea60606573a9da4095b7b2b9c9f91b75ecc))

### Chores

- Remove unnecessary file
  ([`a9cf007`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/a9cf007b0eec24ceb7774e9869ca438a9a6113c6))

### Features

- Add support for scrubpro and vac robot types
  ([`ab8fc1d`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/ab8fc1df715646f41b600908c473a2c4e19ca633))

- Initialize repo
  ([`a088cef`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/a088ceff4d16b25425efe693ffae536bff487bb8))

- Update broken readme url
  ([`f4f8afd`](https://github.com/lionsbot-official/fleet_adapter_lionsbot/commit/f4f8afd73009d45bd559daede865b7088538e7f3))
