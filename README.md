# LionsBot Fleet Adapter
![LionsBot](https://github.com/lionsbot-official/fleet_adapter_lionsbot/blob/main/lionsbot-banner.png)

This repository contains the open source fleet adapter for integrating OpenRMF with LionsBot's robots. The different robot families adapters are in their respective directory.

## Supported Robot Family

Currently, fleet adapters are developed and has been tested for these supported families of robots:

| Fleet adapter          | Robot family | Robot type |
|------------------------| ---- |------------|
| fleet_adapter_leoscrub | Leobot | LeoScrub   |
| fleet_adapter_r3       | R3 | Scrub, Vac, Scrub Pro      |

## RMF Installation 

| RMF Version | Installation Instructions     | Supported distros | Supported ROS2 versions |
|-------|-------------------------------|-------------------|-----------------------|
| 22.09 | [Installation Instructions](https://github.com/open-rmf/rmf/tree/release/22.09) | Ubuntu 22.04 | Humble |

## Tasks Summary

| Task           | Description |
|----------------|-------------|
| dispatch_clean | send a cleaning task with specified zone |
| cancel_task    | cancel a specified task |
| dispatch_go_to_place | send a navigation task with a specified location |
| pause_task | pause the task of a specified robot |
| continue_task | resume the task of a specified robot |

> **NOTE**: `pause_task` and `continue_task` are custom tasks. The python scripts have to be added into where the rest of the official tasks are located.

## Building the fleet adapter

```bash
cd $HOME && mkdir -p fleet_adapter_lionsbot_ws/src
```

```bash
cd fleet_adapter_lionsbot_ws/
```

```bash
git clone https://github.com/lionsbot-official/fleet_adapter_lionsbot --branch rmf/22.09 --single-branch --depth 1 src
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
rosdep update --rosdistro $ROS_DISTRO
```

```bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr
```

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 
```

**Alternatively**, if you like to use **Docker** and avoid the hassle of resolving dependency conflicts, please run the commands below instead:

```bash
cd $HOME && git clone https://github.com/lionsbot-official/fleet_adapter_lionsbot --branch rmf/22.09 --single-branch --depth 1
```

Build docker image for `fleet_adapter_r3`:
```bash
cd fleet_adapter_lionsbot && docker build -f r3_Dockerfile -t fleet_adapter_lionsbot:r3
```

Build docker image for `fleet_adapter_leoscrub`:
```bash
cd fleet_adapter_lionsbot && docker build -f leoscrub_Dockerfile -t fleet_adapter_lionsbot:leoscrub
```

## Running the fleet adapter

```
ros2 run fleet_adapter_r3 fleet_adapter_r3 -c config.yaml -n nav_graph.yaml -d dock_summary.yaml
```

**Alternatively**, if you used the **Docker** instructions above to build, please run the commands below instead:

Build and run docker container for `fleet_adapter_r3`:
```bash
docker run -it --rm  \
--network host \
-v /dev/shm:/dev/shm \
--name fleet_adapter_lionsbot_r3_c \
fleet_adapter_lionsbot:r3 bash -c \
"ros2 run fleet_adapter_r3 fleet_adapter \
-c config.yaml \
-n nav_graph.yaml \
-d dock_summary.yaml"
```

Build and run docker container for `fleet_adapter_leoscrub`:
```bash
docker run -it --rm  \
--network host \
-v /dev/shm:/dev/shm \
--name fleet_adapter_lionsbot_leoscrub_c \
fleet_adapter_lionsbot:leoscrub bash -c \
"ros2 run fleet_adapter_leoscrub fleet_adapter \
-c config.yaml \
-n nav_graph.yaml \
-d dock_summary.yaml"
```
