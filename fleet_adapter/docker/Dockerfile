FROM ros:jazzy

SHELL ["/bin/bash", "-c"]

WORKDIR /ws/src/fleet_adapter
COPY . .

# Install only the dependencies needed to build the fleet adapter. The
# visualization dependencies are intentionally skipped because this image
# does not launch RViz or any RMF visualization node.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3-numpy \
      python3-yaml \
      python3-websocket \
      python3-requests \
      python3-pip \
      ros-jazzy-rmw-cyclonedds-cpp && \
    python3 -m pip install --break-system-packages \
      nudged==0.3.1 && \
    rosdep update && \
    rosdep install \
      --from-paths . \
      --ignore-src \
      --rosdistro jazzy \
      --skip-keys "rmf_visualization_building_systems rmf_visualization_fleet_states rmf_visualization_navgraphs rmf_visualization_floorplans rviz2" \
      -r -y && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws

RUN echo 'source /ws/install/setup.bash' >> /root/.bashrc

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select fleet_adapter --symlink-install

COPY docker/start_adapter.sh /start_adapter.sh
RUN chmod +x /start_adapter.sh

ENTRYPOINT ["/start_adapter.sh"]
