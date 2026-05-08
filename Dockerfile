# RoboRacer ICRA 2026 - qualifying submission image.
# Base must be the official AutoDRIVE Devkit per competition rules.
FROM autodriveecosystem/autodrive_roboracer_api:2026-icra-practice

ARG OVERLAY=/home/autodrive_overlay
ENV OVERLAY=${OVERLAY}

# Extra ROS / system deps not present in the devkit image.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-example-interfaces \
      ros-humble-lifecycle-msgs \
      ros-humble-rclcpp-action \
      ros-humble-rclcpp-components \
      ros-humble-rclcpp-lifecycle \
      ros-humble-visualization-msgs \
      ros-humble-generate-parameter-library \
      python3-jsonschema \
 && rm -rf /var/lib/apt/lists/*

WORKDIR ${OVERLAY}

# Team packages. autodrive_roboracer is intentionally NOT copied —
# the devkit image already ships a built copy at /home/autodrive_devkit/install.
COPY src/racer            ${OVERLAY}/src/racer
COPY src/control          ${OVERLAY}/src/control
COPY src/perception       ${OVERLAY}/src/perception
COPY src/racer_interfaces ${OVERLAY}/src/racer_interfaces
COPY src/deps/clingwrap   ${OVERLAY}/src/clingwrap
COPY src/deps/jig/jig     ${OVERLAY}/src/jig

RUN bash -c "source /opt/ros/humble/setup.bash \
          && source /home/autodrive_devkit/install/setup.bash \
          && colcon build --merge-install \
               --cmake-args -DCMAKE_BUILD_TYPE=Release \
               --packages-up-to racer"

# Replace the devkit's stub entrypoint with one that launches our stack.
COPY autodrive_devkit.sh /home/autodrive_devkit.sh
RUN chmod +x /home/autodrive_devkit.sh

WORKDIR /home/autodrive_devkit
