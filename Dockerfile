# syntax=docker/dockerfile:1
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        git \
        libgl1 \
        libglib2.0-0 \
        libusb-1.0-0 \
        udev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/duck_ws

COPY . /opt/duck_ws/src/open_duck_mini_runtime

RUN python3 -m pip install --upgrade pip
RUN pip install --no-cache-dir -e src/open_duck_mini_runtime

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select duck_walk_bringup"

ENV DUCK_WS=/opt/duck_ws
ENV ROS_DOMAIN_ID=0

RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
    && echo 'source /opt/duck_ws/install/setup.bash' >> /etc/bash.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
