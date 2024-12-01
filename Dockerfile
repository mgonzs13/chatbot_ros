# use the official ROS 2 humble base image
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

# set the working directory and copy files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

# clone dependencies
RUN cd /root/ros2_ws/src && vcs import < dependencies.repos

# install dependencies
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    gcc \
    git \
    wget \
    portaudio19-dev \
    python3 \
    python3-pip
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN pip3 install -r src/llama_ros/requirements.txt
RUN pip3 install -r src/whisper_ros/requirements.txt
RUN pip3 install -r src/tts_ros/requirements.txt

# colcon the ws
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# run a default command, e.g., starting a bash shell
CMD ["bash"]
