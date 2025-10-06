# chatbot_ros

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-GPL--3.0-informational)](https://opensource.org/license/gpl-3-0) [![GitHub release](https://img.shields.io/github/release/mgonzs13/chatbot_ros.svg)](https://github.com/mgonzs13/chatbot_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/chatbot_ros.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/chatbot_ros.svg)](https://github.com/mgonzs13/chatbot_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/chatbot_ros)](https://github.com/mgonzs13/chatbot_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/chatbot_ros)](https://github.com/mgonzs13/chatbot_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/chatbot_ros.svg)](https://github.com/mgonzs13/chatbot_ros/graphs/contributors) [![Python Formatter Check](https://github.com/mgonzs13/chatbot_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/python-formatter.yml?branch=main)

| ROS 2 Distro |                           Branch                            |                                                                                                       Build status                                                                                                        | Docker Image | Documentation |
| :----------: | :---------------------------------------------------------: | :-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------: | ------------- |
|  **Humble**  | [`main`](https://github.com/mgonzs13/chatbot_ros/tree/main) | [![Humble Build](https://github.com/mgonzs13/chatbot_ros/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/humble-docker-build.yml?branch=main) |      -       | -             |

</div>

## Table of Contents

1. [Dependencies](#dependencies)
2. [Installation](#installation)
3. [Usage](#usage)

## Dependencies

- [yasmin](https://github.com/uleroboticsgroup/yasmin)[[3.4.0](https://github.com/uleroboticsgroup/yasmin/releases/tag/3.4.0)]
- [audio_common](https://github.com/mgonzs13/audio_common)[[4.0.7](https://github.com/mgonzs13/audio_common/releases/tag/4.0.7)]
- [llama_ros](https://github.com/mgonzs13/llama_ros)[[5.3.3](https://github.com/mgonzs13/llama_ros/releases/tag/5.3.3)]
- [whisper_ros](https://github.com/mgonzs13/whisper_ros)[[3.2.3](https://github.com/mgonzs13/whisper_ros/releases/tag/3.2.3)]
- [piper_ros](https://github.com/mgonzs13/piper_ros)[[1.3.5](https://github.com/mgonzs13/piper_ros/releases/tag/1.3.5)]

## Installation

Dependencies can be installed using [vcstool](https://github.com/dirk-thomas/vcstool). Then, you can follow the next steps to install all tools and the chatbot:

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/chatbot_ros
vcs import < chatbot_ros/dependencies.repos
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r src/llama_ros/requirements.txt
colcon build --cmake-args -DGGML_CUDA=ON
```

## Usage

```shell
ros2 launch chatbot_bringup chatbot.launch.py
```
