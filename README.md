# chatbot_ros

## Dependencies

- [yasmin](https://github.com/uleroboticsgroup/yasmin)[[2.4.2](https://github.com/uleroboticsgroup/yasmin/releases/tag/2.4.2)]
- [audio_common](https://github.com/mgonzs13/audio_common)[[3.0.2](https://github.com/mgonzs13/audio_common/releases/tag/3.0.2)]
- [llama_ros](https://github.com/mgonzs13/llama_ros)[[4.0.1](https://github.com/mgonzs13/llama_ros/releases/tag/4.0.1)]
- [whisper_ros](https://github.com/mgonzs13/whisper_ros)[[2.6.2](https://github.com/mgonzs13/whisper_ros/releases/tag/2.6.2)]
- [tts_ros](https://github.com/mgonzs13/tts_ros)[[2.3.1](https://github.com/mgonzs13/tts_ros/releases/tag/2.3.1)]

## Installation

Dependencies can be installed using [vcstool](https://github.com/dirk-thomas/vcstool). Then, you can follow the next steps to install all tools and the chatbot:

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/chatbot_ros
$ vcs import < chatbot_ros/dependencies.repos
$ pip3 install -r requirements.txt
$ cd ~/ros2_ws
$ rosdep install --from-paths src --ignore-src -r -y
$ colcon build --cmake-args -DGGML_CUDA=ON
```

## Usage

```shell
$ ros2 launch chatbot_bringup chatbot.launch.py
```
