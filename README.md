# chatbot_ros

## Dependencies

- [yasmin](https://github.com/uleroboticsgroup/yasmin)[[2.2.0](https://github.com/uleroboticsgroup/yasmin/releases/tag/2.2.0)]
- [audio_common](https://github.com/mgonzs13/audio_common)[[3.0.2](https://github.com/mgonzs13/audio_common/releases/tag/3.0.2)]
- [llama_ros](https://github.com/mgonzs13/llama_ros)[[2.5.2](https://github.com/mgonzs13/llama_ros/releases/tag/2.5.2)]
- [whiser_ros](https://github.com/mgonzs13/whisper_ros)[[1.2.0](https://github.com/mgonzs13/whisper_ros/releases/tag/1.2.0)]
- [tts_ros](https://github.com/mgonzs13/tts_ros)[[2.3.1](https://github.com/mgonzs13/tts_ros/releases/tag/2.3.1)]

## Installation

```shell
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/chatbot_ros
$ cd ~/ros2_ws
$ colcon build
```

## Usage

```shell
$ ros2 launch chatbot_bringup chatbot.launch.py
```
