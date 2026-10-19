# chatbot_ros

<div align="center">

[![License: MIT](https://img.shields.io/badge/GitHub-GPL--3.0-informational)](https://opensource.org/license/gpl-3-0) [![GitHub release](https://img.shields.io/github/release/mgonzs13/chatbot_ros.svg)](https://github.com/mgonzs13/chatbot_ros/releases) [![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/chatbot_ros.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros?branch=main) [![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/chatbot_ros.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/commits/main) [![GitHub issues](https://img.shields.io/github/issues/mgonzs13/chatbot_ros)](https://github.com/mgonzs13/chatbot_ros/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/chatbot_ros)](https://github.com/mgonzs13/chatbot_ros/pulls) [![Contributors](https://img.shields.io/github/contributors/mgonzs13/chatbot_ros.svg)](https://github.com/mgonzs13/chatbot_ros/graphs/contributors) [![C++ Formatter Check](https://github.com/mgonzs13/chatbot_ros/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/cpp-formatter.yml?branch=main) [![Python Formatter Check](https://github.com/mgonzs13/chatbot_ros/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/python-formatter.yml?branch=main)

| ROS 2 Distro |                           Branch                            |                                                                                                         Build status                                                                                                         | Docker Image | Documentation |
| :----------: | :---------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: | :----------: | ------------- |
|  **Humble**  | [`main`](https://github.com/mgonzs13/chatbot_ros/tree/main) |  [![Humble Build](https://github.com/mgonzs13/chatbot_ros/actions/workflows/humble-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/humble-docker-build.yml?branch=main)   |      -       | -             |
|  **Jazzy**   | [`main`](https://github.com/mgonzs13/chatbot_ros/tree/main) |    [![Jazzy Build](https://github.com/mgonzs13/chatbot_ros/actions/workflows/jazzy-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/jazzy-docker-build.yml?branch=main)    |      -       | -             |
| **Lyrical**  | [`main`](https://github.com/mgonzs13/chatbot_ros/tree/main) | [![Lyrical Build](https://github.com/mgonzs13/chatbot_ros/actions/workflows/lyrical-docker-build.yml/badge.svg?branch=main)](https://github.com/mgonzs13/chatbot_ros/actions/workflows/Lyrical-docker-build.yml?branch=main) |      -       | -             |

</div>

## Table of Contents

1. [Dependencies](#dependencies)
2. [Installation](#installation)
3. [Usage](#usage)
4. [Features](#features)
5. [Configuration](#configuration)

## Dependencies

- [yasmin](https://github.com/uleroboticsgroup/yasmin)[[6.1.2](https://github.com/uleroboticsgroup/yasmin/releases/tag/6.1.2)]
- [audio_common](https://github.com/mgonzs13/audio_common)[[4.0.11](https://github.com/mgonzs13/audio_common/releases/tag/4.0.11)]
- [llama_ros](https://github.com/mgonzs13/llama_ros)[[6.2.2](https://github.com/mgonzs13/llama_ros/releases/tag/6.2.2)]
- [whisper_ros](https://github.com/mgonzs13/whisper_ros)[[4.0.10](https://github.com/mgonzs13/whisper_ros/releases/tag/4.0.10)]
- [piper_ros](https://github.com/mgonzs13/piper_ros)[[2.0.5](https://github.com/mgonzs13/piper_ros/releases/tag/2.0.5)]

## Installation

Dependencies can be installed using [vcstool](https://github.com/dirk-thomas/vcstool). Then, you can follow the next steps to install all tools and the chatbot:

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/chatbot_ros
vcs import < chatbot_ros/dependencies.repos
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DGGML_CUDA=ON
```

## Usage

```shell
ros2 launch chatbot_bringup chatbot.launch.py
```

Launches Whisper (STT), llama_ros (LLM), Piper (TTS) and the chatbot

## Features

### Conversation memory

- **Short-term memory:** the most recent turns (`history.max_turns`) are sent
  back to the LLM with every request, so the chatbot remembers the ongoing
  conversation.
- **Long-term memory (optional):** past turns are embedded with a
  `llama_ros` embedding model and stored in a small vector database on disk
  (`$HOME/.cache/chatbot_ros/memory.json`). The most similar memories are
  injected into the system prompt of new requests. Enable it with:

```shell
ros2 launch chatbot_bringup chatbot.launch.py memory:=true
```

This launches a second `llama_ros` node (`embedding.yaml`, bge-base-en-v1.5)
in the `embedding` namespace. The model is downloaded on the first run. The
memory is best-effort: if the embeddings service is not reachable the chatbot
keeps working without long-term memory.

### Voice-friendly answers

- The system prompt is fully configurable and can be loaded from a file.
- Responses are sanitized before being spoken: markdown (bold, lists,
  headings, code blocks, tables), emoji, links and other non-natural text are
  removed so the TTS only says natural language.
- Whisper transcriptions are filtered: non-speech annotations such as
  `[BLANK_AUDIO]`, `(music)`, `♪ ... ♪` and hallucinated phrases
  ("Thank you for watching!") are discarded and the chatbot asks the user to
  repeat instead.

## Configuration

All parameters live in
[`chatbot_bringup/config/chatbot.yaml`](chatbot_bringup/config/chatbot.yaml)
and can be overridden at launch (e.g.
`ros2 launch chatbot_bringup chatbot.launch.py` after editing the file).

| Parameter                     | Type     | Default                          | Description                                                         |
| ----------------------------- | -------- | -------------------------------- | ------------------------------------------------------------------- |
| `messages.greeting`           | string   | `"Hi, how can I help you?"`      | Message spoken at startup (empty disables the greeting)             |
| `messages.system_prompt`      | string   | voice-oriented prompt            | System prompt sent with every request (empty disables it)           |
| `messages.system_prompt_file` | string   | `""`                             | Load the system prompt from this file when `system_prompt` is empty |
| `messages.retry_messages`     | string[] | 3 messages                       | Spoken in round-robin when no speech was recognized                 |
| `stt.prompt`                  | string   | `""`                             | Initial prompt passed to Whisper                                    |
| `stt.ignored_phrases`         | string[] | hallucination phrases            | Transcriptions containing these are treated as noise                |
| `llm.temperature`             | double   | `0.4`                            | Sampling temperature for the chat completions                       |
| `history.max_turns`           | int      | `8`                              | User/assistant turn pairs kept in the short-term memory             |
| `memory.enabled`              | bool     | `false`                          | Enable the embeddings-based long-term memory                        |
| `memory.service`              | string   | `/embedding/generate_embeddings` | Embeddings service used by the long-term memory                     |
| `memory.storage_file`         | string   | `$HOME/.cache/chatbot_ros/...`   | JSON file where memories are persisted                              |
| `memory.retrieve_k`           | int      | `3`                              | Number of memories injected into the system prompt                  |
| `memory.min_similarity`       | double   | `0.55`                           | Minimum cosine similarity for a memory to be retrieved              |
| `memory.max_entries`          | int      | `512`                            | Maximum number of memories kept on disk                             |

The language model and its context length are configured in
[`chatbot_bringup/config/llama.yaml`](chatbot_bringup/config/llama.yaml).
The embedding model used by the long-term memory is configured in
[`chatbot_bringup/config/embedding.yaml`](chatbot_bringup/config/embedding.yaml).
