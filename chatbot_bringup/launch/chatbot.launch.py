# Copyright (C) 2023 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from llama_bringup.utils import create_llama_launch


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("whisper_bringup"),
                "launch",
                "whisper.launch.py",
            )
        ),
        launch_arguments={
            "launch_audio_capturer": LaunchConfiguration(
                "launch_audio_capturer", default=True
            ),
            "model_repo": "ggerganov/whisper.cpp",
            "model_filename": "ggml-large-v3-turbo-q5_0.bin",
        }.items(),
    )

    llama_cmd = create_llama_launch(
        n_ctx=4096,
        n_batch=256,
        n_gpu_layers=-1,
        n_threads=-1,
        n_predict=-1,
        model_repo="Qwen/Qwen2.5-Coder-3B-Instruct-GGUF",
        model_filename="qwen2.5-coder-3b-instruct-q4_k_m.gguf",
        system_prompt_type="ChatML",
    )

    piper_node_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("piper_bringup"),
                "launch",
                "piper.launch.py",
            )
        ),
        launch_arguments={
            "launch_audio_player": LaunchConfiguration(
                "launch_audio_player", default=True
            ),
            "model_repo": "rhasspy/piper-voices",
            "model_filename": "en/en_US/lessac/low/en_US-lessac-low.onnx",
            "config_model_repo": "rhasspy/piper-voices",
            "config_model_filename": "en/en_US/lessac/low/en_US-lessac-low.onnx.json",
        }.items(),
    )

    chatbot_node_cmd = Node(
        package="chatbot_ros",
        executable="chat_bot_node",
        output="both",
    )

    yasmin_viewer_cmd = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="both",
        parameters=[{"host": "0.0.0.0"}],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(whisper_cmd)
    ld.add_action(piper_node_cmd)
    ld.add_action(llama_cmd)
    ld.add_action(chatbot_node_cmd)
    ld.add_action(yasmin_viewer_cmd)

    return ld
