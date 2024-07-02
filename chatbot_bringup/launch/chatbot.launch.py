# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from llama_bringup.utils import create_llama_launch
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")

    whisper_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                "whisper_bringup"), "launch", "whisper.launch.py")),
        launch_arguments={
            "launch_audio_capturer": LaunchConfiguration("launch_audio_capturer", default=True)
        }.items(),
    )

    llama_cmd = create_llama_launch(
        n_ctx=2048,
        n_batch=256,
        n_gpu_layers=33,
        n_threads=1,
        n_predict=-1,

        model_repo="lmstudio-community/Phi-3-mini-4k-instruct-GGUF",
        model_filename="Phi-3-mini-4k-instruct-IQ4_NL.gguf",

        system_prompt_type="Phi-3",
        debug=False
    )

    audio_player_cmd = Node(
        package="audio_common",
        executable="audio_player_node",
        name="player_node",
        namespace="audio",
        output="both",
        remappings=[("audio", "out")],
        condition=IfCondition(PythonExpression(
                [LaunchConfiguration("launch_audio_player", default=True)]))
    )

    tts_node_cmd = Node(
        package="tts_ros",
        executable="tts_node",
        output="both",
        parameters=[{
            "device": "cuda"
        }],
        remappings=[("audio", "/audio/out")]
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
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(whisper_cmd)
    ld.add_action(llama_cmd)
    ld.add_action(audio_player_cmd)
    ld.add_action(tts_node_cmd)
    ld.add_action(chatbot_node_cmd)
    ld.add_action(yasmin_viewer_cmd)

    return ld
