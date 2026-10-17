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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1"
    )

    chatbot_params_file = LaunchConfiguration(
        "chatbot_params_file",
        default=os.path.join(
            get_package_share_directory("chatbot_bringup"),
            "config",
            "chatbot.yaml",
        ),
    )

    memory_cmd = DeclareLaunchArgument(
        "memory",
        default_value="false",
        description="Launch an embedding model for the long-term memory",
    )

    whisper_params_file = LaunchConfiguration(
        "whisper_params_file",
        default=os.path.join(
            get_package_share_directory("chatbot_bringup"),
            "config",
            "whisper.yaml",
        ),
    )

    llama_params_file = LaunchConfiguration(
        "llama_params_file",
        default=os.path.join(
            get_package_share_directory("chatbot_bringup"),
            "config",
            "llama.yaml",
        ),
    )

    memory_params_file = LaunchConfiguration(
        "memory_params_file",
        default=os.path.join(
            get_package_share_directory("chatbot_bringup"),
            "config",
            "embedding.yaml",
        ),
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
            "whisper_params_file": whisper_params_file,
        }.items(),
    )

    llama_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("llama_bringup"),
                "launch",
                "base.launch.py",
            )
        ),
        launch_arguments={
            "params_file": llama_params_file,
            "node_name": "llama_node",
            "namespace": "llama",
        }.items(),
    )

    embedding_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("llama_bringup"),
                "launch",
                "base.launch.py",
            )
        ),
        launch_arguments={
            "params_file": memory_params_file,
            "node_name": "llama_embedding_node",
            "namespace": "llama",
        }.items(),
        condition=IfCondition(LaunchConfiguration("memory")),
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
        }.items(),
    )

    chatbot_node_cmd = Node(
        package="chatbot_ros",
        executable="chat_bot_node",
        output="both",
        parameters=[chatbot_params_file],
    )

    yasmin_viewer_cmd = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        output="both",
        parameters=[{"host": "0.0.0.0"}],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(memory_cmd)

    ld.add_action(whisper_cmd)
    ld.add_action(piper_node_cmd)
    ld.add_action(llama_cmd)
    ld.add_action(embedding_cmd)
    ld.add_action(chatbot_node_cmd)
    ld.add_action(yasmin_viewer_cmd)

    return ld
