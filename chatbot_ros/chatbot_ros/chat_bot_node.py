#!/usr/bin/env python3

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


import rclpy

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from chatbot_ros.states import ListenFSM
from chatbot_ros.states import LlamaState
from chatbot_ros.states import SpeakState


class ChatBot:

    def __init__(self) -> None:

        # create a state machine
        self.sm = StateMachine(outcomes=[CANCEL, ABORT])

        # add states
        self.sm.add_state(
            "GREETING",
            SpeakState(),
            transitions={
                SUCCEED: "LISTENING",
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )

        self.sm.add_state(
            "LISTENING",
            ListenFSM(),
            transitions={
                SUCCEED: "RESPONDING",
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )

        self.sm.add_state(
            "RESPONDING",
            LlamaState(),
            transitions={
                SUCCEED: "LISTENING",
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )

        YasminViewerPub("CHAT_BOT", self.sm)

    def execute_chat_bot(self) -> None:
        blackboard = Blackboard()
        blackboard.tts = "Hi, how can I help you?"
        self.sm(blackboard)


def main():
    rclpy.init()
    set_ros_loggers()
    chatbot = ChatBot()
    chatbot.execute_chat_bot()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
