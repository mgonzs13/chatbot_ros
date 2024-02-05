#!/usr/bin/env python3

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


import rclpy
from simple_node import Node

from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from chatbot_ros.states import ListenFSM
from chatbot_ros.states import LlamaState
from chatbot_ros.states import SpeakState


class ChatBotNode(Node):

    def __init__(self) -> None:
        super().__init__("chat_bot_node")

        # create a state machine
        self.sm = StateMachine(outcomes=[CANCEL, ABORT])

        # add states
        self.sm.add_state(
            "GREETING",
            SpeakState(self),
            transitions={
                SUCCEED: "LISTENING",
                ABORT: ABORT,
                CANCEL: CANCEL
            }
        )

        self.sm.add_state(
            "LISTENING",
            ListenFSM(self),
            transitions={
                SUCCEED: "GENERATING_RESPONSE",
                ABORT: ABORT,
                CANCEL: CANCEL
            }
        )

        self.sm.add_state(
            "GENERATING_RESPONSE",
            LlamaState(self),
            transitions={
                SUCCEED: "SPEAKING",
                ABORT: ABORT,
                CANCEL: CANCEL
            }
        )

        self.sm.add_state(
            "SPEAKING",
            SpeakState(self),
            transitions={
                SUCCEED: "LISTENING",
                ABORT: ABORT,
                CANCEL: CANCEL
            }
        )

        YasminViewerPub(self, "CHAT_BOT", self.sm)

    def execute_chat_bot(self) -> None:
        blackboard = Blackboard()
        blackboard.tts = "Hi, how can I help you"
        self.sm(blackboard)


def main(args=None):
    rclpy.init(args=args)
    node = ChatBotNode()
    node.execute_chat_bot()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
