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


import threading

from yasmin_ros import ActionState
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from llama_msgs.action import GenerateResponse
from audio_common_msgs.action import TTS


class LlamaState(ActionState):

    def __init__(self) -> None:

        super().__init__(
            GenerateResponse, "/llama/generate_response",
            self.create_llama_goal,
            result_handler=self.handle_result,
            feedback_handler=self.handle_feedback
        )

        self._partial_text = ""
        self._total_texts = 0
        self._say_texts = 0
        self._tts_end_event = threading.Event()

        self._tts_client = ActionClient(
            YasminNode.get_instance(),
            TTS, "/say",
            callback_group=ReentrantCallbackGroup()
        )

    def create_llama_goal(self, blackboard: Blackboard) -> GenerateResponse.Goal:
        goal = GenerateResponse.Goal()
        goal.prompt = blackboard.stt
        goal.reset = True
        goal.sampling_config.temp = 0.0
        return goal

    def handle_result(
        self,
        blackboard: Blackboard,
        result: GenerateResponse.Result
    ) -> str:

        self._node.get_logger().info(result.response.text)
        self._node.get_logger().info(
            f"Total tokens: {len(result.response.tokens)}")

        self._tts_end_event.clear()
        self._tts_end_event.wait()

        self._partial_text = ""
        self._total_texts = 0
        self._say_texts = 0

        return SUCCEED

    def handle_feedback(
        self,
        blackboard: Blackboard,
        feedback: GenerateResponse.Feedback
    ) -> None:

        self._partial_text += feedback.partial_response.text

        if feedback.partial_response.text.strip().endswith(('.', '!', '?', ':')):

            text = self._partial_text
            self._partial_text = ""

            if text.strip():
                self._total_texts += 1
                self.say(text)

    def say(self, text: str) -> None:
        goal = TTS.Goal()
        goal.text = text
        self._tts_client.wait_for_server()

        send_goal_future = self._tts_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self._say_texts += 1

        if self._say_texts == self._total_texts:
            self._tts_end_event.set()
