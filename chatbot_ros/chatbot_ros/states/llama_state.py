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

        self._response = ""
        self._partial_text = ""
        self._full_response = ""
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
        return goal

    def handle_result(
        self,
        blackboard: Blackboard,
        result: GenerateResponse.Result
    ) -> str:

        blackboard.response = result.response.text
        self._response = result.response.text

        self._tts_end_event.clear()
        self._tts_end_event.wait()

        self._response = ""
        self._partial_text = ""
        self._full_response = ""

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
                self.say(text)

            self._full_response += text

        if self._full_response == self._response:
            self._tts_end_event.set()

    def say(self, text: str) -> None:
        goal = TTS.Goal()
        goal.text = text
        self._tts_client.wait_for_server()
        self._tts_client.send_goal(goal)
