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


from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard
from llama_msgs.action import GenerateResponse


class LlamaState(ActionState):

    def __init__(self) -> None:

        super().__init__(
            GenerateResponse, "/llama/generate_response",
            self.create_llama_goal,
            result_handler=self.handle_result
        )

    def create_llama_goal(self, blackboard: Blackboard) -> GenerateResponse.Goal:
        goal = GenerateResponse.Goal()

        promtp = f"""<|im_start|>system
You are an AI assistant that follows instruction extremely well. Help as much as you can.<|im_end|>
<|im_start|>user
{blackboard.stt}<|im_end|>
<|im_start|>assistant
"""

        goal.prompt = promtp
        goal.reset = True
        return goal

    def handle_result(
        self,
        blackboard: Blackboard,
        result: GenerateResponse.Result
    ) -> str:
        blackboard.tts = result.response.text.strip()
        return SUCCEED
