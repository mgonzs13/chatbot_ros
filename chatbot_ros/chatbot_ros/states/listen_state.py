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


from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin.blackboard import Blackboard
from whisper_msgs.action import STT


class ListenState(ActionState):

    def __init__(self) -> None:

        super().__init__(
            STT,
            "/whisper/listen",
            self.create_whisper_goal,
            result_handler=self.handle_result,
        )

    def create_whisper_goal(self, blackboard: Blackboard) -> STT.Goal:
        goal = STT.Goal()
        return goal

    def handle_result(self, blackboard: Blackboard, result: STT.Result) -> str:

        blackboard.stt = result.transcription.text
        return SUCCEED
