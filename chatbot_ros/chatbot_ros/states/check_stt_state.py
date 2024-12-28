#!/usr/bin/env python3

# Copyright (C) 2023 Miguel Ángel González Santamarta

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


from yasmin import State
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT


class CheckSttState(State):

    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard) -> str:

        if blackboard.stt:
            return SUCCEED

        blackboard.tts = "I didn't understand youd. Can you repeat?"
        return ABORT
