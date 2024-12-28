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


from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from chatbot_ros.states import ListenState
from chatbot_ros.states import CheckSttState
from chatbot_ros.states import SpeakState


class ListenFSM(StateMachine):

    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED, CANCEL, ABORT])

        self.add_state(
            "LISTENING",
            ListenState(),
            transitions={
                SUCCEED: "CHECKING_STT",
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )

        self.add_state(
            "CHECKING_STT",
            CheckSttState(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: "COMPLAINING",
            },
        )

        self.add_state(
            "COMPLAINING",
            SpeakState(),
            transitions={
                SUCCEED: "LISTENING",
                ABORT: ABORT,
                CANCEL: CANCEL,
            },
        )
