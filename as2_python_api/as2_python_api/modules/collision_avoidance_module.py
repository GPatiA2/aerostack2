"""Collision Avoidance Module."""

from __future__ import annotations

# Copyright 2022 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

__authors__ = 'Miguel Fernández Cortizas, Pedro Arias Pérez, David Pérez Saura, Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from typing import TYPE_CHECKING

from as2_python_api.behavior_actions.collision_avoidance_behavior import CollisionAvoidanceBehavior
from as2_python_api.modules.module_base import ModuleBase

if TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class CollisionAvoidanceModule(ModuleBase, CollisionAvoidanceBehavior):
    """Collision Avoidance Module."""

    __alias__ = 'collision_avoidance'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(self, x: float, y: float, z: float, speed: float,
                 safety_distance: float = 1.5, frame_id: str = 'earth',
                 wait: bool = True) -> bool:
        """Execute collision avoidance to a goal position.

        :param x: x coordinate (m) to go to
        :param y: y coordinate (m) to go to
        :param z: z coordinate (m) to go to
        :param speed: speed (m/s) to travel at
        :param safety_distance: safety distance (m) for conflict threshold
        :param frame_id: reference frame of the coordinates, defaults to "earth"
        :param wait: blocking call, defaults to True
        :return: True if was accepted, False otherwise
        """
        return self.start(x, y, z, speed, safety_distance, frame_id, wait)
