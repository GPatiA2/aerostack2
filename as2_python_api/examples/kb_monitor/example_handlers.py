# Copyright 2026 Universidad Politécnica de Madrid
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

"""Example KB event handler functions.

Each function must follow this signature::

    def my_handler(bindings: list[dict], ctx: KBHandlerContext) -> None: ...

``bindings``
    List of variable-binding dicts published by knowledge_core when the event
    fires, e.g. ``[{'drone': 'drone0'}, {'drone': 'drone1'}]``.

``ctx``
    :class:`~as2_python_api.kb_monitor.kb_event_handler.KBHandlerContext`
    instance.  Provides:

    * ``ctx.drone_namespace`` — the drone this handler is registered for.
    * ``ctx.publish_mission_update(msg)`` — send a :class:`MissionUpdate`.
    * ``ctx.add_fact(triple)`` — add an RDF triple to the KB.
    * ``ctx.remove_fact(triple)`` — remove an RDF triple from the KB.
"""

from as2_msgs.msg import MissionUpdate

from as2_python_api.kb_monitor.kb_event_handler import KBHandlerContext


def handle_drone_stuck(bindings: list[dict], ctx: KBHandlerContext) -> None:
    """Pause the current mission when the drone reports being stuck.

    Expected KB event pattern: ``?drone is_stuck true``
    """
    for binding in bindings:
        drone = binding.get('drone', ctx.drone_namespace)

        msg = MissionUpdate()
        msg.drone_id = drone
        msg.action = MissionUpdate.PAUSE
        ctx.publish_mission_update(msg)

        # Mark in the KB that we reacted so other rules are not re-triggered.
        ctx.add_fact(f'{drone} stuck_acknowledged true')


def handle_target_found(bindings: list[dict], ctx: KBHandlerContext) -> None:
    """Insert a go-to-target item into the running mission when a target is found.

    Expected KB event pattern: ``?drone has_found ?target``
    """
    import json

    from as2_python_api.mission_interpreter.mission import MissionItem

    for binding in bindings:
        drone = binding.get('drone', ctx.drone_namespace)
        target = binding.get('target')
        if target is None:
            continue

        # Build a minimal mission item that moves toward the found target.
        # Adjust action name and parameters to match your actual behaviour stack.
        item = MissionItem(
            action='go_to',
            args={'target': target},
        )

        insert_msg = MissionUpdate()
        insert_msg.drone_id = drone
        insert_msg.action = MissionUpdate.INSERT
        insert_msg.mission = json.dumps({'items': [item.dict()]})
        ctx.publish_mission_update(insert_msg)


def handle_mission_complete(bindings: list[dict], ctx: KBHandlerContext) -> None:
    """Land the drone when the KB signals that the mission objective is met.

    Expected KB event pattern: ``?drone mission_complete true``
    """
    for binding in bindings:
        drone = binding.get('drone', ctx.drone_namespace)

        stop_msg = MissionUpdate()
        stop_msg.drone_id = drone
        stop_msg.action = MissionUpdate.STOP
        ctx.publish_mission_update(stop_msg)
