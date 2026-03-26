"""KB Mission Monitor ROS 2 node."""

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

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import json
from typing import Callable

import rclpy
from as2_msgs.msg import MissionUpdate
from as2_python_api.kb_monitor.kb_event_handler import _QueryHelper
from as2_python_api.mission_interpreter.mission import InterpreterStatus
from kb_msgs.srv import Event
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import String


class KBMissionMonitor(Node):
    """Programmatic base for monitoring KB events and driving a mission interpreter.

    Unlike :class:`~kb_event_handler.KBMonitorNode` (which is config-file driven),
    this class is intended to be used directly in code: instantiate it, call
    :meth:`subscribe_to_event` for each pattern you want to watch, then spin.

    Example
    -------
    .. code-block:: python

        rclpy.init()
        monitor = KBMissionMonitor('drone0')

        def on_stuck(bindings):
            msg = MissionUpdate()
            msg.drone_id = 'drone0'
            msg.action = MissionUpdate.PAUSE
            monitor.publish_mission_update(msg)

        monitor.subscribe_to_event(['?drone is_stuck true'], on_stuck)
        rclpy.spin(monitor)
    """

    def __init__(self, drone_namespace: str, kb_namespace: str = 'kb') -> None:
        super().__init__('kb_mission_monitor')
        self.drone_namespace = drone_namespace
        self.kb_namespace = kb_namespace

        # Knowledge Base publishers
        self.add_fact_pub: Publisher = self.create_publisher(
            String, f'/{drone_namespace}/{kb_namespace}/add_fact', 10
        )
        self.remove_fact_pub: Publisher = self.create_publisher(
            String, f'/{drone_namespace}/{kb_namespace}/remove_fact', 10
        )

        # Knowledge Base events service client
        self._events_cli = self.create_client(
            Event, f'/{drone_namespace}/{kb_namespace}/events'
        )

        # Dedicated query helper (own thread — safe to call from callbacks)
        self._query_helper = _QueryHelper(f'/{drone_namespace}/{kb_namespace}/query')

        # Keep hard references to avoid garbage-collection of subscriptions
        self._event_subs: list[Subscription] = []

        # Mission interpreter publisher / status subscriber
        self.mission_update_pub: Publisher = self.create_publisher(
            MissionUpdate, f'/{drone_namespace}/mission_update', 10
        )
        self.mission_status_sub: Subscription = self.create_subscription(
            String, f'/{drone_namespace}/mission_status',
            self._mission_status_callback, 10
        )
        self.mission_status: InterpreterStatus | None = None

    def destroy_node(self) -> None:
        """Clean up the query helper thread before destroying the node."""
        self._query_helper.destroy()
        super().destroy_node()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def query(
        self,
        patterns: list[str],
        vars: list[str] | None = None,
        models: list[str] | None = None,
        timeout_sec: float = 5.0,
    ) -> list[dict]:
        """Query the drone's knowledge base synchronously.

        Safe to call from inside a :meth:`subscribe_to_event` callback.

        Parameters
        ----------
        patterns:
            RDF triple patterns with ``?``-prefixed variables,
            e.g. ``['?drone rdf:type Robot', '?drone has_battery ?level']``.
        vars:
            Variables to return; ``None`` returns all bound variables.
        models:
            KB models to search; ``None`` / ``[]`` means all models.
        timeout_sec:
            Maximum seconds to wait for a response.

        Returns
        -------
        list[dict]
            List of variable-binding dicts.  Empty list on timeout or error.
        """
        return self._query_helper.query(patterns, vars, models, timeout_sec)

    def subscribe_to_event(
        self,
        patterns: list[str],
        callback: Callable[[list[dict]], None],
        one_shot: bool = False,
        models: list[str] | None = None,
    ) -> bool:
        """Register a KB event and wire it to a callback function.

        Calls the knowledge_core ``/events`` service synchronously, obtains the
        event topic, and creates a ROS 2 subscription.  Must be called *before*
        the node is handed to an executor (i.e. before ``rclpy.spin``).

        Parameters
        ----------
        patterns:
            RDF triple patterns with ``?``-prefixed variables,
            e.g. ``['?drone is_stuck true']``.
        callback:
            Function called on every trigger.
            Signature: ``callback(bindings: list[dict]) -> None``.
        one_shot:
            Remove the KB subscription after the first trigger.
        models:
            KB models to watch; ``None`` / ``[]`` means all models.

        Returns
        -------
        bool
            ``True`` if the subscription was successfully created.
        """
        events_srv = f'/{self.drone_namespace}/{self.kb_namespace}/events'

        if not self._events_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'KB events service "{events_srv}" is not available.'
            )
            return False

        req = Event.Request()
        req.patterns = patterns
        req.one_shot = one_shot
        req.models = models if models is not None else []

        future = self._events_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done() or future.result() is None:
            self.get_logger().error(
                f'Service call to "{events_srv}" timed out or failed '
                f'for patterns {patterns}.'
            )
            return False

        result: Event.Response = future.result()  # type: ignore
        if not result.id:
            self.get_logger().error(
                f'KB returned an empty event id for patterns {patterns}; skipping.'
            )
            return False

        event_topic: str = result.topic
        self.get_logger().info(
            f'Subscribed to KB event "{result.id}" on topic "{event_topic}".'
        )

        def _cb(msg: String, _callback=callback) -> None:
            try:
                bindings: list[dict] = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().error(
                    f'Failed to decode KB event payload: {exc}; raw: "{msg.data}"'
                )
                return
            try:
                _callback(bindings)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'KB event callback raised: {exc}')

        sub = self.create_subscription(String, event_topic, _cb, 10)
        self._event_subs.append(sub)
        return True

    def publish_mission_update(self, msg: MissionUpdate) -> None:
        """Publish a :class:`MissionUpdate` to the drone's mission interpreter."""
        self.mission_update_pub.publish(msg)

    def add_fact(self, fact: str) -> None:
        """Add an RDF triple to the drone's knowledge base."""
        msg = String()
        msg.data = fact
        self.add_fact_pub.publish(msg)

    def remove_fact(self, fact: str) -> None:
        """Remove an RDF triple from the drone's knowledge base."""
        msg = String()
        msg.data = fact
        self.remove_fact_pub.publish(msg)

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _mission_status_callback(self, msg: String) -> None:
        try:
            self.mission_status = InterpreterStatus.parse_raw(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'Failed to decode mission status: {exc}')
