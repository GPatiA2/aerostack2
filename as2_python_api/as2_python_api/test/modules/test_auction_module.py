"""Test AuctionModule."""

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

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2022 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import unittest
from unittest.mock import MagicMock, patch

from as2_msgs.msg import AuctionItem
from as2_python_api.behavior_actions.behavior_handler import BehaviorHandler
from as2_python_api.modules.auction_module import AuctionModule


def make_mock_drone():
    """Create a minimal mock DroneInterface."""
    drone = MagicMock()
    drone.modules = {}
    return drone


class TestAuctionModule(unittest.TestCase):
    """Unit tests for AuctionModule."""

    def setUp(self):
        """Set up test fixtures, bypassing ROS2 initialization."""
        self.drone = make_mock_drone()
        with patch.object(BehaviorHandler, '__init__', return_value=None):
            self.module = AuctionModule(self.drone)
        # BehaviorHandler.__init__ was skipped, so set _node manually
        self.module._node = self.drone

    # ------------------------------------------------------------------
    # Basic attributes
    # ------------------------------------------------------------------

    def test_alias(self):
        """Module alias must be 'auction'."""
        self.assertEqual(self.module.__alias__, 'auction')

    def test_module_registered_in_drone(self):
        """Module must register itself in drone.modules under its alias."""
        self.assertIn('auction', self.drone.modules)
        self.assertIs(self.drone.modules['auction'], self.module)

    # ------------------------------------------------------------------
    # __call__ / __auction
    # ------------------------------------------------------------------

    def test_call_returns_true_on_success(self):
        """__call__ must return True when start() succeeds."""
        items = [AuctionItem()]
        with patch.object(self.module, 'start', return_value=True):
            result = self.module('my_auction', items, 'greedy', ['drone1'])
        self.assertTrue(result)

    def test_call_returns_false_on_goal_rejected(self):
        """__call__ must return False when GoalRejected is raised."""
        items = [AuctionItem()]
        with patch.object(
            self.module, 'start', side_effect=BehaviorHandler.GoalRejected('rejected')
        ):
            result = self.module('my_auction', items, 'greedy', ['drone1'])
        self.assertFalse(result)

    def test_call_builds_correct_goal_message(self):
        """__call__ must populate the goal message fields correctly."""
        items = [AuctionItem(), AuctionItem()]
        bidders = ['drone1', 'drone2']
        captured = {}

        def capture(goal_msg, wait):
            captured['goal'] = goal_msg
            return True

        with patch.object(self.module, 'start', side_effect=capture):
            self.module('test_name', items, 'type_a', bidders)

        goal = captured['goal']
        self.assertEqual(goal.name, 'test_name')
        self.assertEqual(goal.type, 'type_a')
        self.assertEqual(goal.bidders, bidders)
        self.assertEqual(len(goal.elements), 2)

    def test_call_default_wait_is_true(self):
        """__call__ must pass wait=True to start() by default."""
        with patch.object(self.module, 'start', return_value=True) as mock_start:
            self.module('name', [], 'type', ['b1'])
        _goal_msg, wait_arg = mock_start.call_args[0]
        self.assertTrue(wait_arg)

    def test_call_wait_false_propagated(self):
        """__call__ must forward wait=False to start()."""
        with patch.object(self.module, 'start', return_value=True) as mock_start:
            self.module('name', [], 'type', ['b1'], wait=False)
        _goal_msg, wait_arg = mock_start.call_args[0]
        self.assertFalse(wait_arg)

    # ------------------------------------------------------------------
    # run_auction (blocking)
    # ------------------------------------------------------------------

    def test_run_auction_is_blocking(self):
        """run_auction must call start() with wait=True."""
        with patch.object(self.module, 'start', return_value=True) as mock_start:
            result = self.module.run_auction('name', [AuctionItem()], 'type', ['b1'])
        self.assertTrue(result)
        _goal_msg, wait_arg = mock_start.call_args[0]
        self.assertTrue(wait_arg)

    def test_run_auction_returns_false_on_rejection(self):
        """run_auction must return False when GoalRejected is raised."""
        with patch.object(
            self.module, 'start', side_effect=BehaviorHandler.GoalRejected('rejected')
        ):
            result = self.module.run_auction('name', [], 'type', ['b1'])
        self.assertFalse(result)

    # ------------------------------------------------------------------
    # run_auction_async (non-blocking)
    # ------------------------------------------------------------------

    def test_run_auction_async_is_non_blocking(self):
        """run_auction_async must call start() with wait=False."""
        with patch.object(self.module, 'start', return_value=True) as mock_start:
            result = self.module.run_auction_async('name', [AuctionItem()], 'type', ['b1'])
        self.assertTrue(result)
        _goal_msg, wait_arg = mock_start.call_args[0]
        self.assertFalse(wait_arg)

    def test_run_auction_async_returns_false_on_rejection(self):
        """run_auction_async must return False when GoalRejected is raised."""
        with patch.object(
            self.module, 'start', side_effect=BehaviorHandler.GoalRejected('rejected')
        ):
            result = self.module.run_auction_async('name', [], 'type', ['b1'])
        self.assertFalse(result)

    def test_get_plan_item_default_method(self):
        """get_plan_item with no method override must use __call__."""
        plan_item = AuctionModule.get_plan_item(
            None,
            name='auction_1',
            elements=[],
            auction_type='greedy',
            bidders=['b1', 'b2'],
        )
        self.assertEqual(plan_item.behavior, 'auction')
        self.assertEqual(plan_item.method, '__call__')

    def test_get_plan_item_run_auction_method(self):
        """get_plan_item must record the specified method name."""
        plan_item = AuctionModule.get_plan_item(
            AuctionModule.run_auction,
            name='auction_2',
            elements=[],
            auction_type='greedy',
            bidders=['b1'],
        )
        self.assertEqual(plan_item.behavior, 'auction')
        self.assertEqual(plan_item.method, 'run_auction')


if __name__ == '__main__':
    unittest.main()
