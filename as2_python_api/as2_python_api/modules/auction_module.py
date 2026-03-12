"""Auction Module."""

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

import typing

from as2_msgs.action import Auction
from as2_msgs.msg import AuctionItem
from as2_python_api.behavior_actions.auction_handler import AuctionBehavior
from as2_python_api.modules.module_base import ModuleBase

if typing.TYPE_CHECKING:
    from ..drone_interface import DroneInterface


class AuctionModule(ModuleBase, AuctionBehavior):
    """Auction Module."""

    __alias__ = 'auction'

    def __init__(self, drone: 'DroneInterface') -> None:
        super().__init__(drone, self.__alias__)

    def __call__(
        self,
        name: str,
        elements: list[AuctionItem],
        auction_type: str,
        bidders: list[str],
        wait: bool = True,
    ) -> bool:
        """
        Run an auction.

        :param name: auction name
        :type name: str
        :param elements: items to be auctioned
        :type elements: List[AuctionItem]
        :param auction_type: auction type
        :type auction_type: str
        :param bidders: list of bidder names
        :type bidders: List[str]
        :param wait: blocking call, defaults to True
        :type wait: bool, optional
        :return: True if accepted, False otherwise
        :rtype: bool
        """
        return self.__auction(name, elements, auction_type, bidders, wait)

    def __auction(
        self,
        name: str,
        elements: list[AuctionItem],
        auction_type: str,
        bidders: list[str],
        wait: bool = True,
    ) -> bool:
        goal_msg = Auction.Goal()
        goal_msg.name = name
        goal_msg.elements = elements
        goal_msg.type = auction_type
        goal_msg.bidders = bidders
        self._node.get_logger().info(
            f'Starting auction with name: {name}, type: {auction_type}, and bidders: {bidders}'
        )
        try:
            return self.start(goal_msg, wait)
        except self.GoalRejected as err:
            self._node.get_logger().warn(str(err))
        return False

    # Method simplifications
    def run_auction(
        self,
        name: str,
        elements: list[AuctionItem],
        auction_type: str,
        bidders: list[str],
    ) -> bool:
        """
        Run an auction, blocking call.

        :param name: auction name
        :type name: str
        :param elements: items to be auctioned
        :type elements: List[AuctionItem]
        :param auction_type: auction type
        :type auction_type: str
        :param bidders: list of bidder names
        :type bidders: List[str]
        :return: True if accepted, False otherwise
        :rtype: bool
        """
        return self.__auction(name, elements, auction_type, bidders, wait=True)

    def run_auction_async(
        self,
        name: str,
        elements: list[AuctionItem],
        auction_type: str,
        bidders: list[str],
    ) -> bool:
        """
        Run an auction, non-blocking call.

        :param name: auction name
        :type name: str
        :param elements: items to be auctioned
        :type elements: List[AuctionItem]
        :param auction_type: auction type
        :type auction_type: str
        :param bidders: list of bidder names
        :type bidders: List[str]
        :return: True if accepted, False otherwise
        :rtype: bool
        """
        return self.__auction(name, elements, auction_type, bidders, wait=False)
