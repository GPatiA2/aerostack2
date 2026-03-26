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

"""Parameter dataclasses for the KB event monitor."""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from dataclasses import dataclass, field
from typing import Callable


@dataclass
class EventHandlerParams:
    """Parameters describing a KB event and its associated handler function.

    Attributes
    ----------
    handler_id : str
        Unique name for this handler (used in the config file to reference it).
    patterns : list[str]
        RDF triple patterns with variables (starting with ``?``) that define the
        event, e.g. ``['?drone is_stuck true']``.
    one_shot : bool
        If ``True`` the KB removes the subscription after the first trigger.
    models : list[str]
        KB models to watch (empty list means all models).
    handler : Callable
        Python function called when the event fires.
        Expected signature::

            def my_handler(bindings: list[dict], ctx: KBHandlerContext) -> None: ...

        ``bindings`` is the JSON-decoded list of variable-binding dicts published
        by knowledge_core; ``ctx`` exposes ROS 2 publishing helpers.
    """

    handler_id: str
    patterns: list[str]
    one_shot: bool
    models: list[str]
    handler: Callable
