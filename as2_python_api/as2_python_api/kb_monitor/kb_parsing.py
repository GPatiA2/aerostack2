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

"""JSON configuration parser for the KB event monitor."""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import importlib.util
import json
import sys
from typing import Callable

from as2_python_api.kb_monitor.kb_params import EventHandlerParams


class KBJSONParser:
    """Parse a JSON configuration file and produce per-drone handler lists.

    JSON structure
    --------------
    .. code-block:: json

        {
          "kb_namespace": "kb",
          "handlers": [
            {
              "id": "my_handler",
              "patterns": ["?drone is_stuck true"],
              "one_shot": false,
              "models": [],
              "func": {
                "path": "/abs/path/to/handlers.py",
                "name": "handlers_module",
                "func_name": "handle_stuck"
              }
            }
          ]
        }

    The drone namespace is not part of the config file; it is supplied at
    runtime via a command-line argument so that the same config can be reused
    for any drone without modification.

    Attributes
    ----------
    kb_namespace : str
        ROS namespace of the KB node relative to the drone namespace.
        Defaults to ``"kb"``.
    handlers : list[EventHandlerParams]
        Ordered list of all parsed handlers.
    """

    def __init__(self, file: str) -> None:
        with open(file) as f:
            info: dict = json.load(f)

        self.kb_namespace: str = info.get('kb_namespace', 'kb')

        self.handlers: list[EventHandlerParams] = []
        seen: set[str] = set()
        for handler_info in info.get('handlers', []):
            params: EventHandlerParams = self._parse_handler(handler_info)
            if params.handler_id in seen:
                print(f'WARNING: Handler "{params.handler_id}" is defined more than once; '
                      'overriding the previous definition.')
            seen.add(params.handler_id)
            self.handlers.append(params)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _parse_handler(self, info: dict) -> EventHandlerParams:
        handler_id: str = info['id']
        patterns: list[str] = info['patterns']
        one_shot: bool = info.get('one_shot', False)
        models: list[str] = info.get('models', [])
        handler: Callable = self._parse_callable(info['func'])
        return EventHandlerParams(handler_id, patterns, one_shot, models, handler)

    def _parse_callable(self, callable_info: dict) -> Callable:
        module_path: str = callable_info['path']
        module_name: str = callable_info['name']
        callable_name: str = callable_info['func_name']

        spec = importlib.util.spec_from_file_location(module_name, module_path)
        if spec is None:
            raise ModuleNotFoundError(
                f'Module "{module_name}" not found at path "{module_path}"'
            )
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)  # type: ignore[union-attr]

        if not hasattr(module, callable_name):
            raise AttributeError(
                f'Function "{callable_name}" not found in module "{module_name}" '
                f'(path: "{module_path}")'
            )
        return getattr(module, callable_name)
