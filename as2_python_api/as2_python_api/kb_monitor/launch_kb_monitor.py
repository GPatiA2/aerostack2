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

"""Entry point: launch a KB monitor node from a JSON config file."""

__authors__ = 'Guillermo GP-Lenza'
__copyright__ = 'Copyright (c) 2026 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse

from as2_python_api.kb_monitor.kb_event_handler import KBMonitorNode
from as2_python_api.kb_monitor.kb_parsing import KBJSONParser
import rclpy
from rclpy.executors import MultiThreadedExecutor


def _options() -> dict:
    parser = argparse.ArgumentParser(
        description='Launch a KB event monitor from a JSON configuration file.'
    )
    parser.add_argument('file', type=str, help='Path to the JSON config file.')
    parser.add_argument(
        '--drone_namespace',
        type=str,
        required=True,
        help='Namespace of the drone to monitor (e.g. "drone0").',
    )
    return vars(parser.parse_args())


def main():
    """Parse config, register handlers, and spin."""
    args = _options()
    cfg_file: str = args['file']
    drone_namespace: str = args['drone_namespace']

    print(f'Launching KB monitor for drone: {drone_namespace}')

    cfg = KBJSONParser(cfg_file)

    rclpy.init()

    monitor = KBMonitorNode('kb_monitor')

    for params in cfg.handlers:
        monitor.register_handler(drone_namespace, cfg.kb_namespace, params)

    executor = MultiThreadedExecutor()
    executor.add_node(monitor)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
