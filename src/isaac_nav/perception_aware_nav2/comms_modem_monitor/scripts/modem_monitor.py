#!/usr/bin/env python3
import os
import re
import socket
import subprocess
import sys
import time
from typing import Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from comms_modem_monitor.msg import ModemLink


class ModemMonitorNode(Node):
    def __init__(self):
        super().__init__('modem_monitor', namespace=self._resolve_namespace())

        self.robot_id = self.get_namespace().lstrip('/') or 'unknown'

        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('mmcli_modem', 0)
        self.declare_parameter('topic_name', 'modem_link')
        self.declare_parameter('command_timeout_s', 5.0)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.mmcli_modem = str(self.get_parameter('mmcli_modem').value)
        self.topic_name = str(self.get_parameter('topic_name').value)
        self.command_timeout_s = float(self.get_parameter('command_timeout_s').value)

        self.publisher_ = self.create_publisher(ModemLink, self.topic_name, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_metrics)

        self.get_logger().info(
            f'Publishing modem metrics to {self.get_namespace()}/{self.topic_name} at {self.rate_hz} Hz'
        )
        
    def _resolve_namespace(self) -> str:
        raw = os.environ.get('ROBOT', '').strip() or socket.gethostname().split('.')[0]
        # ROS namespaces allow only [A-Za-z0-9_]; sanitize the hostname
        ns = re.sub(r'[^0-9A-Za-z_]', '_', raw)
        if ns and ns[0].isdigit():
            ns = '_' + ns
        return ns

    # def _resolve_namespace(self) -> str:
    #     namespace = os.environ.get('ROBOT', '').strip()
    #     if namespace:
    #         return namespace
    #     return ''

    def _send_at_command(self, cmd: str) -> str:
        try:
            result = subprocess.run(
                [
                    'sudo', '-n',
                    '/usr/bin/mmcli',
                    '-m', self.mmcli_modem,
                    f'--command={cmd}',
                ],
                capture_output=True,
                text=True,
                timeout=self.command_timeout_s,
            )
            #(
            #     ['sudo', 'mmcli', '-m', self.mmcli_modem, f'--command={cmd}'],
            #     capture_output=True,
            #     text=True,
            #     timeout=self.command_timeout_s,
            # )
            output = result.stdout + result.stderr
            if 'response:' in output:
                return output.split('response:', 1)[1].strip()
            return output
        except subprocess.TimeoutExpired:
            return ''
        except Exception:
            return ''

    def _parse_quoted_csv(self, content: str) -> list:
        parts = []
        current = ''
        in_quotes = False
        for char in content:
            if char == '"':
                in_quotes = not in_quotes
            elif char == ',' and not in_quotes:
                parts.append(current.strip().strip('"'))
                current = ''
                continue
            current += char
        if current:
            parts.append(current.strip().strip('"'))
        return parts

    def _parse_metrics(self, responses: Dict[str, str]) -> Dict[str, float]:
        rsrp = None
        rsrq = None
        sinr = None
        combined = '\n'.join(responses.values())

        for line in combined.split('\n'):
            line = line.strip().strip("'")
            if not line:
                continue

            if line.startswith('+QENG:'):
                parts = self._parse_quoted_csv(line[6:].strip())
                if len(parts) > 14 and parts[0] == 'servingcell':
                    rsrp = self._parse_float(parts[12])
                    rsrq = self._parse_float(parts[13])
                    sinr = self._parse_float(parts[14])

            elif line.startswith('+QRSRP:'):
                match = re.search(r'[-+]?\d+(?:\.\d+)?', line[8:])
                if match:
                    rsrp = self._parse_float(match.group(0))

            elif line.startswith('+QRSRQ:'):
                match = re.search(r'[-+]?\d+(?:\.\d+)?', line[8:])
                if match:
                    rsrq = self._parse_float(match.group(0))

            elif line.startswith('+QSINR:'):
                match = re.search(r'[-+]?\d+(?:\.\d+)?', line[8:])
                if match:
                    sinr = self._parse_float(match.group(0))


            # elif line.startswith('+QRSRP:'):
            #     values = [value.strip() for value in line[8:].split(',')]
            #     if values and values[0].isdigit():
            #         rsrp = self._parse_float(values[0])

            # elif line.startswith('+QRSRQ:'):
            #     values = [value.strip() for value in line[8:].split(',')]
            #     if values and values[0].isdigit():
            #         rsrq = self._parse_float(values[0])

            # elif line.startswith('+QSINR:'):
            #     values = [value.strip() for value in line[8:].split(',')]
            #     if values and values[0].isdigit():
            #         sinr = self._parse_float(values[0])

        return {'rsrp': rsrp, 'rsrq': rsrq, 'sinr': sinr}

    def _parse_float(self, value: str):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def publish_metrics(self):
        responses = {
            'servingcell': self._send_at_command('AT+QENG="servingcell"'),
            'rsrq': self._send_at_command('AT+QRSRQ'),
            'rsrp': self._send_at_command('AT+QRSRP'),
            'sinr': self._send_at_command('AT+QSINR'),
        }
        metrics = self._parse_metrics(responses)

        msg = ModemLink()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.robot_id = self.robot_id
        msg.rsrp = metrics['rsrp'] if metrics['rsrp'] is not None else float('nan')
        msg.rsrq = metrics['rsrq'] if metrics['rsrq'] is not None else float('nan')
        msg.sinr = metrics['sinr'] if metrics['sinr'] is not None else float('nan')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
