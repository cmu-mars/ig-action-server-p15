from typing import Dict

from rclpy.node import Node


class PortedNode(Node):
    ports: Dict[str, any] = []
