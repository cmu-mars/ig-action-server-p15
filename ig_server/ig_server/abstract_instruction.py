from typing import List, Dict
from abc import ABC, abstractmethod
import attr
from rclpy.node import Node

from ig_server.ig_server import PortedNode


@attr.s(slots=True)
class AbstractInstruction(ABC):
    node: PortedNode = attr.ib()
    _is_active: bool = attr.ib(init=False, default=False)

    @classmethod
    @abstractmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        pass

    @abstractmethod
    def execute(self):
        pass

    @abstractmethod
    def cancel(self):
        pass

    @abstractmethod
    def to_pretty_string(self):
        pass

    def active(self):
        return self._is_active

    def activate(self):
        self._is_active = True

    def deactivate(self):
        self._is_active = False
