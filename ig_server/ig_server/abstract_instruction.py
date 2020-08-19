from abc import ABC, abstractmethod
from typing import List, Tuple

import attr

from ig_server.exceptions import IGException
from ig_server.ros_wrappers import PortedNode


def value(type_, val):
    try:
        return type_(val)
    except ValueError as e:
        return e


@attr.s(slots=True)
class AbstractInstruction(ABC):
    node: PortedNode = attr.ib()
    _is_active: bool = attr.ib(init=False, default=False)

    @classmethod
    def convert_params(cls, values: List, types: Tuple) -> Tuple:
        if len(values) == len(types):
            merged = [(values[i], types[i]) for i in range(0, len(values))]
            ret = tuple([value(merged[i][1], merged[i][0]) for i in range(0, len(values))])
            if any(isinstance(item, ValueError) for item in ret):
                errors = []
                for i in range(0, len(ret)):
                    if isinstance(ret[i], ValueError):
                        errors.append(f'expected {str(types[i])} for "{values[i]}" in parameter'
                                      f' {i}')
                raise IGException('\n'.join(errors))
            return ret

        else:
            raise IGException(f"received {len(values)} parameters, expecting {len(types)}")

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
