from typing import List

import attr
from medicine_dispenser_interfaces.srv import DispenseMedicine as DSM

from ig_server.abstract_instruction import AbstractInstruction
from ig_server.ig_server import PortedNode


@attr.s(slots=True)
class DispenseMedicine(AbstractInstruction):

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        return DispenseMedicine(node=node)

    def __attrs_post_init__(self):
        if "dispense_medicine" not in self.node.ports:
            self.node.ports['dispense_medicine'] = self._node.create_client(DSM,
                                                                            "dispense_medicine")

    def execute(self):
        while not self.node.ports['dispense_medicine'].wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn("Service 'dispense_medicine' not available, waiting again")
        req = DSM.Request()
        resp = self.node.ports['dispense_medicine'].call(req)
        return resp

    def cancel(self):
        pass

    def to_pretty_string(self):
        return f'DispenseMedicine()'
