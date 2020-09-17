from typing import Dict, List

from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter


class PortedNode(Node):
    ports: Dict[str, any] = {}

    def __init__(self,
                 node_name: str,
                 *,
                 context: Context = None,
                 cli_args: List[str] = None,
                 namespace: str = None,
                 use_global_arguments: bool = True,
                 start_parameter_services: bool = True,
                 parameter_overrides: List[Parameter] = None,
                 allow_undeclared_parameters: bool = False,
                 automatically_declare_parameters_from_overrides: bool = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace,
                         use_global_arguments=use_global_arguments,
                         start_parameter_services=start_parameter_services,
                         parameter_overrides=parameter_overrides,
                         allow_undeclared_parameters=allow_undeclared_parameters,
                         automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
