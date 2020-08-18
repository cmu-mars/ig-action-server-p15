import math
import threading
import thread
from typing import List

import attr
import numpy
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped

from nav2_msgs.action import NavigateToPose

from ig_server.abstract_instruction import AbstractInstruction, check_args
from ig_server.ig_server import PortedNode

#################################################################################
# Had to pull this in because tf2 is not translated to ROS 2 yet for python
#################################################################################
# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]
# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError) as e:
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes
    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.cos(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)

    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    quaternion = numpy.empty((4,), dtype=numpy.float64)
    if repetition:
        quaternion[i] = cj * (cs + sc)
        quaternion[j] = sj * (cc + ss)
        quaternion[k] = sj * (cs - sc)
        quaternion[3] = cj * (cc - ss)
    else:
        quaternion[i] = cj * sc - sj * cs
        quaternion[j] = cj * ss + sj * cc
        quaternion[k] = cj * cs - sj * sc
        quaternion[3] = cj * cc + sj * ss
    if parity:
        quaternion[j] *= -1

    return quaternion


#################################################################################

@attr.s(frozen=True, slots=True)
class Locate(AbstractInstruction):
    x: float = attr.ib()
    y: float = attr.ib()
    w: float = attr.ib()

    def load_from_params(cls, node: PortedNode, params: List[any]):
        if len(params) < 3:
            raise Exception(f'Locate found {len(params)} parameters, expecting 3')
        x = float(params[0])
        y = float(params[1])
        w = float(params[2])

        return Locate(node=node, x=x, y=y, w=w)

    def __attrs_post_init__(self):
        if "initialpose" not in self.node.ports:
            self.node.ports['initialpose'] = rclpy.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10, latch=True)

    def execute(self):
        initial_pose = PosWithCovarianceStamped()
        initial_pose.header.stamp = rclpy.Time.now()
        initial_pose.header.frame_id = 'map'

        initial_pose.pose.pose.position.x = self.x
        initial_pose.pose.pose.position.y = self.y
        initial_pose.pose.pose.position.z = 0
        q = quaternion_from_euler(0, 0, self.w)
        initial_pose.pose.pose.orientation.x = q[0]
        initial_pose.pose.pose.orientation.y = q[1]
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.node.ports['initialpose'].publish(initial_pose)
        rclpy.sleep(1)

    def cancel(self):
        pass

    def to_pretty_string(self):
        return f'Locate({self._x}, {self._y}, {self._w})'


@attr.s(slots=True)
class Move(AbstractInstruction):

    x: float = attr.ib()
    y: float = attr.ib()
    velocity: float = attr.ib()
    action: str = attr.ib()
    yaw: float = attr.ib(default=0.0)

    _goal: NavigateToPose = attr.ib(init=False)
    _result: (bool, str) = attr.ib(init=False)
    _active_goal_handle: rclpy.action.GoalHandle = attr.ib(init=False)
    _result_block: threading.Event = attr.ib(init=False)
    _cancel_block: threading.Event = attr.ib(init=False)

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        if len(params) < 4 or len(params) > 5:
            raise Exception(f'Move: found {len(params)} parameters, expecting 4 or 5')
        x = float(params[0])
        y = float(params[1])
        v = float(params[2])
        a = str(params[3])
        w = float(params[4]) if len(params) == 5 else 0.0
        return Move(node=node,
                    x=x, y=y, velocity=v, action=a, yaw=w)

    def __attrs_post_init__(self):
        self._goal = self.create_goal()
        self._result = None
        self._active_goal_handle = None
        self._result_block = None
        self._cancel_block = None

        if "nav2_actions" in self.node.ports.keys():
            self.node.ports['nav2_pose'] = ActionClient(self.node, NavigateToPose, "/NavigateToPose")

    def create_goal(self):
        if self.action == 'Absolute':
            frame_type = "map"
        else:
            frame_type = "base_link"

        goal = NavigateToPose()
        goal.pose.header.frame_id = frame_type
        goal.pose.header.stamp = rclpy.Time.now()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y if frame_type == 'map' else 0
        q = quaternion_from_euler(0, 0, self.yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        return goal

    def execute(self):
        self._cancel_block = None
        self.node.ports['nav2_pose'].wait_for_server()

        self._result_block = threading.Event()

        gh = self.node.ports['nav2_pose'].send_goal_async(self.goal)
        sgf = gh.add_done_callback(self._goal_response_callback)

        self._result_block.wait()
        self._result_block = None

        return self._result

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted():
            self._result = False, "Goal rejected"
            self._result_block.set()
            return

        gf = goal_handle.get_result_async()
        gf.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        self._result = True, None
        self._result_block.set()

    def cancel(self):
        self._cancel_block = threading.Event()

        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async(self.cancel_done)

        self._cancel_block.wait(30)

        self._result = (False, "canceled")
        if self._result_block is not None:
            self._result_block.set()

    def cancel_done(self, future):
        self._cancel_block.set()

    def to_pretty_string(self):
        return f"Move({self._x}, {self._y}, {self._velocity}, {self._action}{f', {self._w}' if self._w != 0 else ''})"


class MoveAbs(Move):

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        if len(params) != 3:
            raise Exception(f'MoveAbs received {len(params)} parameters, expected 3')

        x = float(params[0])
        y = float(params[1])
        v = float(params[2])

        return Move(x=x, y=y, velocity=v, node=Node, action="Absolute", yaw=0)

    def to_pretty_string(self):
        return f"MoveAbs({self._args[0]}, {self._args[1]}, {self._args[2]})"

@attr.s(slots=True)
class Stop(AbstractInstruction):

    distance: float = attr.ib()
    object_: str = attr.ib()

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        if len(params) != 2:
            raise Exception(f"Stop received {len(params)} parameters, expected 2")
        distance = float(params[0])
        object_ = str(params[1])

        return Stop(distance=distance, object_=object_)

    def execute(self):
        print(f"Checking if {self.object_} is within {self.distance} distance...")
        print(f"Is {self.object_} within {self.distance} distance?")
        try:
            ans = input()
            return ans in ("yes", "y", "", "\n")
        except KeyboardInterrupt:
            return False

    def cancel(self):
        print("Cannot cancel the input")

    def to_pretty_string(self):
        return f'Stop({self.distance}, {self.object_})'


@attr.s(slots=True)
class Visible(AbstractInstruction):
    object_: str = attr.ib()

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        if len(params) != 1:
            raise Exception(f'Visible received {len(params)} parameters, expecting 1')
        o = str(params[0])
        return Visible(object_=o)

    def execute(self):
        print(f"Checking if {self.object_} is visible...")
        print(f"Is {self.object_} visible?")
        ans = input()
        return ans in ("yes", "y", "", "\n")

    def cancel(self):
        print("Cannot cancel input")

    def to_pretty_string(self):
        return f'Visible({self.object_})'
