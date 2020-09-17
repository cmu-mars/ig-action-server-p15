import threading
import time
from typing import List

import attr
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from ig_server.abstract_instruction import AbstractInstruction
from ig_server.exceptions import IGException
from ig_server.ros_wrappers import PortedNode
from ig_server.transformations import quaternion_from_euler


# #################################################################################
# # Had to pull this in because tf2 is not translated to ROS 2 yet for python
# #################################################################################
# # axis sequences for Euler angles
# _NEXT_AXIS = [1, 2, 0, 1]
# # map axes strings to/from tuples of inner axis, parity, repetition, frame
# _AXES2TUPLE = {
#     'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
#     'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
#     'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
#     'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
#     'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
#     'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
#     'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
#     'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}
#
# _TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())
#
#
# def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
#     try:
#         firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
#     except (AttributeError, KeyError) as e:
#         axes = _TUPLE2AXES[axes]
#         firstaxis, parity, repetition, frame = axes
#     i = firstaxis
#     j = _NEXT_AXIS[i + parity]
#     k = _NEXT_AXIS[i - parity + 1]
#
#     if frame:
#         ai, ak = ak, ai
#     if parity:
#         aj = -aj
#
#     ai /= 2.0
#     aj /= 2.0
#     ak /= 2.0
#     ci = math.cos(ai)
#     si = math.sin(ai)
#     cj = math.cos(aj)
#     sj = math.cos(aj)
#     ck = math.cos(ak)
#     sk = math.sin(ak)
#
#     cc = ci * ck
#     cs = ci * sk
#     sc = si * ck
#     ss = si * sk
#
#     quaternion = numpy.empty((4,), dtype=numpy.float64)
#     if repetition:
#         quaternion[i] = cj * (cs + sc)
#         quaternion[j] = sj * (cc + ss)
#         quaternion[k] = sj * (cs - sc)
#         quaternion[3] = cj * (cc - ss)
#     else:
#         quaternion[i] = cj * sc - sj * cs
#         quaternion[j] = cj * ss + sj * cc
#         quaternion[k] = cj * cs - sj * sc
#         quaternion[3] = cj * cc + sj * ss
#     if parity:
#         quaternion[j] *= -1
#
#     return quaternion
#
#
# #################################################################################

@attr.s(frozen=True, slots=True)
class Locate(AbstractInstruction):
    x: float = attr.ib()
    y: float = attr.ib()
    w: float = attr.ib()

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        x, y, w = cls.convert_params(params, (float, float, float))
        return Locate(node=node, x=x, y=y, w=w)

    def execute(self) -> bool:
        if "initialpose" not in self.node.ports:
            self.node.ports['initialpose'] = self.node.create_publisher(PoseWithCovarianceStamped,
                                                                        'initialpose', 10)

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = self.node.get_clock().now().to_msg() if self.node is not None \
            else None
        initial_pose.header.frame_id = 'map'

        initial_pose.pose.pose.position.x = self.x
        initial_pose.pose.pose.position.y = self.y
        initial_pose.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.w)
        initial_pose.pose.pose.orientation.x = q[0]
        initial_pose.pose.pose.orientation.y = q[1]
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]
        initial_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

        self.node.ports['initialpose'].publish(initial_pose)
        time.sleep(5)
        return True

    def cancel(self):
        pass

    def to_pretty_string(self):
        return f'Locate({self.x}, {self.y}, {self.w})'


#
# @attr.s(slots=True)
# class LocateWithGZ(Locate):
#
#     @classmethod
#     def load_from_params(cls, node: PortedNode, params: List[any]):
#         x, y, w = cls.convert_params(params, (float, float, float))
#         return LocateWithGZ(node=node, x=x, y=y, w=w)
#
#     def execute(self):
#         if 'get_model_state' not in self.node.ports:
#             self.node.ports['get_model_state'] = rclpy.create_client(GetModelState,
#                                                                      '/gazebo/get_model_state')
#             self.node.ports['set_model_state'] = rclpy.create_client(SetModelState,
#                                                                      '/gazebo/set_model_state')
#         req = GetModelState.Request()
#         tb = self.node.ports['get_model_state'].call(req)
#         tb.pose.position.x = self.x
#         tb.pose.position.y = self.y
#         q = (tb.pose.orientation.x, tb.pose.orientation.y, tb.pose.orientation.z,
#              tb.pose.orientation.w)
#
#         eu = euler_from_quaternion(q)
#         eu = eu[0], eu[1], self.w
#         q = quaternion_from_euler(eu[0], eu[1], eu[2])
#         tb.pose.orientation.x = q[0]
#         tb.pose.orientation.y = q[1]
#         tb.pose.orientation.z = q[2]
#         tb.pose.orientation.w = q[3]
#
#         req = SetModelState.Request()
#         req.model_name = 'turtlebot3_waffle_pi'
#         req.pose = tb.pose
#         req.twist = tb.twist
#
#         res = self.node.ports['set_model_state'].call(req).response
#         if res.success:
#             return super().execute()
#         else:
#             return False, "Unable to set location in Gazebo"


@attr.s(slots=True)
class Move(AbstractInstruction):
    x: float = attr.ib()
    y: float = attr.ib()
    velocity: float = attr.ib()
    action: str = attr.ib()
    yaw: float = attr.ib(default=0.0)

    _goal: NavigateToPose = attr.ib(init=False)
    _result: (bool, str) = attr.ib(init=False)
    _active_goal_handle = attr.ib(init=False)  # Type is not exported
    _result_block: threading.Event = attr.ib(init=False)

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        if len(params) == 4:
            x, y, v, a = cls.convert_params(params, (float, float, float, str))
            w = 0.0
        elif len(params) == 5:
            x, y, v, a, w = cls.convert_params(params, (float, float, float, str, float))
        else:
            raise IGException(f'Move: found {len(params)} parameters, expecting 4 or 5')

        return Move(node=node,
                    x=x, y=y, velocity=v, action=a, yaw=w)

    def __attrs_post_init__(self):
        self._result = None
        self._active_goal_handle = None

    def create_goal(self):
        if self.action == 'Absolute':
            frame_type = "map"
        else:
            frame_type = "base_link"
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame_type
        goal.pose.header.stamp = self.node.get_clock().now().to_msg() if self.node is not None \
            else None
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y if frame_type == 'map' else 0
        q = quaternion_from_euler(0, 0, self.yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        return goal

    def execute(self):
        self.node.get_logger().info("Executing Move")
        if self.node is not None:
            if "nav2_actions" not in self.node.ports:
                self.node.ports['nav2_pose'] = ActionClient(self.node, NavigateToPose,
                                                            "/navigate_to_pose")
        self._goal = self.create_goal()

        # ack_future = self.node.ports['nav2_pose'].send_goal_async(self.goal)
        # ack_gh = ack_future.result()
        # if not ack_gh.accepted():
        #     self.node.get_logger().info("Goal was rejected")
        #     self._result = False, "Goal rejected"
        #     return self._result
        # self.node.get_logger().info("Goal accepted, awaiting result")
        # result = await ack_gh.get_result_async()
        # self.node.get_logger().info(f"Got the result: {result}")
        # if self._result is None:
        #     self._result = True, None
        # return self._result

        self.node.get_logger().info("Waiting for /NavigateToPose to become available")
        ready = self.node.ports['nav2_pose'].wait_for_server()
        # self.execute_threaded()
        self.execute_ros_spin()
        # self.node.get_logger().info(f"Sending goal to /navigate_to_pose: {str(self._goal)}")
        # result = self.node.ports['nav2_pose'].send_goal(self._goal)
        # self._result = True, None
        if self._result is not None and self._result[1] is not None:
            self.node.get_logger().info(f"{self._result[1]}")
        return self._result[0]

    def execute_ros_spin(self):
        ack_future = self.node.ports['nav2_pose']. \
            send_goal_async(self._goal,
                            feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self.node, ack_future)
        ack_gh = ack_future.result()
        if not ack_gh.accepted:
            self.node.get_logger().info("Goal was rejected")
            self._result = False, "Goal rejected"
            return
        result_future = ack_gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result()
        # If result is not set, then that means it hasn't been canceled
        if self._result is None:
            if self._distance > 1:
                self._result = False, f'More than a meter away from ({self.x}, {self.y})'
            else:
                self._result = True, None

    def _feedback_callback(self, msg):
        feedback = msg.feedback
        self._distance = feedback.distance_remaining


    def execute_threaded(self):
        self._result_block = threading.Event()

        def goal_response_callback(future):
            goal_handle = future.result()

            if not goal_handle.accepted():
                self._result = False, "Goal rejected"
                self._result_block.set()
                return

            self.node.get_logger().info("Goal accepted")
            self._gf_future = goal_handle.get_result_async()
            self._gf_future.add_done_callback(goal_result_callback)

        def goal_result_callback(future):
            result = future.result().result
            self.node.get_logger().info(f'Received result: {str(result)}')
            self._result = True, None
            self._gf_future = None
            self._result_block.set()

        def send():
            self.node.get_logger().info(f"Sending goal to /navigate_to_pose: {str(self._goal)}")
            gh = self.node.ports['nav2_pose'] \
                .send_goal_async(self._goal,
                                 feedback_callback=self._feedback_callback)
            gh.add_done_callback(self._goal_response_callback)
            self._result_block.wait()

        goal_thread = threading.Thread(name='send_goal', target=send)
        goal_thread.start()
        goal_thread.join()

        self._result_block = None


    def cancel(self):
        if self._active_goal_handle is not None:
            cancel_future = self._active_goal_handle.cancel_goal_async(self.cancel_done)
            rclpy.spin_until_future_complete(self.node, cancel_future)
        self._result = (False, "canceled")


    def cancel_done(self, future):
        self._cancel_block.set()


    def to_pretty_string(self):
        return f"Move({self.x}, {self.y}, {self.velocity}, {self.action}" \
               f"{f', {self.yaw}' if self.yaw != 0 else ''})"


class MoveAbs(Move):

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        x, y, v = cls.convert_params(params, (float, float, float))

        return MoveAbs(x=x, y=y, velocity=v, node=node, action="Absolute", yaw=0)

    def to_pretty_string(self):
        return f"MoveAbs({self.x}, {self.y}, {self.velocity})"


class MoveAbsH(Move):

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        x, y, v, w = cls.convert_params(params, (float, float, float, float))
        return MoveAbsH(node=node, x=x, y=y, velocity=v, action='Absolute', yaw=w)

    def to_pretty_string(self):
        return f"MoveAbsH({self.x}, {self.y}, {self.velocity}, {self.yaw})"


@attr.s(slots=True)
class Stop(AbstractInstruction):
    distance: float = attr.ib()
    object_: str = attr.ib()

    @classmethod
    def load_from_params(cls, node: PortedNode, params: List[any]):
        distance, object_ = cls.convert_params(params, (float, str))
        return Stop(node=node, distance=distance, object_=object_)

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
        o = cls.convert_params(params, [str])
        return Visible(node=node, object_=o)

    def execute(self):
        print(f"Checking if {self.object_} is visible...")
        print(f"Is {self.object_} visible?")
        ans = input()
        return ans in ("yes", "y", "", "\n")

    def cancel(self):
        print("Cannot cancel input")

    def to_pretty_string(self):
        return f'Visible({self.object_})'
