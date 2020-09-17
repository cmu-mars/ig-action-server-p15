import threading
import time
import traceback

import rclpy
from action_msgs.msg import GoalStatus
from ig_action_msgs.action import InstructionGraph
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from ig_server.abstract_instruction import AbstractInstruction
from ig_server.ig_evaluator_lark import IGEvaluator
from ig_server.ros_wrappers import PortedNode
from ig_server.instructions import common, sei


class CancelTracker(object):

    def __init__(self):
        self._canceled = False
        self.lock = threading.Lock()

    def is_canceled(self):
        with self.lock:
            return self._canceled

    def cancel(self):
        with self.lock:
            self._canceled = True


class IGServer(PortedNode):
    _init_time = None
    _tf = None
    _canceled: CancelTracker = None
    _success: bool = True
    cp3 = None
    _current_instruction: AbstractInstruction = None
    _result: InstructionGraph.Result = None

    def __init__(self):
        super().__init__('ig_action_server')  # TODO: Name probably  should be parameterized
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.instructions = None

        self._action_server = ActionServer(
            self,
            InstructionGraph,
            'execute_ig',
            execute_callback=self.execute_cb,
            cancel_callback=self.cancel_cb,
            handle_accepted_callback=self.handle_accepted_cb,
            goal_callback=self.goal_callback
        )
        self.get_logger().info("Instruction Graph Server up and waiting")

    def goal_callback(self, goal_request):
        """Reject a goal_request if we already have a goal executing."""
        self.get_logger().info("Handling goal callback")
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_cb(self, goal_handle):
        self.get_logger().info("Handling an accepted goal")
        with self._goal_lock:
            # Only handles one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info("Aborting the previous goal")
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def cancel_cb(self, goal):
        self.get_logger().info("Goal is being canceled")
        if self._current_instruction is not None and self._current_instruction.active():
            self.get_logger().info(f"Canceling {self._current_instruction.to_pretty_string()}")
            self._current_instruction.cancel()
        self.instructions = None
        return CancelResponse.ACCEPT

    def execute_cb(self, goal_handle):
        try:
            self.get_logger().info("Goal is being executed")
            self.wait_for_cancel_to_finish()
            self.execute_instructions(goal_handle)
        except Exception as e:
            self.publish_feedback(f"Exception raised: {str(e)}")
            traceback.print_exc()
        return self._result

    def wait_for_cancel_to_finish(self):
        while self._canceled is not None and self.instructions is not None:
            self.get_logger().info("Waiting for old instructions to be canceled")
            time.sleep(1)

    def execute_instructions(self, goal_handle):
        self.get_logger().info(f'Got goal handle: {str(goal_handle)}')
        self.instructions = goal_handle.request.order

        if self._canceled is not None and not self._canceled.is_canceled():
            self.get_logger().info(
                "Received a set of instructions without canceling the previous one")
            return

        self._success = True
        self._canceled = CancelTracker()
        self.publish_feedback(f'BRASS | IG | Received a new goal: {self.instructions}')

        evaluator: IGEvaluator = IGEvaluator(self, self.instructions, self._canceled.is_canceled,
                                             self.publish_feedback,
                                             [common, sei])
        self._success = evaluator.parse()
        if self._success and not self._canceled.is_canceled():
            self.publish_feedback(f'Received a new valid IG: {self.instructions}')
            self.publish_feedback('Executing graph')
            self._success = evaluator.eval_instructions()
        elif self._canceled.is_canceled():
            self.publish_feedback('Execution for goal is canceled')

        if self._canceled.is_canceled():
            self.publish_feedback('BRASS | IG | Goal canceled')
            self.publish_result('Execution for goal canceled')
        elif self._success:
            self.publish_feedback('BRASS | IG | Goal completed successfully')
            self.publish_result("Execution for goal completed successfully")
            self._canceled = None
        else:
            self.publish_feedback('BRASS | IG | Goal failed')
            self.publish_result('Execution for goal failed')
            self._canceled = None

    def publish_feedback(self, msg):
        self.get_logger().info(msg)
        try:
            feedback_msg = InstructionGraph.Feedback()
            feedback_msg.sequence = msg
            self._goal_handle.publish_feedback(feedback_msg)
        except Exception as e:
            self.get_logger().info(traceback.format_exc())

    def publish_result(self, result):
        self._result = InstructionGraph.Result()
        self._result.sequence = result
        if self._success:
            self._goal_handle.succeed()
        else:
            self._goal_handle.abort()


def main(args=None):
    rclpy.init(args=args)
    ig_action_server = IGServer()
    rclpy.spin(ig_action_server)


if __name__ == '__main__':
    main()
