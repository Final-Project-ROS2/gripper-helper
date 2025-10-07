import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from control_msgs.action import GripperCommand

class GripperWrapper(Node):
    def __init__(self):
        super().__init__('gripper_wrapper_node')

        # Publisher for monitoring
        self.monitor_pub = self.create_publisher(String, '/gripper/monitor', 10)

        # Callback group for concurrency (server <-> client)
        self.cb_group = ReentrantCallbackGroup()

        # Wrapper action server
        self.action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_wrapper',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group)

        # Action client to the real gripper
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_position_controller/gripper_cmd',
            callback_group=self.cb_group)

        self.get_logger().info('Gripper wrapper node started.')

    async def execute_callback(self, goal_handle):
        # Publish to monitor topic
        msg = String()
        msg.data = f"position: {goal_handle.request.command.position}, max_effort: {goal_handle.request.command.max_effort}"
        self.monitor_pub.publish(msg)

        self.get_logger().info(f"Received gripper action goal: {msg.data}")

        # Wait for the underlying gripper action server
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            goal_handle.abort()
            self.get_logger().error('Gripper action server not available.')
            return GripperCommand.Result()

        # Forward the goal
        forwarded_goal = GripperCommand.Goal()
        forwarded_goal.command.position = goal_handle.request.command.position
        forwarded_goal.command.max_effort = goal_handle.request.command.max_effort

        # Send goal asynchronously to underlying gripper
        send_goal_future = self.gripper_client.send_goal_async(
            forwarded_goal,
            feedback_callback=lambda fb: self._feedback_cb(goal_handle, fb))
        rclpy.spin_until_future_complete(self, send_goal_future)
        forwarded_result_handle = send_goal_future.result()

        if not forwarded_result_handle.accepted:
            goal_handle.abort()
            self.get_logger().warn('Forwarded goal was rejected by underlying gripper.')
            return GripperCommand.Result()

        # Wait for result
        get_result_future = forwarded_result_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        goal_handle.succeed()
        return result

    def _feedback_cb(self, original_goal_handle, feedback_msg):
        # Echo feedback from underlying gripper to original goal caller
        original_goal_handle.publish_feedback(feedback_msg.feedback)

def main(args=None):
    rclpy.init(args=args)
    node = GripperWrapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
