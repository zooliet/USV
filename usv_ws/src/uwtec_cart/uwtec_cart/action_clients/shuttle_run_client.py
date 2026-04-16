from rclpy.action.server import CancelResponse
from rclpy.action.client import ActionClient

from uwtec_interfaces.action import GeoLoc


class ShuttleRunClient(ActionClient):
    def __init__(self, node, action_type, action_name):
        super().__init__(node, action_type, action_name)
        node.get_logger().info("shuttle_run_action_client has been initialized")
        self.node = node  # Agent node
        self.goal_handle = None

    def action(self, latitude, longitude):
        # Cancel any existing goal before sending a new one
        if self.goal_handle:
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self.cancel_response_callback)
            self.goal_handle = None  # Clear the goal handle
        else:
            goal_msg = GeoLoc.Goal()
            goal_msg.point.latitude = latitude
            goal_msg.point.longitude = longitude

            self.node.get_logger().info("Waiting for action server...")
            self.wait_for_server()  # Wait for the server to be ready

            self.node.get_logger().info("Sending goal request...")
            # Send goal asynchronously and set up done and feedback callbacks
            self.send_goal_future = self.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )

            self.send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f"Received feedback: {feedback.progress}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info("Goal rejected")
            return

        self.node.get_logger().info("Goal accepted, waiting for result...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

        # Store the goal handle for potential cancellation
        self.goal_handle = goal_handle

    def get_result_callback(self, future):
        result = future.result().result
        self.goal_handle = None
        self.node.get_logger().info(f"Result: {result.success}")

    def cancel(self):
        if self.goal_handle:
            self.cancel_future = self.goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self.cancel_response_callback)
            self.goal_handle = None  # Clear the goal handle
        else:
            self.node.get_logger().info("No active goal to cancel.")

    def cancel_response_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == CancelResponse.ACCEPT:
            self.node.get_logger().info("Goal successfully cancelled")
        else:
            self.node.get_logger().info("Goal cancellation request rejected")
