import argparse
import ipaddress
from pyproj import Transformer

import asyncio
import async_timeout

from redis.asyncio import Redis
from redis.asyncio.client import PubSub

import rclpy
from rclpy.node import Node
from rclpy.action.server import CancelResponse
from rclpy.action.client import ActionClient

from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand
# from uwtec_navigation.utils import signed_angle

class ActionClientNode(Node):
    def __init__(self, debug):
        super().__init__("agent_node")
        self.get_logger().info("Agent node has been started.")
        self.debug = debug

        # self.transformer = Transformer.from_crs(
        #     "EPSG:4326", "EPSG:32652", always_xy=True
        # )
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_quality = 0
        self.num_sats = 0

        self.action_client_heading = ActionClient(self, SimpleCommand, 'test_heading_and_offset')
        # self.action_client_heading = ActionClient(self, SimpleCommand, 'nav_heading_and_offset')
        self.goal_handle_heading = None

        # self.action_client_wpf = ActionClient(self, SimpleCommand, 'test_waypoints_follower')
        # # self.action_client_wpf = ActionClient(self, SimpleCommand, 'nav_waypoints_follower')
        # self.goal_handle_wpf = None
        #
        # self.action_client_shuttle = ActionClient(self, SimpleCommand, 'test_shuttle_run')
        # # self.action_client_shuttle = ActionClient(self, SimpleCommand, 'nav_shuttle_run')
        # self.goal_handle_shuttle = None

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        # self.utm_x, self.utm_y = self.transformer.transform(
        #     self.longitude, self.latitude
        # )
        self.yaw = -msg.heading % 360  # convert to CCW positive
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats
        # self.heading = east_facing_ccw_angle(self.heading)

    ## for action_client_heading
    def send_goal_heading(self):
        goal_msg = SimpleCommand.Goal()
        goal_msg.cmd = "heading"

        self.get_logger().info("Waiting for action server...")
        self.action_client_heading.wait_for_server()  # Wait for the server to be ready

        self.get_logger().info("Sending goal request...")
        # Send goal asynchronously and set up done and feedback callbacks
        self.send_goal_future_heading = self.action_client_heading.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback_heading
        )

        self.send_goal_future_heading.add_done_callback(self.goal_response_callback_heading)

    def send_cancel_heading(self):
        if self.goal_handle_heading:
            self.cancel_future_heading = self.goal_handle_heading.cancel_goal_async()
            self.cancel_future_heading.add_done_callback(self.cancel_response_callback_heading)
            self.goal_handle_heading = None  # Clear the goal handle
        else:
            self.get_logger().info("No active goal to cancel.")

    def feedback_callback_heading(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.progress}")

    def goal_response_callback_heading(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        self.get_result_future_heading = goal_handle.get_result_async()
        self.get_result_future_heading.add_done_callback(self.get_result_callback_heading)

        # Store the goal handle for potential cancellation
        self.goal_handle_heading =  goal_handle

    def get_result_callback_heading(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.success}")

    def cancel_response_callback_heading(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == CancelResponse.ACCEPT:
            self.get_logger().info("Goal successfully cancelled")
        else:
            self.get_logger().info("Goal cancellation request rejected")



class Agent():
    def __init__(self, redis, node):
        self.redis = redis
        self.node = node

    async def process(self, message):
        print(f"Received message: {message}")
        # if message['channel'].decode() == 'channel::agent':
        data = message['data'].decode().split(",")
        cmd = data[0]
        params = None if len(data) == 1 else data[1:]

        if cmd == 'ping':
            response = f"pong,{self.node.latitude},{self.node.longitude},{self.node.gps_quality},{self.node.num_sats}"
            await self.redis.publish('channel::tui', response)

        elif cmd == 'poweroff':
            print('poweroff')
        elif cmd == 'reboot':
            print('reboot')

        elif cmd == 'find' and params[0] == 'heading':
            self.node.send_goal_heading()
        elif cmd == 'cancel' and params[0] == 'heading':
            self.node.send_cancel_heading()

        elif cmd == 'perform' and params[0] == 'shuttle':
            self.node.send_goal_shuttle()
        elif cmd == 'cancel' and params[0] == 'shuttle':
            self.node.send_cancel_shuttle()

        elif cmd == 'perform' and params[0] == 'wpf':
            self.node.send_goal_wpf()
        elif cmd == 'cancel' and params[0] == 'wpf':
            self.node.send_cancel_wpf()



async def ros_loop(nodes):
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        while executor.context.ok():
            # while rclpy.ok():
            executor.spin_once(timeout_sec=0.1) # 0.001 ?
            await asyncio.sleep(0.001)
    finally:
        for node in nodes:
            executor.remove_node(node)
        executor.shutdown()


async def redis_loop(redis: Redis, node: ActionClientNode):
    # redis = Redis.from_url("redis://localhost")
    pubsub = redis.pubsub()
    # await pubsub.subscribe("channel::tui", "channel::apps")
    await pubsub.subscribe("channel::agent")

    agent = Agent(redis, node)

    while True:
        try:
            async with async_timeout.timeout(1):
                message = await pubsub.get_message(ignore_subscribe_messages=True)
                if message:
                    await agent.process(message)
                #     if message["data"].decode() == "ACTION":
                #         print("Received ACTION command, sending goal...")
                #         # Here you would trigger the action client to send a goal
                #         node.send_goal(order=10)  # Example order value
                #     elif message["data"].decode() == "CANCEL":
                #         print("Received CANCEL command, sending cancel request...")
                #         node.send_cancel()
                #     elif message["data"].decode() == "STOP":
                #         break
                await asyncio.sleep(0.001)
        except asyncio.TimeoutError:
            pass

async def main_async(redis, *nodes):
    ros_tasks = asyncio.create_task(ros_loop(list(nodes)))
    redis_tasks = asyncio.create_task(redis_loop(redis, nodes[0]))
    await asyncio.gather(ros_tasks, redis_tasks)


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    ap.add_argument("--redis-ip", type=ipaddress.ip_address, default="127.0.0.1", help="IP address to connect to (e.g., 192.168.1.1")
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    redis_ip = args.get('redis-ip', '127.0.0.1') 
    redis_ip = ipaddress.ip_address(redis_ip)
    redis = Redis.from_url(f"redis://{redis_ip}")
    print(redis)
    node = ActionClientNode(debug=args.get('debug', False))
    try:
        asyncio.run(main_async(redis, node))
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()

