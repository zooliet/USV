import argparse
import os
import yaml
from typing import cast
from itertools import pairwise
from ament_index_python.packages import get_package_share_directory
# from pyproj import Transformer
# import ipaddress

import asyncio
import async_timeout

from redis.asyncio import Redis
# from redis.asyncio.client import PubSub

import rclpy
from rclpy.node import Node
# from rclpy.action.server import CancelResponse
# from rclpy.action.client import ActionClient

from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand, GeoLoc

from uwtec_agent.action_clients import (
    HeadingAndOffsetClient, 
    ShuttleRunClient, 
    NavToWpsClient
)

# from uwtec_navigation.utils import signed_angle
#
from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
    # TopicInfo,
    # Sub,
)


class Agent(Node):
    def __init__(self, debug=False):
        super().__init__("agent_node")
        self.get_logger().info("Agent node has been started.")

        self.debug = debug
        self.ticks = 0
        self.redis = Redis.from_url("redis://localhost")

        # self.transformer = Transformer.from_crs(
        #     "EPSG:4326", "EPSG:32652", always_xy=True
        # )
        self.latitude = 0.0
        self.longitude = 0.0
        self.gps_quality = 0
        self.num_sats = 0

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )
        # self.action_client_heading_and_offset = HeadingAndOffsetClient(
        #     self, SimpleCommand, "test_heading_and_offset"
        # )
        self.action_client_heading_and_offset = HeadingAndOffsetClient(
            self, SimpleCommand, "heading_and_offset"
        )
        
        # self.action_client_shuttle_run = ShuttleRunClient(self, GeoLoc, "test_shuttle_run")
        self.action_client_shuttle_run = ShuttleRunClient(self, GeoLoc, "shuttle_run")
        
        # self.action_client_wpf = NavToWpsClient(self, SimpleCommand, "test_nav_to_wps")
        self.action_client_wpf = NavToWpsClient(self, SimpleCommand, "nav_to_wps")
        
        pid = os.getpid()
        with open("/tmp/agent_node.pid", "w") as f:
            f.write(str(pid))

    def gps_custom_callback(self, msg):
        self.ticks += 1
        if self.ticks % 10 == 0 and self.debug:  # Print stats every 1 second
            self.get_logger().info(
                f"Latitude: {self.latitude}, Longitude: {self.longitude}"
            )
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        # self.utm_x, self.utm_y = self.transformer.transform(
        #     self.longitude, self.latitude
        # )
        self.yaw = -msg.heading % 360  # convert to CCW positive
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats
        # self.heading = east_facing_ccw_angle(self.heading)

    async def process(self, message):
        self.get_logger().info(f"Received message: {message}")
        # if message['channel'].decode() == 'channel::agent':
        data = message["data"].decode().split(",")
        cmd = data[0]
        params = [] if len(data) == 1 else data[1:]

        if cmd == "ping":
            response = f"pong,{self.latitude},{self.longitude},{self.gps_quality},{self.num_sats}"
            await self.redis.publish("channel::tui", response)

        elif cmd == "poweroff":
            # print("poweroff")
            executable_py = os.path.join(
                get_package_share_directory("uwtec_agent"),
                "script",
                "uwtec_poweroff.py",
            )
            os.system(f"python {executable_py}")
        elif cmd == "reboot":
            executable_py = os.path.join(
                get_package_share_directory("uwtec_agent"), "script", "uwtec_reboot.py"
            )
            os.system(f"python {executable_py}")
        elif cmd == "upload":
            # print("upload coordinates")
            if len(params) % 2 != 0 or len(params) < 2:
                self.get_logger().info(
                    "Invalid number of parameters for upload command"
                )
                return

            coords = []
            for lat, lon in pairwise(params):
                coord = {"latitude": lat, "longitude": lon}
                coords.append(coord)

            wps_yaml_path = os.path.join(
                get_package_share_directory("uwtec_navigation"), "config", "wps.yaml"
            )
            with open(wps_yaml_path, "w") as wps_file:
                yaml.dump(coords, wps_file, sort_keys=False)

        elif cmd == "action" and params[0] == "heading_and_offset":
            self.action_client_heading_and_offset.action()
        elif cmd == "cancel" and params[0] == "heading_and_offset":
            self.action_client_heading_and_offset.cancel()

        elif cmd == "action" and params[0] == "shuttle_run":
            latitude = float(params[1])
            longitude = float(params[2])
            self.action_client_shuttle_run.action(latitude, longitude)
        elif cmd == "cancel" and params[0] == "shuttle_run":
            self.action_client_shuttle_run.cancel()

        elif cmd == "action" and params[0] == "wpf":
            self.action_client_wpf.action()
        elif cmd == "cancel" and params[0] == "wpf":
            self.action_client_wpf.cancel()


# async def ros_loop(nodes):
#     from rclpy.executors import MultiThreadedExecutor
#
#     executor = MultiThreadedExecutor()
#     for node in nodes:
#         executor.add_node(node)
#
#     try:
#         while executor.context.ok():
#             # while rclpy.ok():
#             executor.spin_once(timeout_sec=0.1)  # 0.001 ?
#             await asyncio.sleep(0.001)
#     finally:
#         for node in nodes:
#             executor.remove_node(node)
#         executor.shutdown()


async def redis_loop():
    with auto_session().lock() as session:
        # action_client = cast(ActionClientNode, session)
        agent = cast(Agent, session)

    redis = Redis.from_url("redis://localhost")
    pubsub = redis.pubsub()
    # await pubsub.subscribe("channel::tui", "channel::apps")
    await pubsub.subscribe("channel::agent")

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
            await asyncio.sleep(0.01)


async def main_async():
    # ros_task = asyncio.create_task(ros_loop(list(nodes)))
    redis_task = asyncio.create_task(redis_loop())
    await asyncio.wait([redis_task])
    # await asyncio.gather(ros_task, redis_task)


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
    # ap.add_argument(
    #     "--redis-ip",
    #     type=ipaddress.ip_address,
    #     default="127.0.0.1",
    #     help="IP address to connect to (e.g., 192.168.1.1",
    # )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    # redis_ip = args.get("redis-ip", "127.0.0.1")
    # redis_ip = ipaddress.ip_address(redis_ip)
    # redis = Redis.from_url(f"redis://{redis_ip}")
    # node = ActionClientNode(debug=args.get("debug", False))

    agent = Agent(debug=args.get("debug", False))
    # action_client = ActionClientNode(debug=args.get("debug", False))
    session = ThreadedSession(node=agent)
    set_auto_session(session)
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        auto_session().close()
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
