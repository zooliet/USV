import argparse
import os
import yaml
from typing import cast
from itertools import pairwise
from ament_index_python.packages import get_package_share_directory
# from pyproj import Transformer

import asyncio
import async_timeout
from redis.asyncio import Redis

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand, GeoLoc

from uwtec_agent.action_clients import (
    HeadingAndOffsetClient,
    ShuttleRunClient,
    NavToWpsClient,
)

from uwtec_navigation.utils import (
    calc_heading_by_offset,
    get_gyro_offset,
)


from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
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
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_joy_uros", 10)
        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)

        self.action_client_heading_and_offset = HeadingAndOffsetClient(
            self, SimpleCommand, "heading_and_offset"
        )

        self.action_client_shuttle_run = ShuttleRunClient(self, GeoLoc, "shuttle_run")

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
        data = message["data"].decode().split(":")
        cmd = data[0]
        params = [] if len(data) == 1 else data[1:]

        if cmd == "ping":
            gyro_offset = get_gyro_offset("heading.yaml")
            current_heading = calc_heading_by_offset(self.yaw, gyro_offset)
            response = f"pong,{self.latitude},{self.longitude},{current_heading:.2f},{self.yaw:.2f},{self.gps_quality},{self.num_sats}"
            await self.redis.publish("channel::tui", response)

        elif cmd == "poweroff":
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

        elif cmd == "stop":
            twist_msg = Twist()
            twist_msg.linear.x = 0.01
            twist_msg.linear.y = 0.01
            self.cmd_vel_pub.publish(twist_msg)
            # self.get_logger().info("Emergency stop command issued.")

        elif cmd == "test-run":
            twist_msg = Twist()
            twist_msg.linear.x = 0.5
            twist_msg.linear.y = 0.5
            self.cmd_vel_pub.publish(twist_msg)
            # self.get_logger().info("test-run command issued.")

        elif cmd == "upload" and params[0] == "wps":
            try:
                float(params[1])
            except (ValueError, IndexError):
                wps_name = params[1]
                params_for_coords = params[2:]
            else:
                wps_name = "wps.yaml"
                params_for_coords = params[1:]

            if len(params_for_coords) % 2 != 0 or len(params_for_coords) < 2:
                self.get_logger().info(
                    "Invalid number of parameters for upload command"
                )
                return

            coords = []
            for lat, lon in pairwise(params_for_coords):
                coord = {"latitude": float(lat), "longitude": float(lon)}
                coords.append(coord)

            wps_yaml_path = os.path.join(
                get_package_share_directory("uwtec_navigation"), "config", wps_name
            )
            with open(wps_yaml_path, "w") as wps_file:
                yaml.dump(coords, wps_file, sort_keys=False)

        elif cmd == "clear" and params[0] == "wps":
            wps_name = params[1] if len(params) > 1 else "wps.yaml"
            wps_yaml_path = os.path.join(
                get_package_share_directory("uwtec_navigation"), "config", wps_name
            )
            # remove the file or clear its contents
            os.remove(wps_yaml_path)

        elif cmd == "append" and params[0] == "wps":
            wps_name = params[1] if len(params) > 1 else "wps.yaml"
            coord = {"latitude": self.latitude, "longitude": self.longitude}
            wps_yaml_path = os.path.join(
                get_package_share_directory("uwtec_navigation"), "config", wps_name
            )
            if os.path.exists(wps_yaml_path):
                with open(wps_yaml_path, "r") as wps_file:
                    existing_coords = yaml.safe_load(wps_file) or []
            else:
                existing_coords = []
            existing_coords.append(coord)
            with open(wps_yaml_path, "w") as wps_file:
                yaml.dump(existing_coords, wps_file, sort_keys=False)

        elif cmd == "action" and params[0] == "gyro-offset":
            self.action_client_heading_and_offset.action()
        elif cmd == "cancel" and params[0] == "gyro-offset":
            self.action_client_heading_and_offset.cancel()

        elif cmd == "action" and params[0] == "shuttle-run":
            if len(params[1:]) != 2:
                self.get_logger().info(
                    "Invalid number of parameters for shuttle_run action"
                )
                return
            latitude = float(params[1])
            longitude = float(params[2])
            self.action_client_shuttle_run.action(latitude, longitude)
        elif cmd == "cancel" and params[0] == "shuttle-run":
            self.action_client_shuttle_run.cancel()

        elif cmd == "action" and params[0] == "wpf":
            wps_name = params[1] if len(params) > 1 else "wps.yaml"
            self.action_client_wpf.action(wps_name)
        elif cmd == "cancel" and params[0] == "wpf":
            self.action_client_wpf.cancel()


async def redis_loop():
    with auto_session().lock() as session:
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
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    agent = Agent(debug=args.get("debug", False))
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
