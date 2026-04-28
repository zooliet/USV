import os
import argparse
from typing import cast
from ament_index_python.packages import get_package_share_directory

import asyncio
import async_timeout
from redis.asyncio import Redis
from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
)

import rclpy
from rclpy.node import Node
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand, SimpleNav, GeoLoc


from uwtec_cart.action_clients import (
    CalibrateGyroClient,
    TestRunClient,
    ShuttleRunClient,
    NavToWpsClient,
)

from uwtec_cart.utils import (
    get_config_value,
    set_config_value,
    calc_heading_from_yaw_and_offset,
)


class Agent(Node):
    def __init__(self, debug=False):
        super().__init__("agent_node")
        self.get_logger().info("Agent node has been started.")

        self.debug = debug
        self.redis = Redis.from_url("redis://localhost")

        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.yaw: float = 0.0
        self.gps_quality: int = 0
        self.num_sats: int = 0

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

        self.test_run_client = TestRunClient(self, SimpleNav, "test_run")
        self.calibrate_gyro_client = CalibrateGyroClient(
            self, SimpleCommand, "calibrate_gyro"
        )
        self.shuttle_run_client = ShuttleRunClient(self, GeoLoc, "shuttle_run")
        self.nav_to_wps_client = NavToWpsClient(self, SimpleCommand, "nav_to_wps")

    def gps_custom_callback(self, msg: CustomNavSat):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.yaw = msg.heading  # ccw positive, 0 ~ 360
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats

    async def redis_loop(self):
        pubsub = self.redis.pubsub()
        await pubsub.subscribe("channel::agent")
        while rclpy.ok():
            try:
                async with async_timeout.timeout(1):
                    message = await pubsub.get_message(ignore_subscribe_messages=True)
                    if message:
                        await self.process(message)
            except asyncio.TimeoutError:
                pass
            finally:
                await asyncio.sleep(1.0)

    async def process(self, message):
        self.get_logger().debug(f"Received message: {message}")
        data = message["data"].decode().split(":")
        cmd = data[0]
        params = [] if len(data) == 1 else data[1:]

        if cmd == "ping":
            await self.response_with_pong()

        elif cmd == "get":
            await self.get_params(params)

        elif cmd == "set":
            # gryo-offset, linear-speed, angular-speed
            await self.set_params(params)

        elif cmd == "poweroff":
            self.get_logger().info("Received poweroff command.")
            executable_py = os.path.join(
                get_package_share_directory("uwtec_cart"),
                "script",
                "uwtec_poweroff.py",
            )
            os.system(f"python {executable_py}")

        elif cmd == "reboot":
            self.get_logger().info("Received reboot command.")
            executable_py = os.path.join(
                get_package_share_directory("uwtec_cart"), "script", "uwtec_reboot.py"
            )
            os.system(f"python {executable_py}")

        elif (
            cmd == "test-run"
        ):  # e.g., tes-run:forward:20:10, test-run:turn:0:90, test-run:stop
            self.get_logger().info("Received test-run command.")
            # Cancel any existing goal if parameters are insufficient
            if len(params) == 0 or params[0] not in [
                "forward",
                "stop",
                "turn",
                "drive-to",
                "nav-to",
            ]:
                self.test_run_client.cancel()
            else:
                cmd = params[0]
                try:
                    distance = float(params[1])
                    angle = float(params[2])
                except (IndexError, ValueError):
                    distance = 0.0
                    angle = 0.0
                self.test_run_client.action(cmd, distance, angle)

        elif cmd == "calibrate-gyro":
            self.get_logger().info("Received calibrate-gyro command.")
            self.calibrate_gyro_client.action()  # this will toggle between sending a new goal and cancelling the existing one

        elif cmd == "shuttle-run":  # e.g., shuttle-run:37.719457:127.525468
            self.get_logger().info("Received shuttle-run command.")
            if len(params) != 2:
                self.shuttle_run_client.cancel()
            else:
                try:
                    latitude = float(params[0])
                    longitude = float(params[1])
                except (ValueError, IndexError):
                    self.get_logger().error(
                        "Invalid shuttle-run parameters. Expected: shuttle-run:<latitude>:<longitude>"
                    )
                else:
                    self.shuttle_run_client.action(latitude, longitude)

        elif cmd == "nav-to-wps":  # nav-to-wps:wps.yaml, nav-to-wps:songsanri.yaml
            self.get_logger().info("Received nav-to-wps command.")
            if len(params) < 1:
                self.nav_to_wps_client.cancel()
            else:
                route = params[0]
                self.nav_to_wps_client.action(route)

    async def response_with_pong(self):
        gyro_offset = get_config_value("gyro_offset", 0.0)
        current_heading = calc_heading_from_yaw_and_offset(self.yaw, gyro_offset)
        response = f"pong:{self.latitude}:{self.longitude}:{current_heading:.2f}:{self.yaw:.2f}:{self.gps_quality}:{self.num_sats}"
        await self.redis.publish("channel::gui", response)

    async def get_params(self, params):
        if len(params) < 1:
            self.get_logger().error("Invalid get command format.")
            response = "nok:Invalid get command format. Usage: get:<param_name>"
        else:
            param_name = params[0].replace("-", "_")
            if param_name in ["gyro_offset", "linear_speed", "angular_speed"]:
                default_value = "0.0"
            else:
                default_value = "undefined"
            param_value = get_config_value(param_name, default_value)
            response = f"ok:{params[0]}:{param_value}"
        await self.redis.publish("channel::gui", response)

    async def set_params(self, params):
        if len(params) < 2:
            self.get_logger().error("Invalid set command format.")
            response = (
                "nok:Invalid set command format. Usage: set:<param_name>:<param_value>"
            )
        else:
            param_name = params[0].replace("-", "_")

            try:
                if param_name in ["gyro_offset", "linear_speed", "angular_speed"]:
                    param_value = float(params[1])
                else:
                    param_value = params[1]
                set_config_value(param_name, param_value)
                response = f"ok:{params[0]}:{param_value}"
            except ValueError:
                response = (
                    f"nok:Invalid value for parameter {param_name}. Expected a number."
                )

        await self.redis.publish("channel::gui", response)


async def main_async():
    with auto_session().lock() as session:
        agent = cast(Agent, session)

    redis_task = asyncio.create_task(agent.redis_loop())
    await asyncio.gather(redis_task)


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
