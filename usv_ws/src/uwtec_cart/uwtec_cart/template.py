from typing import cast
import time
import asyncio
import aioserial
import async_timeout
import datetime

import rclpy
from rclpy.node import Node
from uwtec_interfaces.msg import CustomNavSat

from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
)


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator")
        self.get_logger().info("Navigator node has been started.")

        self.nav_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.nav_callback, 1
        )
        self.periodic_timer = self.create_timer(3.0, self.periodic_task)
        self.ticks = 0

    def nav_callback(self, msg: CustomNavSat):
        self.get_logger().info(
            f"Received NavSat: Lat {msg.latitude:.6f}, Lon {msg.longitude:.6f}, "
            f"Heading {msg.heading:.2f}, Sats {msg.num_sats}"
        )

    def periodic_task(self):
        self.ticks += 1
        print(f"live. Tick: {self.ticks}")
        print(f"Current time: {datetime.datetime.now()}")
        # await asyncio.sleep(3)
        #

    async def task3_async(self):
        while rclpy.ok():
            print("*", end="", flush=True)
            await asyncio.sleep(1.0)


class SideNode(Node):
    def __init__(self):
        super().__init__("side_node")
        self.get_logger().info("SideNode has been started.")

        self.create_timer(2.0, self.side_task)

    def side_task(self):
        self.get_logger().info(
            "SideNode is doing its own work independently of Navigator..."
        )

    async def side_task_async(self):
        while rclpy.ok():
            print("#", end="", flush=True)
            print(self.get_clock().now().to_msg())
            await asyncio.sleep(5.0)


async def task1_async():
    # with auto_session().lock() as session:
    #     node = cast(Navigator, session)
    while rclpy.ok():
        print("+", end="", flush=True)
        await asyncio.sleep(1.0)


async def task2_async():
    while rclpy.ok():
        print("-", end="", flush=True)
        await asyncio.sleep(1.0)


async def main_async():
    with auto_session().lock() as session:
        node = cast(Navigator, session)
    side_node = SideNode()
    side_session = ThreadedSession(node=side_node)
    side_session.start()

    task1 = asyncio.create_task(task1_async())
    task2 = asyncio.create_task(task2_async())
    task3 = asyncio.create_task(node.task3_async())
    task4 = asyncio.create_task(side_node.side_task_async())
    await asyncio.gather(task1, task2, task3, task4)

    #
    # print("Navigator is starting...")
    # while rclpy.ok():
    #     print(".", end="", flush=True)
    #     await asyncio.sleep(0.01)


def main():
    rclpy.init()

    navigator = Navigator()
    navigator_session = ThreadedSession(node=navigator)
    set_auto_session(navigator_session)

    # side_node = SideNode()
    # side_session = ThreadedSession(node=side_node)
    # side_session.start()

    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        pass
    finally:
        auto_session().close()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
