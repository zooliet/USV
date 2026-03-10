import asyncio

import rclpy
from rclpy.node import Node

from uwtec_navigation.action_servers import (
    ShuttleRunServer,
    HeadingAndOffsetServer,
    NavToWpsServer
)

async def ros_loop(nodes):
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        while executor.context.ok():
            # while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)  # 0.1 ?
            await asyncio.sleep(0.001)
    finally:
        for node in nodes:
            executor.remove_node(node)
        executor.shutdown()


async def main_async(*nodes):
    ros_tasks = asyncio.create_task(ros_loop(list(nodes)))
    await ros_tasks


def main(args=None):
    rclpy.init(args=args)

    # ap = argparse.ArgumentParser()
    # ap.add_argument(
    #     "-i", "--interval", type=float, default=0.1, help="timer interval: 0.1*"
    # )
    # ap.add_argument(
    #     "--angular-speed",
    #     type=float,
    #     default=0.5,
    #     help="Angular speed in [0-1] (default: 0.5*)",
    # )
    # ap.add_argument(
    #     "--linear-speed",
    #     type=float,
    #     default=0.5,
    #     help="Linear speed in [0-1] (default: 0.5*)",
    # )
    # ap.add_argument(
    #     "--debug", action="store_true", help="Enable debug mode (default: False*)"
    # )
    # options, _ = ap.parse_known_args()
    # # args = vars(ap.parse_args())
    # args = vars(options)
    # print(args)

    shuttle_run = ShuttleRunServer()
    nav_to_wps = NavToWpsServer()
    headinig_and_offset = HeadingAndOffsetServer() 

    # nodes = [shuttle_run, nav_to_wps, headinig_and_offset]
    try:
        asyncio.run(main_async(shuttle_run, nav_to_wps, headinig_and_offset))
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        shuttle_run.destroy_node()
        nav_to_wps.destroy_node()
        headinig_and_offset.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

