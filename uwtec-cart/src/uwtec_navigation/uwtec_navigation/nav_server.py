import asyncio
import rclpy

from uwtec_navigation.action_servers import (
    ShuttleRunServer,
    HeadingAndOffsetServer,
    NavToWpsServer,
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

    shuttle_run = ShuttleRunServer()
    nav_to_wps = NavToWpsServer()
    headinig_and_offset = HeadingAndOffsetServer()

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
