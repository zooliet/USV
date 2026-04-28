import asyncio
import rclpy

from uwtec_cart.action_servers import (
    CalibrateGyroServer,
    TestRunServer,
    ShuttleRunServer,
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
            executor.spin_once(timeout_sec=0.1)  # 0.1 ?
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

    calibrate_gyro_server = CalibrateGyroServer()
    test_run_server = TestRunServer()
    # shuttle_run_server = ShuttleRunServer()
    # nav_to_wps_server = NavToWpsServer()

    try:
        asyncio.run(
            main_async(
                calibrate_gyro_server,
                test_run_server,
                # shuttle_run_server,
                # nav_to_wps_server,
            )
        )
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        calibrate_gyro_server.destroy_node()
        test_run_server.destroy_node()
        # shuttle_run_server.destroy_node()
        # nav_to_wps_server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
