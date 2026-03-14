import argparse
import pynmea2
import time
# from typing import cast

import asyncio
import aioserial
import async_timeout

import rclpy
from rclpy.node import Node

from uwtec_interfaces.msg import CustomNavSat


class Localizer(Node):
    def __init__(self, debug=False, local_debug=False):
        super().__init__("localizer")
        self.get_logger().info("Localizer node has been started.")
        self.latitude = 36.5665
        self.longitude = 127.9780
        self.num_sats = 0
        self.gps_qual = 0
        self.gyro_heading = 0.0
        self.ticks = 0
        self.debug = debug
        self.local_debug = local_debug

        # pub topics
        self.localizer_pub = self.create_publisher(CustomNavSat, "/gps/custom", 10)

        # timer for every 100ms sec
        self.pub_timer = self.create_timer(0.1, self.pub_task)

    def pub_task(self):
        self.ticks += 1
        if self.ticks % 10 == 0 and self.debug:  # Print stats every 1 second
            self.get_logger().info(
                f"Latitude: {self.latitude}, Longitude: {self.longitude}, Satellites: {self.num_sats}, GPS Quality: {self.gps_qual}, Gyro Heading: {self.gyro_heading:.2f}"
            )

        this_time = self.get_clock().now().to_msg()
        msg = CustomNavSat()
        msg.header.stamp = this_time
        msg.header.frame_id = "gps_link"
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.heading = self.gyro_heading
        msg.num_sats = self.num_sats
        msg.gps_quality = self.gps_qual
        self.localizer_pub.publish(msg)


async def gnss_reader(gnss_device, node):
    gnss_frame_no = 0
    gnss_frame_errors = 0
    start = time.perf_counter()

    while True:
        data: bytes = await gnss_device.readline_async()  # around 0.1 from GPGGA 0.1
        gnss_frame_no += 1
        try:
            frame = data.decode().strip()
        except UnicodeDecodeError:
            gnss_frame_errors += 1
        else:
            if frame.startswith("$") and "GGA" in frame:
                try:
                    msg = pynmea2.parse(frame)
                except pynmea2.ChecksumError:
                    gnss_frame_errors += 1
                except Exception as _:
                    gnss_frame_errors += 1
                else:
                    node.latitude = msg.latitude
                    node.longitude = msg.longitude
                    node.num_sats = int(msg.num_sats)
                    node.gps_qual = int(msg.gps_qual)

        if gnss_frame_no % 10 == 0 and node.local_debug:  # every 10 frames == 1 sec
            eplased = time.perf_counter() - start
            start = time.perf_counter()
            node.get_logger().info(
                f"GNSS Frame Errors: {gnss_frame_errors}/{gnss_frame_no}"
            )
            node.get_logger().info(
                f"GNSS Processing Rate: {10 / eplased:.2f} frames/sec"
            )
            node.get_logger().info(
                f"GNSS Processing time: {(eplased / 10):.4f} secs/frame"
            )


async def gyro_reader(gyro_device, node):
    gyro_frame_no = 0
    gyro_frame_errors = 0
    start = time.perf_counter()

    while True:
        try:
            async with async_timeout.timeout(
                0.01
            ):  # optimal sampling freq: around 100 Hz
                # read 44 bytes at a time: 11B*3 + 11B
                frame = await gyro_device.read_async(size=44)
                gyro_frame_no += 1
                rpy = []
                for i in range(44 - 10):  # i: 0 - 33
                    if frame[i] == 85 and frame[i + 1] == 83:
                        rpy = frame[i : i + 11]
                        check_sum = sum(rpy[:-1]) % 256
                        if check_sum == rpy[-1]:
                            gyro_heading = (rpy[6] + rpy[7] * 256) / 32768 * 180
                            node.gyro_heading = -(gyro_heading % -360)
                        break
        # except asyncio.TimeoutError:
        except Exception as _:
            gyro_frame_errors += 1

        await asyncio.sleep(0.01)  # yield for other tasks

        if gyro_frame_no % 10 == 0 and node.local_debug:  # every 10 frames == 0.01+⍺
            eplased = time.perf_counter() - start
            start = time.perf_counter()
            node.get_logger().info(
                f"Gyro Frame Errors: {gyro_frame_errors}/{gyro_frame_no}"
            )
            node.get_logger().info(
                f"Gyro Processing Rate: {10 / eplased:.2f} frames/sec"
            )
            node.get_logger().info(
                f"Gyro Processing time: {(eplased / 10):.4f} secs/frame"
            )
            # print(
            #     f"Frame No: {gyro_frame_no}\n{list(rpy)}\nHeading: {node.gyro_heading:.2f}"
            # )


async def ros_loop(nodes):
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        while executor.context.ok():
            # while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)  # 0.001
            await asyncio.sleep(0.004)
    finally:
        for node in nodes:
            executor.remove_node(node)
        executor.shutdown()


async def main_async(node, gnss_port, gyro_port, baudrate):
    ros_tasks = asyncio.create_task(ros_loop([node]))
    # await ros_tasks

    gnss_device = aioserial.AioSerial(port=gnss_port, baudrate=baudrate)
    gnss_task = asyncio.create_task(gnss_reader(gnss_device, node=node))
    # await asyncio.wait([gnss_task])

    gyro_device = aioserial.AioSerial(port=gyro_port, baudrate=baudrate)
    gyro_task = asyncio.create_task(gyro_reader(gyro_device, node=node))
    # await asyncio.wait([gyro_task])

    await asyncio.wait([gnss_task, gyro_task, ros_tasks])


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser(description="Localizer for the project")
    ap.add_argument("--gnss-port", default="/dev/ttyGNSS", help="UM982 device path")
    ap.add_argument("--gyro-port", default="/dev/ttyGYRO", help="MPU6050 device path")
    ap.add_argument(
        "-b", "--baudrate", type=int, default=115200, help="current baudrate"
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )
    ap.add_argument(
        "--local-debug",
        action="store_true",
        help="Enable local debug mode (default: False)",
    )

    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    localizer = Localizer(
        debug=args.get("debug", False), local_debug=args.get("local_debug", False)
    )
    try:
        asyncio.run(
            main_async(
                node=localizer,
                gnss_port=args.get("gnss_port", "/dev/ttyGNSS"),
                gyro_port=args.get("gyro_port", "/dev/ttyGYRO"),
                baudrate=args.get("baudrate", 115200),
            )
        )
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        localizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
