import argparse
import pynmea2
from pynmea2 import GGA
import time
from typing import cast

import asyncio
import aioserial
import async_timeout

import rclpy
from rclpy.node import Node

from uwtec_interfaces.msg import CustomNavSat
from asyncio_for_robotics.ros2 import (
    ThreadedSession,
    auto_session,
    set_auto_session,
    # TopicInfo,
    # Sub,
)


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
        self.localizer_pub = self.create_publisher(CustomNavSat, "/gps/custom", 1)
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
        msg.num_sats = int(self.num_sats)
        msg.gps_quality = int(self.gps_qual)
        self.localizer_pub.publish(msg)


async def gnss_reader(port: aioserial.AioSerial):
    with auto_session().lock() as session:
        node = cast(Localizer, session)

    gnss_frame_no = 0
    gnss_frame_errors = 0
    start = time.perf_counter()

    while True:
        data: bytes = await port.readline_async()
        gnss_frame_no += 1
        try:
            frame = data.decode().strip()
        except UnicodeDecodeError:
            gnss_frame_errors += 1
        else:
            if frame.startswith("$") and "GGA" in frame:
                try:
                    msg: GGA = cast(GGA, pynmea2.parse(frame))
                    node.latitude = msg.latitude
                    node.longitude = msg.longitude
                    node.num_sats = cast(int, msg.num_sats)
                    node.gps_qual = cast(int, msg.gps_qual)
                except pynmea2.ChecksumError:
                    gnss_frame_errors += 1
                except Exception as _:
                    gnss_frame_errors += 1

        if (
            gnss_frame_no % 10 == 0 and node.local_debug
        ):  # Print stats every 10 frames, i.e., 1 sec
            eplased = time.perf_counter() - start
            start = time.perf_counter()
            node.get_logger().info(
                f"GNSS Frame Errors: {gnss_frame_errors}/{gnss_frame_no}"
            )
            node.get_logger().info(
                f"GNSS Processing Rate: {10 / eplased:.2f} frames/sec"
            )
            node.get_logger().info(
                f"GNSS rocessing time: {(eplased / 10):.4f} secs/frame"
            )


async def gyro_reader(port: aioserial.AioSerial):
    with auto_session().lock() as session:
        node = cast(Localizer, session)

    gyro_frame_no = 0
    gyro_frame_errors = 0
    start = time.perf_counter()

    while True:
        try:
            async with async_timeout.timeout(
                0.01
            ):  # optimal sampling freq: around 100 Hz
                # read 44 bytes at a time: 11B*3 + 11B
                frame = await port.read_async(size=44)
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


async def main_async(gnss_port, gyro_port, baudrate):
    gnss_device = aioserial.AioSerial(port=gnss_port, baudrate=baudrate)
    gnss_task = asyncio.create_task(gnss_reader(gnss_device))
    # await asyncio.wait([gnss_task])

    gyro_device = aioserial.AioSerial(port=gyro_port, baudrate=baudrate)
    gyro_task = asyncio.create_task(gyro_reader(gyro_device))
    # await asyncio.wait([gyro_task])

    await asyncio.wait([gnss_task, gyro_task])


def main(args=None):
    rclpy.init(args=args)

    ap = argparse.ArgumentParser()
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
    localizer_session = ThreadedSession(node=localizer)
    set_auto_session(localizer_session)
    try:
        asyncio.run(
            main_async(
                gnss_port=args["gnss_port"],
                gyro_port=args["gyro_port"],
                baudrate=args["baudrate"],
            )
        )
    except KeyboardInterrupt:
        pass
    finally:
        auto_session().close()
        localizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
