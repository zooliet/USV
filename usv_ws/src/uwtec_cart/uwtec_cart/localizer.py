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
)

# assume GNSS data comes at 10 Hz and gyro data comes at 100 Hz
GNSS_SAMPLING = 0.1  # 0.1 sec, 100 ms, 10 Hz
GYRO_SAMPLING = 0.01  # 0.01 sec, 10 ms, 100 Hz

# for debugging: print stats every 100 frames for GNSS and every 1000 frames for gyro
GNSS_DEBUG_INTERVAL = 100  # every 100 frames: 0.1 * 100 = 10 sec
GYRO_DEBUG_INTERVAL = 1000  # every 1000 frames: 0.01 x 1000 == 10 sec


class Localizer(Node):
    def __init__(self, interval=0.01, debug=False):
        super().__init__("localizer")
        self.get_logger().info("Localizer node has been started.")

        self.interval: float = interval
        self.debug: bool = debug

        # Initialize GPS-related attributes - will be updated by GPS callback
        self.latitude: float = 37.719457
        self.longitude: float = 127.525468
        self.prev_latitude: float = 37.719457
        self.prev_longitude: float = 127.525468

        self.num_sats: int = 0
        self.gps_qual: int = 0
        self.gyro_heading: float = 0.0

        self.ticks = 0
        self.debug_ticks = 1 / self.interval  # print debug info every 1 second
        self.pub_timer = self.create_timer(self.interval, self.pub_task)
        self.localizer_pub = self.create_publisher(CustomNavSat, "/gps/custom", 1)

    def pub_task(self):
        self.ticks += 1
        if self.debug and self.ticks % self.debug_ticks == 0:
            self.get_logger().info(
                f"Latitude: {self.latitude:.6f}, Longitude: {self.longitude:.6f}, "
                f"Num Sats: {self.num_sats}, GPS Qual: {self.gps_qual}, "
                f"Gyro Heading: {self.gyro_heading:.2f}"
            )

        if self.latitude == 0.0 or self.longitude == 0.0:
            self.latitude = self.prev_latitude
            self.longitude = self.prev_longitude
        else:
            self.prev_latitude = self.latitude
            self.prev_longitude = self.longitude

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
                    if type(msg.num_sats) is int:
                        node.num_sats = msg.num_sats
                    if type(msg.gps_qual) is int:
                        node.gps_qual = msg.gps_qual
                except pynmea2.ChecksumError:
                    gnss_frame_errors += 1
                except Exception as _:
                    gnss_frame_errors += 1

        # print stats every 100 frames: 0.1 * 100 = 1.0 sec
        if gnss_frame_no % GNSS_DEBUG_INTERVAL == 0 and node.debug:
            eplased = time.perf_counter() - start
            start = time.perf_counter()
            node.get_logger().info(
                f"GNSS Frame Errors: {gnss_frame_errors}/{gnss_frame_no}"
            )
            node.get_logger().info(
                f"GNSS Processing Rate: {GNSS_DEBUG_INTERVAL / eplased:.2f} frames/sec"
            )
            node.get_logger().info(
                f"GNSS rocessing time: {(eplased / GNSS_DEBUG_INTERVAL):.4f} secs/frame"
            )


async def gyro_reader(port: aioserial.AioSerial):
    with auto_session().lock() as session:
        node = cast(Localizer, session)

    gyro_frame_no = 0
    gyro_frame_errors = 0
    start = time.perf_counter()
    rpy = []  # roll, pitch, yaw (heading)
    continued = False

    while True:
        try:
            async with async_timeout.timeout(
                0.02
            ):  # optimal sampling freq: around 100 Hz
                # read 33 bytes at a time: 11B*3
                frame = await port.read_async(size=33)
                gyro_frame_no += 1
                if continued:
                    frame = rpy + list(frame[:11])
                for i in range(33):  # i: 0 - 32
                    if frame[i] == 85 and frame[i + 1] == 83:
                        rpy = list(frame[i : i + 11])
                        if len(rpy) < 11:
                            continued = True
                            break
                        else:
                            check_sum = sum(rpy[:-1]) % 256
                            if check_sum == rpy[-1]:
                                node.gyro_heading = (
                                    (rpy[6] + rpy[7] * 256) / 32768 * 180
                                )
                            else:
                                gyro_frame_errors += 1
                            continued = False
        except Exception as _:
            gyro_frame_errors += 1

        await asyncio.sleep(GYRO_SAMPLING - 0.002)  # yield for other tasks

        # print stats every 500 frames: 0.02 x 500 == 10 sec
        if gyro_frame_no % GYRO_DEBUG_INTERVAL == 0 and node.debug:
            eplased = time.perf_counter() - start
            start = time.perf_counter()
            node.get_logger().info(
                f"Gyro Frame Errors: {gyro_frame_errors}/{gyro_frame_no}"
            )
            node.get_logger().info(
                f"Gyro Processing Rate: {GYRO_DEBUG_INTERVAL / eplased:.2f} frames/sec"
            )
            node.get_logger().info(
                f"Gyro Processing time: {(eplased / GYRO_DEBUG_INTERVAL):.4f} secs/frame"
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
        "--interval", type=float, default=0.01, help="publishing interval in seconds"
    )
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
        interval=args.get("interval", 0.01), debug=args.get("debug", False)
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
