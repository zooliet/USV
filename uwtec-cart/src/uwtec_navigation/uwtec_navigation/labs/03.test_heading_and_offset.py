import argparse
import time
import os
import yaml
from datetime import datetime
from pyproj import Transformer
from enum import Enum
from ament_index_python.packages import get_package_share_directory

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand
from uwtec_navigation.utils import (
    utm_bearing,
    # signed_angle,
    calc_offset,
    calc_heading_by_offset,
    calc_goal_heading,
    rotate_to_go,
)


class OperationMode(Enum):
    START_OVER = 1
    RUNNING = 2
    PAUSED = 3
    TURN_AROUND = 4
    FINISHED = 5


class DemoNode(Node):
    def __init__(self, interval=0.1, angular_speed=0.5, linear_speed=0.5, debug=False):
        super().__init__("test_heading_and_offset_node")
        self.interval = interval
        self.angular_speed = angular_speed
        self.linear_speed = linear_speed
        self.debug = debug

        self.latitude = 36.5665
        self.longitude = 127.9780
        self.yaw = 0.0
        self.gps_quality = 0
        self.num_sats = 0

        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32652", always_xy=True
        )
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.heading_yaml_path = os.path.join(
            get_package_share_directory("uwtec_navigation"), "config", "heading.yaml"
        )

        self.action_server = ActionServer(
            self,
            SimpleCommand,
            "test_heading_and_offset",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.cmd_vel_nav_pub = self.create_publisher(
            Twist,
            "/cmd_vel_nav",
            # "/cmd_vel",  # 임시로 테스트 할 때
            1,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

    def go_forward(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed
        twist_msg.linear.y = self.linear_speed
        self.cmd_vel_nav_pub.publish(twist_msg)

    def stop_drive(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.01
        twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def turn_around(self, angle):
        twist_msg = Twist()
        if angle > 0:  # ccw turn, turn left
            twist_msg.linear.x = 0.01
            twist_msg.linear.y = self.angular_speed
        else:  # cw turn, turn right
            twist_msg.linear.x = self.angular_speed
            twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def turn_right(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.angular_speed
        twist_msg.linear.y = 0.01
        self.cmd_vel_nav_pub.publish(twist_msg)

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.yaw = -msg.heading % 360  # convert to CCW positive
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats
        # self.heading = east_facing_ccw_angle(self.heading)

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = SimpleCommand.Feedback()
        result = SimpleCommand.Result()

        # initialize local variables
        mode = OperationMode.START_OVER
        start_utm_x, start_utm_y = 0.0, 0.0
        end_utm_x, end_utm_y = 0.0, 0.0
        offset = 0.0
        goal_heading = 0.0
        offsets = []
        runs = 0
        num_runs = 4

        # Reset ticks for each execution
        ticks_for_5_secs = int(5.0 / self.interval)
        ticks_for_1_sec = int(1.0 / self.interval)

        rate = self.create_rate(int(1.0 / self.interval))  # Hz
        while rclpy.ok():  # or while True:
            if mode == OperationMode.START_OVER:
                if ticks_for_1_sec % 10 == 0:
                    disp_sec = ticks_for_5_secs // 10
                    self.get_logger().info(f"Starting in {disp_sec} seconds...")
                if ticks_for_5_secs % 50 == 1:
                    runs += 1
                    # save the current position as the starting point
                    start_utm_x, start_utm_y = self.utm_x, self.utm_y
                    # start moving forward at the specified linear speed
                    self.go_forward()
                    # switch to running mode and reset 5 second timer
                    mode = OperationMode.RUNNING
                    ticks_for_5_secs = int(5.0 / self.interval)
            elif mode == OperationMode.RUNNING:
                # check if 5 seconds have passed
                if ticks_for_5_secs % 50 == 1:
                    # stop the robot
                    self.stop_drive()
                    # switch to paused mode
                    mode = OperationMode.PAUSED
            elif mode == OperationMode.PAUSED:
                # save the current position as the end point
                end_utm_x, end_utm_y = self.utm_x, self.utm_y
                # calculate the bearing between the start and end points
                bearing = utm_bearing(
                    (start_utm_x, start_utm_y), (end_utm_x, end_utm_y)
                )
                # calculate the offset angle based on the bearing
                offset = calc_offset(bearing, self.yaw)
                heading = calc_heading_by_offset(self.yaw, offset)
                offsets.append(offset)
                if runs == num_runs:
                    mode = OperationMode.FINISHED
                else:
                    # switch to turn around mode
                    mode = OperationMode.TURN_AROUND
                    goal_heading = calc_goal_heading(heading, by=90)
                    ticks_for_5_secs = int(5.0 / self.interval)
            elif mode == OperationMode.TURN_AROUND:
                # find the goal angle to turn 90 degrees to the left
                # calculate the shortest angle to turn
                current_heading = calc_heading_by_offset(self.yaw, offset)
                remaining_angle = rotate_to_go(current_heading, goal_heading)
                if (
                    abs(remaining_angle) < 3 or ticks_for_5_secs % 50 == 1
                ):  # within 3 degrees of the goal
                    # stop the robot
                    self.stop_drive()
                    # switch to start-over mode
                    mode = OperationMode.START_OVER
                else:
                    # keep turning towards the goal
                    self.turn_around(remaining_angle)
            elif mode == OperationMode.FINISHED:
                # action is complete, break the loop
                self.offset = sum(offsets) / len(offsets) if offsets else 0.0
                self.get_logger().info(
                    f"Average offset after {runs} runs: {self.offset:.2f} degrees"
                )
                with open(self.heading_yaml_path, "w") as heading_file:
                    yaml.dump({"offset": self.offset}, heading_file, sort_keys=False)
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_drive()
                self.get_logger().info("test_heading_and_offset action canceled")
                result.success = False
                return result

            feedback.progress = 0  # Update progress
            goal_handle.publish_feedback(feedback)

            ## tick updates
            ticks_for_5_secs = (
                ticks_for_5_secs - 1
                if ticks_for_5_secs > 1
                else int(5.0 / self.interval)
            )
            ticks_for_1_sec = (
                ticks_for_1_sec - 1
                if ticks_for_1_sec > 1.0
                else int(1.0 / self.interval)
            )
            if ticks_for_1_sec % 10 == 0 and self.debug and mode != OperationMode.START_OVER:
                # current_time = datetime.now().strftime("%H:%M:%S")
                # self.get_logger().info(current_time)
                self.get_logger().info(
                    f"{runs}/{num_runs}, {mode.name}"
                )

            try:
                rate.sleep()
            except Exception as _:
                # Handle case where ROS context shuts down
                result.success = False
                return result

        goal_handle.succeed()
        self.get_logger().info("test_heading_and_offset action completed successfully")
        result.success = True
        return result


async def ros_loop(nodes):
    from rclpy.executors import MultiThreadedExecutor

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        while executor.context.ok():
            # while rclpy.ok():
            executor.spin_once(timeout_sec=0.001)
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

    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-i", "--interval", type=float, default=0.1, help="timer interval: 0.1*"
    )
    ap.add_argument(
        "--angular-speed",
        type=float,
        default=0.5,
        help="Angular speed in [0-1] (default: 0.5)",
    )
    ap.add_argument(
        "--linear-speed",
        type=float,
        default=0.5,
        help="Linear speed in [0-1] (default: 0.5)",
    )
    ap.add_argument(
        "--debug", action="store_true", help="Enable debug mode (default: False)"
    )
    options, _ = ap.parse_known_args()
    # args = vars(ap.parse_args())
    args = vars(options)
    print(args)

    node = DemoNode(**args)
    try:
        asyncio.run(main_async(node))
    except KeyboardInterrupt:
        print("Shutting down nodes...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
