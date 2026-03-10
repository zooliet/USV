import argparse
import math
from enum import Enum
# from datetime import datetime

import asyncio

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import GeoLoc
from uwtec_navigation.utils import (
    utm_distance,
    distance_and_bearing_xy,
    shortest_distance_and_angle_to_line,
    rotate_to_go,
)


class OperationMode(Enum):
    START_OVER = 1
    RUNNING = 2
    PAUSED = 3
    TURN_AROUND = 4
    FINISHED = 5


class DrivingMode(Enum):
    FORWARD = 1
    HARD_LEFT_FORWARD = 2
    HARD_RIGHT_FORWARD = 3
    MILD_LEFT_FORWARD = 4
    MILD_RIGHT_FORWARD = 5
    TURN_AROUND = 6
    STOP = 7


class DemoNode(Node):
    def __init__(self, interval=0.1, debug=False):
        super().__init__("test_driving_sim_node")
        self.interval = interval
        self.debug = debug
        self.debug_str = ""
        self.lining_up_request = False  # set to True when we need to line up with the goal heading before driving straight to the destination
        self.driving_mode_disp = DrivingMode.STOP.name

        self.action_server = ActionServer(
            self,
            GeoLoc,
            "test_driving_sim",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

    def driving(self, p_src, p_dst, p_current, current_heading):
        distance, goal_heading = distance_and_bearing_xy(p_src, p_dst)
        distance_from_src, angle_from_src = distance_and_bearing_xy(p_src, p_current)
        distance_to_dst, angle_to_dst = distance_and_bearing_xy(p_current, p_dst)
        distance_to_path, angle_to_path = shortest_distance_and_angle_to_line(
            p_src, p_dst, p_current
        )

        if self.debug:
            self.debug_str = (
                f"Planned:   {distance:05.2f} m, {goal_heading:05.2f} deg\n"
                + f"Travelled: {distance_from_src:05.2f} m, {angle_from_src:05.2f} deg\n"
                + f"Remaining: {distance_to_dst:05.2f} m, {angle_to_dst:05.2f} deg\n"
                + f"Deviation: {distance_to_path:05.2f} m, {angle_to_path:05.2f} deg\n"
                + f"Heading:   {current_heading:05.2f} deg at ({self.sim_utm_x:.2f}, {self.sim_utm_y:.2f})"
                + f" => {self.driving_mode_disp}"
            )

        if distance_to_dst < 0.2:
            self.stop_drive()
            return distance_to_dst

        if self.lining_up_request:
            remaining_angle = rotate_to_go(current_heading, goal_heading)
            self.turn_around(remaining_angle)
            return distance_to_dst

        self.lining_up_request = False  # reset lining up request just in case
        if distance_to_path < 0.2:  # within 20 cm
            remaining_angle = rotate_to_go(current_heading, goal_heading)
            # if the angle is too large, we might be going in the wrong direction, so let's turn around first before driving straight to the destination
            if abs(remaining_angle) > 45.0:
                self.turn_around(remaining_angle)
            # if the angle is moderately large, we can turn while driving
            elif abs(remaining_angle) > 6.0:
                driving_mode = (
                    DrivingMode.MILD_LEFT_FORWARD
                    if remaining_angle > 0
                    else DrivingMode.MILD_RIGHT_FORWARD
                )
                self.go_forward(driving_mode, distance=distance_to_dst)
            # if the angle is small, we can just drive straight
            else:
                self.go_forward(DrivingMode.FORWARD, distance=distance_to_dst)

        elif distance_to_path < 1.0:  # within 1 meter
            remaining_angle = rotate_to_go(current_heading, angle_to_dst)
            # if the angle is too large, we might be going in the wrong direction, so let's turn around first before driving straight to the destination
            if abs(remaining_angle) > 45.0:
                self.turn_around(remaining_angle)
            # if the angle is moderately large, we can turn while driving
            elif abs(remaining_angle) > 10.0:
                direction = (
                    DrivingMode.HARD_LEFT_FORWARD
                    if remaining_angle > 0
                    else DrivingMode.HARD_RIGHT_FORWARD
                )
                self.go_forward(direction, distance=distance_to_dst)
            else:
                self.go_forward(DrivingMode.FORWARD, distance=distance_to_dst)

        else:  # more than 1 meter away from the path
            remaining_angle = rotate_to_go(current_heading, angle_to_path)
            if abs(remaining_angle) < 10:  # already facing the line closely enough
                self.go_forward(DrivingMode.FORWARD, distance=distance_to_path)
            else:  # need to turn toward the face the line
                self.turn_around(remaining_angle)

        return distance_to_dst

    def go_forward(self, driving_mode, distance=0.0):
        # In a real implementation, this would publish a Twist message to move the robot forward.
        # In this simulation, we can simply update the current position based on the heading and a fixed speed.
        self.driving_mode_disp = driving_mode.name + f"({distance:.2f} m)"
        if distance < 1.0:  # slow down when close to the destination
            speed = 0.5  # m/s
        else:
            speed = 1.0  # m/s

        if driving_mode == DrivingMode.FORWARD:
            pass
        elif driving_mode == DrivingMode.MILD_LEFT_FORWARD:
            self.sim_heading = (self.sim_heading + 1.0) % 360
        elif driving_mode == DrivingMode.MILD_RIGHT_FORWARD:
            self.sim_heading = (self.sim_heading - 1.0) % 360
        elif driving_mode == DrivingMode.HARD_LEFT_FORWARD:
            self.sim_heading = (self.sim_heading + 3.0) % 360
        elif driving_mode == DrivingMode.HARD_RIGHT_FORWARD:
            self.sim_heading = (self.sim_heading - 3.0) % 360

        delta_x = speed * self.interval * math.cos(math.radians(self.sim_heading))
        delta_y = speed * self.interval * math.sin(math.radians(self.sim_heading))
        self.sim_utm_x += delta_x
        self.sim_utm_y += delta_y

    def turn_around(self, angle):
        if self.lining_up_request:
            self.driving_mode_disp = "Lining Up" + f"({angle:.2f} deg)"
        else:
            self.driving_mode_disp = DrivingMode.TURN_AROUND.name + f"({angle:.2f} deg)"

        if abs(angle) < 3:
            delta = 0.0
            self.lining_up_request = False
        elif abs(angle) < 10:
            delta = 1.0
        else:
            delta = 3.0

        delta = delta if angle > 0 else -delta
        self.sim_heading = (self.sim_heading + delta) % 360

    def stop_drive(self):
        self.driving_mode_disp = DrivingMode.STOP.name

    def gps_custom_callback(self, msg):
        pass
        # we don't use actual GPS data in this simulation.
        # self.latitude = msg.latitude
        # self.longitude = msg.longitude
        # self.utm_x, self.utm_y = self.transformer.transform(
        #     self.longitude, self.latitude
        # )
        # self.yaw = -msg.heading % 360  # convert to CCW positive
        # self.gps_quality = msg.gps_quality
        # self.num_sats = msg.num_sat

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = GeoLoc.Feedback()
        result = GeoLoc.Result()

        # initial simulation position is given by the goal request
        mode = OperationMode.START_OVER

        self.sim_utm_y = goal_handle.request.point.latitude
        self.sim_utm_x = goal_handle.request.point.longitude
        self.sim_heading = goal_handle.request.point.altitude

        p_src = (0, 0)
        p_dst = (10, 10)
        p_current = (self.sim_utm_x, self.sim_utm_y)
        current_heading = self.sim_heading
        distance_to_goal = utm_distance(p_current, p_dst)

        # Reset ticks for each execution
        ticks_for_5_secs = int(5.0 / self.interval)
        ticks_for_1_sec = int(1.0 / self.interval)

        rate = self.create_rate(int(1.0 / self.interval))  # Hz
        while rclpy.ok():  # or while True:
            if mode == OperationMode.START_OVER:
                if ticks_for_1_sec % 10 == 0:
                    sec = ticks_for_5_secs // 10
                    self.get_logger().info(f"Starting simulation in {sec} seconds...")
                if ticks_for_5_secs % 50 == 1:
                    self.lining_up_request = True
                    self.debug_str = ""
                    mode = OperationMode.RUNNING
            elif mode == OperationMode.RUNNING:
                p_current = (self.sim_utm_x, self.sim_utm_y)
                current_heading = self.sim_heading
                distance_to_goal = self.driving(
                    p_src, p_dst, p_current, current_heading
                )

                if distance_to_goal < 0.5:  # Reached destination
                    mode = OperationMode.FINISHED
                    self.get_logger().info("Reached destination!")

            elif mode == OperationMode.FINISHED:
                if self.debug:
                    self.get_logger().info(f"\n{self.debug_str}")
                self.get_logger().info("Simulation finished.")
                break

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("test_driving_sim action canceled")
                # self.stop_drive()
                result.success = False
                return result

            feedback.distances = [0.0, 0.0]
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
            if (
                ticks_for_1_sec % 10 == 0
                and self.debug
                and mode == OperationMode.RUNNING
            ):
                self.get_logger().info(f"\n{self.debug_str}")

            try:
                rate.sleep()
            except Exception as _:
                # Handle case where ROS context shuts down
                result.success = False
                return result

        goal_handle.succeed()
        self.get_logger().info("test_driving_sim action completed successfully")
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
        "--debug", action="store_true", help="Enable debug mode (default: False*)"
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
