from pyproj import Transformer
import copy

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleNav

from uwtec_cart.utils import (
    get_config_value,
    check_timeout,
    calc_heading_from_yaw_and_offset,
    calc_goal_heading,
    calc_goal_coordinates,
    rotate_to_go,
    distance_to_go,
)

from uwtec_cart.utils.driving_mixin import DrivingMode, DrivingMixin


class TestRunServer(DrivingMixin, Node):
    def __init__(self):
        super().__init__("test_run_server")
        self.get_logger().info("TestRunServer has been started.")
        self.interval = 0.01

        # Initialize GPS-related attributes - will be updated by GPS callback
        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32652", always_xy=True
        )
        self.latitude = 0.0
        self.longitude = 0.0
        self.yaw = 0.0
        self.gps_quality = 0
        self.num_sats = 0
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )

        self.action_server = ActionServer(
            self,
            SimpleNav,
            "test_run",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat,
            "/gps/custom",
            self.gps_custom_callback,
            1,
        )

        self.twist = Twist()
        self.prev_twist = copy.deepcopy(self.twist)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 1)
        self.rate = self.create_rate(int(1.0 / self.interval))

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.yaw = msg.heading
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats

    def cancel_callback(self, goal_handle):
        self.get_logger().info("test-run cancelled.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing test-run action...")
        # feedback = SimpleNav.Feedback()
        result = SimpleNav.Result()

        # retrieve params given by the goal request
        cmd = goal_handle.request.cmd
        distance = goal_handle.request.distance
        angle = goal_handle.request.angle
        left_speed = goal_handle.request.left_speed
        right_speed = goal_handle.request.right_speed

        # get *_speed variables from config/system.yaml and assign to instance variables
        # these values are consumed in methods of DrivingMixin
        self.linear_speed = get_config_value("linear_speed", default=0.5)
        self.angular_speed = get_config_value("angular_speed", default=0.5)
        gyro_offset = get_config_value("gyro_offset", default=0.0)

        # initialize start and end UTM coordinates
        current_heading = calc_heading_from_yaw_and_offset(self.yaw, gyro_offset)
        goal_heading = calc_goal_heading(current_heading, by=angle)

        start_utm_x, start_utm_y = self.utm_x, self.utm_y
        goal_utm_x, goal_utm_y = calc_goal_coordinates(
            (start_utm_x, start_utm_y), distance, goal_heading
        )

        mode = DrivingMode.READY
        prev_mode = DrivingMode.READY

        ticks = 1
        while rclpy.ok():
            if mode != prev_mode:
                self.get_logger().info(f"{mode}")
                prev_mode = mode

            if mode == DrivingMode.READY:
                if cmd == "forward":
                    mode = DrivingMode.FORWARD
                elif cmd == "turn":
                    mode = DrivingMode.TURN_TEST
                elif cmd == "nav-to":
                    mode = DrivingMode.RUNNING
                elif cmd == "stop":
                    mode = DrivingMode.FINISHED
                elif cmd == "motor":
                    mode = DrivingMode.MOTOR_TEST

            elif mode == DrivingMode.FINISHED:
                self.stop()
                break  # end of while loop

            else:
                if mode == DrivingMode.MOTOR_TEST:
                    self.simple_forward(left_speed=left_speed, right_speed=right_speed)

                # elif mode == DrivingMode.TURN_TEST:
                #     current_heading = calc_heading_from_yaw_and_offset(
                #         self.yaw, gyro_offset
                #     )
                #     rotation_remaining = rotate_to_go(current_heading, goal_heading)
                #     if abs(rotation_remaining) < 3.0:
                #         mode = DrivingMode.FINISHED
                #     else:
                #         self.turn(rotation_remaining)

                else:
                    current_utm_x, current_utm_y = self.utm_x, self.utm_y
                    current_heading = calc_heading_from_yaw_and_offset(
                        self.yaw, gyro_offset
                    )
                    mode = self.go_driving(
                        mode=mode,
                        current_utm=(current_utm_x, current_utm_y),
                        current_heading=current_heading,
                        start_utm=(start_utm_x, start_utm_y),
                        goal_utm=(goal_utm_x, goal_utm_y),
                    )

                # timeout for forward movement: 30 seconds or distance traveled, whichever comes first
                if check_timeout(ticks, 30.0, self.interval):
                    # print("Timeout check: ticks =", ticks)
                    mode = DrivingMode.FINISHED

                if goal_handle.is_cancel_requested:
                    self.stop()
                    self.get_logger().info("test-run cancelled during execution.")
                    goal_handle.canceled()
                    return SimpleNav.Result(success=False)

            # feedback.progress = 0
            # goal_handle.publish_feedback(feedback)

            try:
                ticks += 1
                self.rate.sleep()
            except Exception as e:
                # Handle case where ROS context shuts down
                print(e)
                result.success = False
                return result

        # end of while loop
        self.stop()
        self.get_logger().info("test-run completed.")
        goal_handle.succeed()
        result.success = True
        return result
