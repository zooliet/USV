from pyproj import Transformer
import copy

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand

from uwtec_cart.utils import (
    get_config_value,
    set_config_value,
    check_timeout,
    utm_bearing,
    calc_offset,
)

from uwtec_cart.utils.driving_mixin import DrivingMode, DrivingMixin


class CalibrateGyroServer(DrivingMixin, Node):
    def __init__(self):
        super().__init__("calibrate_gyro_server")
        self.get_logger().info("CalibrateGyroServer has been started.")
        self.interval = 0.1

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
            SimpleCommand,
            "calibrate_gyro",
            self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

        self.twist = Twist()
        self.prev_twist = copy.deepcopy(self.twist)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_nav", 1)
        # self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 1) # for testing
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
        self.get_logger().info("calibrate-gyro cancelled.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing calibrate-gyro action...")
        # feedback = SimpleCommand.Feedback()
        result = SimpleCommand.Result()

        # retrieve params given by the goal request
        _ = goal_handle.request.cmd

        # get *_speed variables from config/system.yaml and assign to instance variables
        # these values are consumed in methods of DrivingMixin
        self.linear_speed = get_config_value("linear_speed", default=0.5)
        self.angular_speed = get_config_value("angular_speed", default=0.5)
        gyro_offset = get_config_value("gyro_offset", default=0.0)

        # initialize start and end UTM coordinates for bearing calculation
        start_utm_x, start_utm_y = self.utm_x, self.utm_y
        end_utm_x, end_utm_y = self.utm_x, self.utm_y

        mode = DrivingMode.READY

        ticks = 1
        while rclpy.ok():
            if mode == DrivingMode.READY:
                start_utm_x, start_utm_y = self.utm_x, self.utm_y
                mode = DrivingMode.CALIBRATING

            elif mode == DrivingMode.CALIBRATING:
                if check_timeout(ticks, 5.0, self.interval):
                    mode = DrivingMode.FINISHED
                else:
                    self.simple_forward()

            elif mode == DrivingMode.FINISHED:
                self.stop()
                end_utm_x, end_utm_y = self.utm_x, self.utm_y
                bearing = utm_bearing(
                    (start_utm_x, start_utm_y), (end_utm_x, end_utm_y)
                )
                gyro_offset = calc_offset(bearing, self.yaw)
                set_config_value("gyro_offset", gyro_offset)
                break  # exit while loop after finishing calibration

            if goal_handle.is_cancel_requested:
                self.stop()
                self.get_logger().info("calibrate-gyro cancelled during execution.")
                goal_handle.canceled()
                return SimpleCommand.Result(success=False)

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

        # end of while loop: calibration finished successfully
        self.stop()
        self.get_logger().info("calibrate-gyro completed.")
        goal_handle.succeed()
        result.success = True
        return result
