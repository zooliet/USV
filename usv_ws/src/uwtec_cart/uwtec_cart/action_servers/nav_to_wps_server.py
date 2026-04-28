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
    check_timeout,
    calc_heading_from_yaw_and_offset,
    get_waypoints_from_route_file,
)

from uwtec_cart.utils.driving_mixin import OperationMode, DrivingMode, DrivingMixin


class NavToWpsServer(DrivingMixin, Node):
    def __init__(self):
        super().__init__("nav_to_wps_server")
        self.get_logger().info("NavToWpsServer has been started.")
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
            SimpleCommand,
            "nav_to_wps",
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
        self.get_logger().info("nav-to-wps cancelled.")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing nav-to-wps action...")
        # feedback = SimpleCommand.Feedback()
        result = SimpleCommand.Result()

        # retrieve params given by the goal request
        route_file_name = goal_handle.request.cmd

        # read route file and extract waypoints
        try:
            waypoints = get_waypoints_from_route_file(route_file_name)
            wps_index = 0
        except Exception as e:
            self.get_logger().error(f"Failed to read route file: {e}")
            result.success = False
            return result

        # get *_speed variables from config/system.yaml and assign to instance variables
        # these values are consumed in methods of DrivingMixin
        self.linear_speed = get_config_value("linear_speed", default=0.5)
        self.angular_speed = get_config_value("angular_speed", default=0.5)
        gyro_offset = get_config_value("gyro_offset", default=0.0)
        current_heading = calc_heading_from_yaw_and_offset(self.yaw, gyro_offset)

        src_utm_x, src_utm_y = self.utm_x, self.utm_y
        dst_utm_x, dst_utm_y = None, None

        mode = OperationMode.START_OVER

        ticks = 1
        rate = self.create_rate(int(1.0 / self.interval))
        while rclpy.ok():
            if mode == OperationMode.START_OVER:
                dst_utm_x, dst_utm_y = self.transformer.transform(
                    waypoints[wps_index].get("longitude", 0.0),
                    waypoints[wps_index].get("latitude", 0.0),
                )
                src_utm_x, src_utm_y = self.utm_x, self.utm_y
                mode = OperationMode.RUNNING
                self.driving_mode = DrivingMode.READY

            elif mode == OperationMode.RUNNING:
                current_utm_x, current_utm_y = self.utm_x, self.utm_y
                current_heading = calc_heading_from_yaw_and_offset(
                    self.yaw, gyro_offset
                )
                distance_remaining = self.go_driving(
                    (src_utm_x, src_utm_y),
                    (dst_utm_x, dst_utm_y),
                    (current_utm_x, current_utm_y),
                    current_heading,
                )

                # we give 30.0 seconds for the nav_to_wps to complete each way, but it can be stopped earlier if it reaches the destination
                if (
                    check_timeout(ticks, 30.0, self.interval)
                    or distance_remaining < 0.2
                ):  # 20 cm tolerance
                    self.stop()
                    wps_index += 1
                    self.get_logger().info(
                        f"{wps_index}/{len(waypoints)}: Reached destination."
                    )
                    if wps_index > len(waypoints) - 1:
                        mode = OperationMode.FINISHED
                    else:
                        mode = OperationMode.START_OVER

            elif mode == OperationMode.FINISHED:
                self.stop()
                self.get_logger().info("Driving finished.")
                break

            if goal_handle.is_cancel_requested:
                self.stop()
                self.get_logger().info("nav-to-wps cancelled during execution.")
                goal_handle.canceled()
                return SimpleCommand.Result(success=False)

            # feedback.distance = [0.0, 0.0]  # TODO: calculate distance traveled and remaining
            # goal_handle.publish_feedback(feedback)

            try:
                ticks += 1
                rate.sleep()
            except Exception as e:
                # Handle case where ROS context shuts down
                print(e)
                result.success = False
                return result

        self.stop()
        self.get_logger().info("nav-to-wps completed.")
        goal_handle.succeed()
        result.success = True
        return result
