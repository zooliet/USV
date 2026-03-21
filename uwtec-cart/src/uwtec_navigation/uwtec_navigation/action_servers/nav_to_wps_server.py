from pyproj import Transformer

import rclpy
from rclpy.node import Node
from rclpy.action.server import ActionServer, CancelResponse

from geometry_msgs.msg import Twist
from uwtec_interfaces.msg import CustomNavSat
from uwtec_interfaces.action import SimpleCommand
from uwtec_navigation.utils import (
    utm_distance,
    calc_heading_by_offset,
    get_waypoints_from_yaml,
    get_navigation_config,
    # get_gyro_offset,
    update_ticks,
)

from uwtec_navigation.action_servers.driving_mixin import (
    OperationMode,
    DrivingMode,
    DrivingMixin,
)


class NavToWpsServer(DrivingMixin, Node):
    def __init__(
        self,
        interval=0.1,
        debug=False,
    ):
        super().__init__("nav_to_wps_server_node")
        self.get_logger().info("nav_to_wps_server_node has been started.")
        self.interval = interval
        self.angular_speed = 0.0
        self.linear_speed = 0.0
        self.debug = debug

        self.debug_str = ""
        self.driving_mode_disp = DrivingMode.STOP.name
        self.lining_up_request = False  # set to True when we need to line up with the goal heading before driving straight to the destination

        # dummy initial position (will be updated by GPS callback)
        self.transformer = Transformer.from_crs(
            "EPSG:4326", "EPSG:32652", always_xy=True
        )
        self.latitude = 36.5665
        self.longitude = 127.9780
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

        self.cmd_vel_nav_pub = self.create_publisher(
            Twist,
            "/cmd_vel_nav",
            # "/cmd_vel",  # 임시로 테스트 할 때
            1,
        )

        self.localizer_sub = self.create_subscription(
            CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
        )

    def gps_custom_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.utm_x, self.utm_y = self.transformer.transform(
            self.longitude, self.latitude
        )
        self.yaw = -msg.heading % 360  # convert to CCW positive
        self.gps_quality = msg.gps_quality
        self.num_sats = msg.num_sats

    def cancel_callback(self, _):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback = SimpleCommand.Feedback()
        result = SimpleCommand.Result()

        # read speed settings from the config file
        self.linear_speed = get_navigation_config("linear_speed", default=0.5)
        self.angular_speed = get_navigation_config("angular_speed", default=0.5)

        # initialize local variables
        wps_coords, wps_index = get_waypoints_from_yaml(goal_handle.request.cmd)
        # gyro_offset = get_gyro_offset("heading.yaml")
        gyro_offset = get_navigation_config("gyro_offset", default=0.0)
        current_heading = calc_heading_by_offset(self.yaw, gyro_offset)

        p_current = (self.utm_x, self.utm_y)
        p_src = p_dst = (
            p_current  # will be updated in the loop, but initialize to current position to avoid uninitialized variable error
        )
        distance_to_goal = utm_distance(p_current, p_current)
        mode = OperationMode.START_OVER

        # Reset ticks for each execution
        ticks_for_5_secs = int(5.0 / self.interval)
        ticks_for_1_sec = int(1.0 / self.interval)
        rate = self.create_rate(int(1.0 / self.interval))  # Hz
        while rclpy.ok():  # or while True:
            if mode == OperationMode.START_OVER:
                p_current = (self.utm_x, self.utm_y)
                if ticks_for_1_sec % 10 == 0:
                    sec = ticks_for_5_secs // 10
                    self.get_logger().info(f"Starting driving in {sec} seconds...")
                if ticks_for_5_secs % 50 == 1:
                    self.debug_str = ""
                    mode = OperationMode.RUNNING
                    self.lining_up_request = True
                    p_src = p_current
                    p_dst = self.transformer.transform(
                        wps_coords[wps_index]["longitude"],
                        wps_coords[wps_index]["latitude"],
                    )
                    ticks_for_5_secs = int(5.0 / self.interval)
            elif mode == OperationMode.RUNNING:
                p_current = (self.utm_x, self.utm_y)
                current_heading = calc_heading_by_offset(self.yaw, gyro_offset)
                distance_to_goal = self.driving(
                    p_src, p_dst, p_current, current_heading
                )

                if distance_to_goal < 0.5:  # Reached destination
                    mode = OperationMode.FINISHED
                    self.get_logger().info("Reached destination!")

            elif mode == OperationMode.FINISHED:
                self.stop_drive()
                self.get_logger().info("Driving finished.")
                if self.debug:
                    self.get_logger().info(f"\n{self.debug_str}")
                wps_index += 1
                if wps_index == len(wps_coords):
                    break
                else:
                    mode = OperationMode.START_OVER

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("nav_to_wps action canceled")
                self.stop_drive()
                result.success = False
                return result

            feedback.progress = 0
            goal_handle.publish_feedback(feedback)

            # tick updates
            ticks_for_5_secs, ticks_for_1_sec = update_ticks(
                ticks_for_5_secs, ticks_for_1_sec, interval=self.interval
            )

            # print debug info every 1 second
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
        self.stop_drive()
        self.get_logger().info("nav_to_wps action completed successfully")
        result.success = True
        return result
