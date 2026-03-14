# import os
# import yaml
# from datetime import datetime
# from pyproj import Transformer
# from enum import Enum
# from ament_index_python.packages import get_package_share_directory
#
# import asyncio
#
# import rclpy
# from rclpy.node import Node
# from rclpy.action.server import ActionServer, CancelResponse
#
# from geometry_msgs.msg import Twist
# from uwtec_interfaces.msg import CustomNavSat
# from uwtec_interfaces.action import GeoLoc
# from uwtec_interfaces.action import SimpleCommand
# from uwtec_navigation.utils import (
#     utm_bearing,
#     utm_distance,
#     # signed_angle,
#     calc_offset,
#     calc_heading_by_offset,
#     calc_goal_heading,
#     rotate_to_go,
#     distance_and_bearing_xy,
#     shortest_distance_and_angle_to_line,
# )
#
#
# class OperationMode(Enum):
#     START_OVER = 1
#     RUNNING = 2
#     PAUSED = 3
#     TURN_AROUND = 4
#     FINISHED = 5
#
#
# class DrivingMode(Enum):
#     FORWARD = 1
#     HARD_LEFT_FORWARD = 2
#     HARD_RIGHT_FORWARD = 3
#     MILD_LEFT_FORWARD = 4
#     MILD_RIGHT_FORWARD = 5
#     TURN_AROUND = 6
#     STOP = 7
#

# class ShuttleRunServer(Node):
#     def __init__(self, interval=0.1, angular_speed=0.5, linear_speed=0.5, debug=False):
#         super().__init__("shuttle_run_server_node")
#         self.interval = interval
#         self.angular_speed = angular_speed
#         self.linear_speed = linear_speed
#         self.debug = debug
#         self.debug_str = ""
#         self.lining_up_request = False  # set to True when we need to line up with the goal heading before driving straight to the destination
#         self.driving_mode_disp = DrivingMode.STOP.name
#
#         self.latitude = 36.5665
#         self.longitude = 127.9780
#         self.yaw = 0.0
#         self.gps_quality = 0
#         self.num_sats = 0
#         self.heading = 0.0
#         # self.src_latitude = None
#         # self.src_longitude = None
#         # self.dst_latitude = None
#         # self.dst_longitude = None
#
#         self.transformer = Transformer.from_crs(
#             "EPSG:4326", "EPSG:32652", always_xy=True
#         )
#         self.utm_x, self.utm_y = self.transformer.transform(
#             self.longitude, self.latitude
#         )
#         self.heading_yaml_path = os.path.join(
#             get_package_share_directory("uwtec_navigation"), "config", "heading.yaml"
#         )
#
#         self.action_server = ActionServer(
#             self,
#             GeoLoc,
#             "shuttle_run",
#             self.execute_callback,
#             cancel_callback=self.cancel_callback,
#         )
#
#         self.cmd_vel_nav_pub = self.create_publisher(
#             Twist,
#             "/cmd_vel_nav",
#             # "/cmd_vel",  # 임시로 테스트 할 때
#             1,
#         )
#
#         self.localizer_sub = self.create_subscription(
#             CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
#         )
#
#     def driving(self, p_src, p_dst, p_current, current_heading):
#         distance, goal_heading = distance_and_bearing_xy(p_src, p_dst)
#         distance_from_src, angle_from_src = distance_and_bearing_xy(p_src, p_current)
#         distance_to_dst, angle_to_dst = distance_and_bearing_xy(p_current, p_dst)
#         distance_to_path, angle_to_path = shortest_distance_and_angle_to_line(
#             p_src, p_dst, p_current
#         )
#
#         if self.debug:
#             self.debug_str = (
#                 f"Planned:   {distance:05.2f} m, {goal_heading:05.2f} deg\n"
#                 + f"Travelled: {distance_from_src:05.2f} m, {angle_from_src:05.2f} deg\n"
#                 + f"Remaining: {distance_to_dst:05.2f} m, {angle_to_dst:05.2f} deg\n"
#                 + f"Deviation: {distance_to_path:05.2f} m, {angle_to_path:05.2f} deg\n"
#                 + f"Heading:   {current_heading:05.2f} deg at ({self.utm_x:.2f}, {self.utm_y:.2f})"
#                 + f" => {self.driving_mode_disp}"
#             )
#
#         if distance_to_dst < 0.2:
#             self.stop_drive()
#             return distance_to_dst
#
#         if self.lining_up_request:
#             remaining_angle = rotate_to_go(current_heading, goal_heading)
#             self.turn_around(remaining_angle)
#             return distance_to_dst
#
#         self.lining_up_request = False  # reset lining up request just in case
#         if distance_to_path < 0.2:  # within 20 cm
#             remaining_angle = rotate_to_go(current_heading, goal_heading)
#             # if the angle is too large, we might be going in the wrong direction, so let's turn around first before driving straight to the destination
#             if abs(remaining_angle) > 45.0:
#                 self.turn_around(remaining_angle)
#             # if the angle is moderately large, we can turn while driving
#             elif abs(remaining_angle) > 6.0:
#                 driving_mode = (
#                     DrivingMode.MILD_LEFT_FORWARD
#                     if remaining_angle > 0
#                     else DrivingMode.MILD_RIGHT_FORWARD
#                 )
#                 self.go_forward(driving_mode, distance=distance_to_dst)
#             # if the angle is small, we can just drive straight
#             else:
#                 self.go_forward(DrivingMode.FORWARD, distance=distance_to_dst)
#
#         elif distance_to_path < 1.0:  # within 1 meter
#             remaining_angle = rotate_to_go(current_heading, angle_to_dst)
#             # if the angle is too large, we might be going in the wrong direction, so let's turn around first before driving straight to the destination
#             if abs(remaining_angle) > 45.0:
#                 self.turn_around(remaining_angle)
#             # if the angle is moderately large, we can turn while driving
#             elif abs(remaining_angle) > 10.0:
#                 direction = (
#                     DrivingMode.HARD_LEFT_FORWARD
#                     if remaining_angle > 0
#                     else DrivingMode.HARD_RIGHT_FORWARD
#                 )
#                 self.go_forward(direction, distance=distance_to_dst)
#             else:
#                 self.go_forward(DrivingMode.FORWARD, distance=distance_to_dst)
#
#         else:  # more than 1 meter away from the path
#             remaining_angle = rotate_to_go(current_heading, angle_to_path)
#             if abs(remaining_angle) < 10:  # already facing the line closely enough
#                 self.go_forward(DrivingMode.FORWARD, distance=distance_to_path)
#             else:  # need to turn toward the face the line
#                 self.turn_around(remaining_angle)
#
#         return distance_to_dst
#
#     def go_forward(self, driving_mode, distance=0.0):
#         self.driving_mode_disp = driving_mode.name + f"({distance:.2f} m)"
#         if distance < 1.0:  # slow down when close to the goal
#             linear_speed = self.linear_speed * 0.5
#         else:
#             linear_speed = self.linear_speed
#
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.01
#         twist_msg.linear.y = 0.01
#
#         if driving_mode == DrivingMode.FORWARD:
#             # print("- moving forward")
#             twist_msg.linear.x = linear_speed
#             twist_msg.linear.y = linear_speed
#
#         elif driving_mode == DrivingMode.HARD_LEFT_FORWARD:
#             # print("- moving forward with steep left turn")
#             twist_msg.linear.x = linear_speed
#             twist_msg.linear.y = linear_speed * 1.4
#         elif driving_mode == DrivingMode.HARD_RIGHT_FORWARD:
#             # print("- moving forward with steep right turn")
#             twist_msg.linear.x = linear_speed * 1.4
#             twist_msg.linear.y = linear_speed
#         elif driving_mode == DrivingMode.MILD_LEFT_FORWARD:
#             # print("- moving forward with smooth left turn")
#             twist_msg.linear.x = linear_speed
#             twist_msg.linear.y = linear_speed * 1.2
#         elif driving_mode == DrivingMode.MILD_RIGHT_FORWARD:
#             # print("- moving forward with smooth right turn")
#             twist_msg.linear.x = linear_speed * 1.2
#             twist_msg.linear.y = linear_speed
#         else:
#             twist_msg.linear.x = 0.01
#             twist_msg.linear.y = 0.01
#
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def turn_around(self, angle):
#         if self.lining_up_request:
#             self.driving_mode_disp = "Lining Up" + f"({angle:.2f} deg)"
#         else:
#             self.driving_mode_disp = DrivingMode.TURN_AROUND.name + f"({angle:.2f} deg)"
#
#         if abs(angle) < 3:
#             angular_speed = 0.0  # self.angular_speed * 0.5
#             self.lining_up_request = False
#         elif abs(angle) < 6:
#             angular_speed = self.angular_speed * 0.5
#         else:
#             angular_speed = self.angular_speed
#
#         twist_msg = Twist()
#         twist_msg.linear.x = angular_speed if angle < 0 else 0.01
#         twist_msg.linear.y = angular_speed if angle > 0 else 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def stop_drive(self):
#         self.driving_mode_disp = DrivingMode.STOP.name
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.01
#         twist_msg.linear.y = 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def gps_custom_callback(self, msg):
#         self.latitude = msg.latitude
#         self.longitude = msg.longitude
#         self.utm_x, self.utm_y = self.transformer.transform(
#             self.longitude, self.latitude
#         )
#         self.yaw = -msg.heading % 360  # convert to CCW positive
#         self.gps_quality = msg.gps_quality
#         self.num_sats = msg.num_sats
#
#     def cancel_callback(self, _):
#         self.get_logger().info("Received cancel request")
#         return CancelResponse.ACCEPT
#
#     def execute_callback(self, goal_handle):
#         feedback = GeoLoc.Feedback()
#         result = GeoLoc.Result()
#
#         dst_latitude = goal_handle.request.point.latitude
#         dst_longitude = goal_handle.request.point.longitude
#
#         # initial simulation position is given by the goal request
#         mode = OperationMode.START_OVER
#
#         p_current = (self.utm_x, self.utm_y)
#         p_src = None
#         p_dst = None
#         distance_to_goal = utm_distance(p_current, p_current)
#         runs = 0
#         num_runs = 4
#
#         with open(self.heading_yaml_path, "r") as heading_file:
#             data = yaml.safe_load(heading_file)
#             self.offset = data.get("offset", 0.0)
#             # print("Offset: ", self.offset)
#             # print("Yaw: ", self.yaw)
#         current_heading = calc_heading_by_offset(self.yaw, self.offset)
#
#         # if self.dst_latitude is not None:
#         #     self.src_latitude = self.dst_latitude
#         #     self.src_longitude = self.dst_longitude
#         # else:
#         #     self.src_latitude = self.latitude
#         #     self.src_longitude = self.longitude
#         #
#         # p_src = self.transformer.transform(self.src_longitude, self.src_latitude)
#         # p_dst = self.transformer.transform(self.dst_longitude, self.dst_latitude)
#         # p_current = (self.utm_x, self.utm_y)
#
#         # Reset ticks for each execution
#         ticks_for_5_secs = int(5.0 / self.interval)
#         ticks_for_1_sec = int(1.0 / self.interval)
#         rate = self.create_rate(int(1.0 / self.interval))  # Hz
#         while rclpy.ok():  # or while True:
#             if mode == OperationMode.START_OVER:
#                 if ticks_for_1_sec % 10 == 0:
#                     sec = ticks_for_5_secs // 10
#                     self.get_logger().info(f"Starting driving in {sec} seconds...")
#                 if ticks_for_5_secs % 50 == 1:
#                     if p_dst is None:
#                         p_dst = self.transformer.transform(dst_longitude, dst_latitude)
#                     else:
#                         p_dst = p_src
#                     p_src = p_current
#
#                     self.debug_str = ""
#                     mode = OperationMode.RUNNING
#                     self.lining_up_request = True
#                     ticks_for_5_secs = int(5.0 / self.interval)
#             elif mode == OperationMode.RUNNING:
#                 p_current = (self.utm_x, self.utm_y)
#                 current_heading = calc_heading_by_offset(self.yaw, self.offset)
#                 distance_to_goal = self.driving(
#                     p_src, p_dst, p_current, current_heading
#                 )
#
#                 if distance_to_goal < 0.5:  # Reached destination
#                     mode = OperationMode.FINISHED
#                     self.get_logger().info("Reached destination!")
#
#             elif mode == OperationMode.FINISHED:
#                 self.stop_drive()
#                 self.get_logger().info("Driving finished.")
#                 if self.debug:
#                     self.get_logger().info(f"\n{self.debug_str}")
#                 runs += 1
#                 if runs == num_runs:
#                     break
#                 else:
#                     mode = OperationMode.START_OVER
#
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 self.get_logger().info("shuttle_run action canceled")
#                 self.stop_drive()
#                 result.success = False
#                 return result
#
#             feedback.distances = [0.0, 0.0]
#             goal_handle.publish_feedback(feedback)
#
#             ## tick updates
#             ticks_for_5_secs = (
#                 ticks_for_5_secs - 1
#                 if ticks_for_5_secs > 1
#                 else int(5.0 / self.interval)
#             )
#             ticks_for_1_sec = (
#                 ticks_for_1_sec - 1
#                 if ticks_for_1_sec > 1.0
#                 else int(1.0 / self.interval)
#             )
#             if (
#                 ticks_for_1_sec % 10 == 0
#                 and self.debug
#                 and mode == OperationMode.RUNNING
#             ):
#                 self.get_logger().info(f"\n{self.debug_str}")
#
#             try:
#                 rate.sleep()
#             except Exception as _:
#                 # Handle case where ROS context shuts down
#                 result.success = False
#                 return result
#
#         goal_handle.succeed()
#         self.stop_drive()
#         self.get_logger().info("shuttle_run action completed successfully")
#         result.success = True
#         return result


# class NavToWpsServer(Node):
#     def __init__(
#         self,
#         interval=0.1,
#         angular_speed=0.5,
#         linear_speed=0.5,
#         wps_file="wps.yaml",
#         debug=False,
#     ):
#         super().__init__("nav_to_wps_server_node")
#         self.interval = interval
#         self.angular_speed = angular_speed
#         self.linear_speed = linear_speed
#         self.debug = debug
#         self.debug_str = ""
#         self.lining_up_request = False  # set to True when we need to line up with the goal heading before driving straight to the destination
#         self.driving_mode_disp = DrivingMode.STOP.name
#
#         self.latitude = 36.5665
#         self.longitude = 127.9780
#         self.yaw = 0.0
#         self.gps_quality = 0
#         self.num_sats = 0
#         # self.heading = 0.0
#
#         self.transformer = Transformer.from_crs(
#             "EPSG:4326", "EPSG:32652", always_xy=True
#         )
#         self.utm_x, self.utm_y = self.transformer.transform(
#             self.longitude, self.latitude
#         )
#         self.heading_yaml_path = os.path.join(
#             get_package_share_directory("uwtec_navigation"), "config", "heading.yaml"
#         )
#
#         wps_yaml_path = os.path.join(
#             get_package_share_directory("uwtec_navigation"), "config", wps_file
#         )
#         # wps_yaml_path = os.path.join(os.path.dirname(__file__), wps_file)
#         self.coords = []
#         with open(wps_yaml_path, "r") as wps_file:
#             self.coords = yaml.safe_load(wps_file)
#         print(self.coords)
#         self.wps_index = 0
#
#         self.action_server = ActionServer(
#             self,
#             SimpleCommand,
#             "nav_to_wps",
#             self.execute_callback,
#             cancel_callback=self.cancel_callback,
#         )
#
#         self.cmd_vel_nav_pub = self.create_publisher(
#             Twist,
#             "/cmd_vel_nav",
#             # "/cmd_vel",  # 임시로 테스트 할 때
#             1,
#         )
#
#         self.localizer_sub = self.create_subscription(
#             CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
#         )
#
#     def driving(self, p_src, p_dst, p_current, current_heading):
#         distance, goal_heading = distance_and_bearing_xy(p_src, p_dst)
#         distance_from_src, angle_from_src = distance_and_bearing_xy(p_src, p_current)
#         distance_to_dst, angle_to_dst = distance_and_bearing_xy(p_current, p_dst)
#         distance_to_path, angle_to_path = shortest_distance_and_angle_to_line(
#             p_src, p_dst, p_current
#         )
#
#         if self.debug:
#             self.debug_str = (
#                 f"Planned:   {distance:05.2f} m, {goal_heading:05.2f} deg\n"
#                 + f"Travelled: {distance_from_src:05.2f} m, {angle_from_src:05.2f} deg\n"
#                 + f"Remaining: {distance_to_dst:05.2f} m, {angle_to_dst:05.2f} deg\n"
#                 + f"Deviation: {distance_to_path:05.2f} m, {angle_to_path:05.2f} deg\n"
#                 + f"Heading:   {current_heading:05.2f} deg at ({self.utm_x:.2f}, {self.utm_y:.2f})"
#                 + f" => {self.driving_mode_disp}"
#             )
#
#         if distance_to_dst < 0.2:
#             self.stop_drive()
#             return distance_to_dst
#
#         if self.lining_up_request:
#             remaining_angle = rotate_to_go(current_heading, goal_heading)
#             self.turn_around(remaining_angle)
#             return distance_to_dst
#
#         self.lining_up_request = False  # reset lining up request just in case
#         if distance_to_path < 0.2:  # within 20 cm
#             remaining_angle = rotate_to_go(current_heading, goal_heading)
#             # if the angle is too large, we might be going in the wrong direction, so let's turn around first before driving straight to the destination
#             if abs(remaining_angle) > 45.0:
#                 self.turn_around(remaining_angle)
#             # if the angle is moderately large, we can turn while driving
#             elif abs(remaining_angle) > 6.0:
#                 driving_mode = (
#                     DrivingMode.MILD_LEFT_FORWARD
#                     if remaining_angle > 0
#                     else DrivingMode.MILD_RIGHT_FORWARD
#                 )
#                 self.go_forward(driving_mode, distance=distance_to_dst)
#             # if the angle is small, we can just drive straight
#             else:
#                 self.go_forward(DrivingMode.FORWARD, distance=distance_to_dst)
#
#         elif distance_to_path < 1.0:  # within 1 meter
#             remaining_angle = rotate_to_go(current_heading, angle_to_dst)
#             # if the angle is too large, we might be going in the wrong direction, so let's turn around first before driving straight to the destination
#             if abs(remaining_angle) > 45.0:
#                 self.turn_around(remaining_angle)
#             # if the angle is moderately large, we can turn while driving
#             elif abs(remaining_angle) > 10.0:
#                 direction = (
#                     DrivingMode.HARD_LEFT_FORWARD
#                     if remaining_angle > 0
#                     else DrivingMode.HARD_RIGHT_FORWARD
#                 )
#                 self.go_forward(direction, distance=distance_to_dst)
#             else:
#                 self.go_forward(DrivingMode.FORWARD, distance=distance_to_dst)
#
#         else:  # more than 1 meter away from the path
#             remaining_angle = rotate_to_go(current_heading, angle_to_path)
#             if abs(remaining_angle) < 10:  # already facing the line closely enough
#                 self.go_forward(DrivingMode.FORWARD, distance=distance_to_path)
#             else:  # need to turn toward the face the line
#                 self.turn_around(remaining_angle)
#
#         return distance_to_dst
#
#     def go_forward(self, driving_mode, distance=0.0):
#         self.driving_mode_disp = driving_mode.name + f"({distance:.2f} m)"
#         if distance < 1.0:  # slow down when close to the goal
#             linear_speed = self.linear_speed * 0.5
#         else:
#             linear_speed = self.linear_speed
#
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.01
#         twist_msg.linear.y = 0.01
#
#         if driving_mode == DrivingMode.FORWARD:
#             # print("- moving forward")
#             twist_msg.linear.x = linear_speed
#             twist_msg.linear.y = linear_speed
#
#         elif driving_mode == DrivingMode.HARD_LEFT_FORWARD:
#             # print("- moving forward with steep left turn")
#             twist_msg.linear.x = linear_speed
#             twist_msg.linear.y = linear_speed * 1.4
#         elif driving_mode == DrivingMode.HARD_RIGHT_FORWARD:
#             # print("- moving forward with steep right turn")
#             twist_msg.linear.x = linear_speed * 1.4
#             twist_msg.linear.y = linear_speed
#         elif driving_mode == DrivingMode.MILD_LEFT_FORWARD:
#             # print("- moving forward with smooth left turn")
#             twist_msg.linear.x = linear_speed
#             twist_msg.linear.y = linear_speed * 1.2
#         elif driving_mode == DrivingMode.MILD_RIGHT_FORWARD:
#             # print("- moving forward with smooth right turn")
#             twist_msg.linear.x = linear_speed * 1.2
#             twist_msg.linear.y = linear_speed
#         else:
#             twist_msg.linear.x = 0.01
#             twist_msg.linear.y = 0.01
#
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def turn_around(self, angle):
#         if self.lining_up_request:
#             self.driving_mode_disp = "Lining Up" + f"({angle:.2f} deg)"
#         else:
#             self.driving_mode_disp = DrivingMode.TURN_AROUND.name + f"({angle:.2f} deg)"
#
#         if abs(angle) < 3:
#             angular_speed = 0.0  # self.angular_speed * 0.5
#             self.lining_up_request = False
#         elif abs(angle) < 6:
#             angular_speed = self.angular_speed * 0.5
#         else:
#             angular_speed = self.angular_speed
#
#         twist_msg = Twist()
#         twist_msg.linear.x = angular_speed if angle < 0 else 0.01
#         twist_msg.linear.y = angular_speed if angle > 0 else 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def stop_drive(self):
#         self.driving_mode_disp = DrivingMode.STOP.name
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.01
#         twist_msg.linear.y = 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def gps_custom_callback(self, msg):
#         self.latitude = msg.latitude
#         self.longitude = msg.longitude
#         self.utm_x, self.utm_y = self.transformer.transform(
#             self.longitude, self.latitude
#         )
#         self.yaw = -msg.heading % 360  # convert to CCW positive
#         self.gps_quality = msg.gps_quality
#         self.num_sats = msg.num_sats
#
#     def cancel_callback(self, _):
#         self.get_logger().info("Received cancel request")
#         return CancelResponse.ACCEPT
#
#     def execute_callback(self, goal_handle):
#         feedback = SimpleCommand.Feedback()
#         result = SimpleCommand.Result()
#
#         # initial simulation position is given by the goal request
#         mode = OperationMode.START_OVER
#
#         p_current = (self.utm_x, self.utm_y)
#         distance_to_goal = utm_distance(p_current, p_current)
#
#         with open(self.heading_yaml_path, "r") as heading_file:
#             data = yaml.safe_load(heading_file)
#             self.offset = data.get("offset", 0.0)
#             # print("Offset: ", self.offset)
#             # print("Yaw: ", self.yaw)
#         current_heading = calc_heading_by_offset(self.yaw, self.offset)
#
#         # Reset ticks for each execution
#         ticks_for_5_secs = int(5.0 / self.interval)
#         ticks_for_1_sec = int(1.0 / self.interval)
#         rate = self.create_rate(int(1.0 / self.interval))  # Hz
#         while rclpy.ok():  # or while True:
#             if mode == OperationMode.START_OVER:
#                 p_current = (self.utm_x, self.utm_y)
#                 if ticks_for_1_sec % 10 == 0:
#                     sec = ticks_for_5_secs // 10
#                     self.get_logger().info(f"Starting driving in {sec} seconds...")
#                 if ticks_for_5_secs % 50 == 1:
#                     self.debug_str = ""
#                     mode = OperationMode.RUNNING
#                     self.lining_up_request = True
#                     p_src = p_current
#                     p_dst = self.transformer.transform(
#                         self.coords[self.wps_index]["longitude"],
#                         self.coords[self.wps_index]["latitude"],
#                     )
#                     ticks_for_5_secs = int(5.0 / self.interval)
#             elif mode == OperationMode.RUNNING:
#                 p_current = (self.utm_x, self.utm_y)
#                 current_heading = calc_heading_by_offset(self.yaw, self.offset)
#                 distance_to_goal = self.driving(
#                     p_src, p_dst, p_current, current_heading
#                 )
#
#                 if distance_to_goal < 0.5:  # Reached destination
#                     mode = OperationMode.FINISHED
#                     self.get_logger().info("Reached destination!")
#
#             elif mode == OperationMode.FINISHED:
#                 self.stop_drive()
#                 self.get_logger().info("Driving finished.")
#                 if self.debug:
#                     self.get_logger().info(f"\n{self.debug_str}")
#                 self.wps_index += 1
#                 if self.wps_index == len(self.coords):
#                     break
#                 else:
#                     mode = OperationMode.START_OVER
#
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 self.get_logger().info("nav_to_wps action canceled")
#                 self.stop_drive()
#                 result.success = False
#                 return result
#
#             feedback.progress = 0
#             goal_handle.publish_feedback(feedback)
#
#             ## tick updates
#             ticks_for_5_secs = (
#                 ticks_for_5_secs - 1
#                 if ticks_for_5_secs > 1
#                 else int(5.0 / self.interval)
#             )
#             ticks_for_1_sec = (
#                 ticks_for_1_sec - 1
#                 if ticks_for_1_sec > 1.0
#                 else int(1.0 / self.interval)
#             )
#             if (
#                 ticks_for_1_sec % 10 == 0
#                 and self.debug
#                 and mode == OperationMode.RUNNING
#             ):
#                 self.get_logger().info(f"\n{self.debug_str}")
#
#             try:
#                 rate.sleep()
#             except Exception as _:
#                 # Handle case where ROS context shuts down
#                 result.success = False
#                 return result
#
#         goal_handle.succeed()
#         self.stop_drive()
#         self.get_logger().info("nav_to_wps action completed successfully")
#         result.success = True
#         return result


# class HeadingAndOffsetServer(Node):
#     def __init__(self, interval=0.1, angular_speed=0.5, linear_speed=0.5, debug=False):
#         super().__init__("heading_and_offset_server_node")
#         self.interval = interval
#         self.angular_speed = angular_speed
#         self.linear_speed = linear_speed
#         self.debug = debug
#
#         self.latitude = 36.5665
#         self.longitude = 127.9780
#         self.yaw = 0.0
#         self.gps_quality = 0
#         self.num_sats = 0
#
#         self.transformer = Transformer.from_crs(
#             "EPSG:4326", "EPSG:32652", always_xy=True
#         )
#         self.utm_x, self.utm_y = self.transformer.transform(
#             self.longitude, self.latitude
#         )
#         self.heading_yaml_path = os.path.join(
#             get_package_share_directory("uwtec_navigation"), "config", "heading.yaml"
#         )
#
#         self.action_server = ActionServer(
#             self,
#             SimpleCommand,
#             "heading_and_offset",
#             self.execute_callback,
#             cancel_callback=self.cancel_callback,
#         )
#
#         self.cmd_vel_nav_pub = self.create_publisher(
#             Twist,
#             "/cmd_vel_nav",
#             # "/cmd_vel",  # 임시로 테스트 할 때
#             1,
#         )
#
#         self.localizer_sub = self.create_subscription(
#             CustomNavSat, "/gps/custom", self.gps_custom_callback, 1
#         )
#
#     def go_forward(self):
#         twist_msg = Twist()
#         twist_msg.linear.x = self.linear_speed
#         twist_msg.linear.y = self.linear_speed
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def stop_drive(self):
#         twist_msg = Twist()
#         twist_msg.linear.x = 0.01
#         twist_msg.linear.y = 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def turn_around(self, angle):
#         twist_msg = Twist()
#         if angle > 0:  # ccw turn, turn left
#             twist_msg.linear.x = 0.01
#             twist_msg.linear.y = self.angular_speed
#         else:  # cw turn, turn right
#             twist_msg.linear.x = self.angular_speed
#             twist_msg.linear.y = 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def turn_right(self):
#         twist_msg = Twist()
#         twist_msg.linear.x = self.angular_speed
#         twist_msg.linear.y = 0.01
#         self.cmd_vel_nav_pub.publish(twist_msg)
#
#     def gps_custom_callback(self, msg):
#         self.latitude = msg.latitude
#         self.longitude = msg.longitude
#         self.utm_x, self.utm_y = self.transformer.transform(
#             self.longitude, self.latitude
#         )
#         self.yaw = -msg.heading % 360  # convert to CCW positive
#         self.gps_quality = msg.gps_quality
#         self.num_sats = msg.num_sats
#         # self.heading = east_facing_ccw_angle(self.heading)
#
#     def cancel_callback(self, _):
#         self.get_logger().info("Received cancel request")
#         return CancelResponse.ACCEPT
#
#     def execute_callback(self, goal_handle):
#         feedback = SimpleCommand.Feedback()
#         result = SimpleCommand.Result()
#
#         # initialize local variables
#         mode = OperationMode.START_OVER
#         start_utm_x, start_utm_y = 0.0, 0.0
#         end_utm_x, end_utm_y = 0.0, 0.0
#         offset = 0.0
#         goal_heading = 0.0
#         offsets = []
#         runs = 0
#         num_runs = 4
#
#         # Reset ticks for each execution
#         ticks_for_5_secs = int(5.0 / self.interval)
#         ticks_for_1_sec = int(1.0 / self.interval)
#
#         rate = self.create_rate(int(1.0 / self.interval))  # Hz
#         while rclpy.ok():  # or while True:
#             if mode == OperationMode.START_OVER:
#                 if ticks_for_1_sec % 10 == 0:
#                     disp_sec = ticks_for_5_secs // 10
#                     self.get_logger().info(f"Starting in {disp_sec} seconds...")
#                 if ticks_for_5_secs % 50 == 1:
#                     runs += 1
#                     # save the current position as the starting point
#                     start_utm_x, start_utm_y = self.utm_x, self.utm_y
#                     # start moving forward at the specified linear speed
#                     self.go_forward()
#                     # switch to running mode and reset 5 second timer
#                     mode = OperationMode.RUNNING
#                     ticks_for_5_secs = int(5.0 / self.interval)
#             elif mode == OperationMode.RUNNING:
#                 # check if 5 seconds have passed
#                 if ticks_for_5_secs % 50 == 1:
#                     # stop the robot
#                     self.stop_drive()
#                     # switch to paused mode
#                     mode = OperationMode.PAUSED
#             elif mode == OperationMode.PAUSED:
#                 # save the current position as the end point
#                 end_utm_x, end_utm_y = self.utm_x, self.utm_y
#                 # calculate the bearing between the start and end points
#                 bearing = utm_bearing(
#                     (start_utm_x, start_utm_y), (end_utm_x, end_utm_y)
#                 )
#                 # calculate the offset angle based on the bearing
#                 offset = calc_offset(bearing, self.yaw)
#                 heading = calc_heading_by_offset(self.yaw, offset)
#                 offsets.append(offset)
#                 if runs == num_runs:
#                     mode = OperationMode.FINISHED
#                 else:
#                     # switch to turn around mode
#                     mode = OperationMode.TURN_AROUND
#                     goal_heading = calc_goal_heading(heading, by=90)
#                     ticks_for_5_secs = int(5.0 / self.interval)
#             elif mode == OperationMode.TURN_AROUND:
#                 # find the goal angle to turn 90 degrees to the left
#                 # calculate the shortest angle to turn
#                 current_heading = calc_heading_by_offset(self.yaw, offset)
#                 remaining_angle = rotate_to_go(current_heading, goal_heading)
#                 if (
#                     abs(remaining_angle) < 3 or ticks_for_5_secs % 50 == 1
#                 ):  # within 3 degrees of the goal
#                     # stop the robot
#                     self.stop_drive()
#                     # switch to start-over mode
#                     mode = OperationMode.START_OVER
#                 else:
#                     # keep turning towards the goal
#                     self.turn_around(remaining_angle)
#             elif mode == OperationMode.FINISHED:
#                 # action is complete, break the loop
#                 self.offset = sum(offsets) / len(offsets) if offsets else 0.0
#                 self.get_logger().info(
#                     f"Average offset after {runs} runs: {self.offset:.2f} degrees"
#                 )
#                 with open(self.heading_yaml_path, "w") as heading_file:
#                     yaml.dump({"offset": self.offset}, heading_file, sort_keys=False)
#                 break
#
#             if goal_handle.is_cancel_requested:
#                 goal_handle.canceled()
#                 self.stop_drive()
#                 self.get_logger().info("heading_and_offset action canceled")
#                 result.success = False
#                 return result
#
#             feedback.progress = 0  # Update progress
#             goal_handle.publish_feedback(feedback)
#
#             ## tick updates
#             ticks_for_5_secs = (
#                 ticks_for_5_secs - 1
#                 if ticks_for_5_secs > 1
#                 else int(5.0 / self.interval)
#             )
#             ticks_for_1_sec = (
#                 ticks_for_1_sec - 1
#                 if ticks_for_1_sec > 1.0
#                 else int(1.0 / self.interval)
#             )
#             if ticks_for_1_sec % 10 == 0 and self.debug and mode != OperationMode.START_OVER:
#                 # current_time = datetime.now().strftime("%H:%M:%S")
#                 # self.get_logger().info(current_time)
#                 self.get_logger().info(
#                     f"{runs}/{num_runs}, {mode.name}"
#                 )
#
#             try:
#                 rate.sleep()
#             except Exception as _:
#                 # Handle case where ROS context shuts down
#                 result.success = False
#                 return result
#
#         goal_handle.succeed()
#         self.get_logger().info("heading_and_offset action completed successfully")
#         result.success = True
#         return result
