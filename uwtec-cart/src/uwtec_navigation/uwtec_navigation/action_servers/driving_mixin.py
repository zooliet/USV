from enum import Enum
from geometry_msgs.msg import Twist
from uwtec_navigation.utils import (
    rotate_to_go,
    distance_and_bearing_xy,
    shortest_distance_and_angle_to_line,
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


class DrivingMixin:
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.debug = False
        self.angular_speed = 0.0
        self.linear_speed = 0.0
        self.lining_up_request = False
        self.driving_mode_disp = "N/A"
        self.cmd_vel_nav_pub = None

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
                + f"Heading:   {current_heading:05.2f} deg at ({p_current[0]:.2f}, {p_current[1]:.2f})"
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
        self.driving_mode_disp = driving_mode.name + f"({distance:.2f} m)"
        if distance < 0.5:
            linear_speed = 0.2
        elif distance < 1.0:
            linear_speed = self.linear_speed * 0.3
        elif distance < 3.0:
            linear_speed = self.linear_speed * 0.5
        else:
            linear_speed = self.linear_speed

        twist_msg = Twist()
        twist_msg.linear.x = 0.01
        twist_msg.linear.y = 0.01

        if driving_mode == DrivingMode.FORWARD:
            # print("- moving forward")
            twist_msg.linear.x = linear_speed
            twist_msg.linear.y = linear_speed

        elif driving_mode == DrivingMode.HARD_LEFT_FORWARD:
            # print("- moving forward with steep left turn")
            twist_msg.linear.x = linear_speed * 0.7
            twist_msg.linear.y = linear_speed
        elif driving_mode == DrivingMode.HARD_RIGHT_FORWARD:
            # print("- moving forward with steep right turn")
            twist_msg.linear.x = linear_speed
            twist_msg.linear.y = linear_speed * 0.7
        elif driving_mode == DrivingMode.MILD_LEFT_FORWARD:
            # print("- moving forward with smooth left turn")
            twist_msg.linear.x = linear_speed * 0.9
            twist_msg.linear.y = linear_speed
        elif driving_mode == DrivingMode.MILD_RIGHT_FORWARD:
            # print("- moving forward with smooth right turn")
            twist_msg.linear.x = linear_speed
            twist_msg.linear.y = linear_speed * 0.9
        else:
            twist_msg.linear.x = 0.01
            twist_msg.linear.y = 0.01

        if self.cmd_vel_nav_pub is not None:
            self.cmd_vel_nav_pub.publish(twist_msg)

    def turn_around(self, angle):
        if self.lining_up_request:
            self.driving_mode_disp = "Lining Up" + f"({angle:.2f} deg)"
        else:
            self.driving_mode_disp = DrivingMode.TURN_AROUND.name + f"({angle:.2f} deg)"

        if abs(angle) < 3.0:  # no need to turn if the angle is already small enough
            angular_speed = 0.01
            self.lining_up_request = False
        elif abs(angle) < 10.0:
            angular_speed = 0.2
        elif abs(angle) < 30.0:
            angular_speed = self.angular_speed * 0.5
        else:
            angular_speed = self.angular_speed

        twist_msg = Twist()
        twist_msg.linear.x = angular_speed if angle < 0 else 0.01
        twist_msg.linear.y = angular_speed if angle > 0 else 0.01
        if self.cmd_vel_nav_pub is not None:
            self.cmd_vel_nav_pub.publish(twist_msg)

    def stop_drive(self):
        self.driving_mode_disp = DrivingMode.STOP.name
        twist_msg = Twist()
        twist_msg.linear.x = 0.01
        twist_msg.linear.y = 0.01
        if self.cmd_vel_nav_pub is not None:
            self.cmd_vel_nav_pub.publish(twist_msg)
