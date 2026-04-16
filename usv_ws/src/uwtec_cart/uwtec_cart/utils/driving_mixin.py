from enum import Enum
import copy
from geometry_msgs.msg import Twist

from uwtec_cart.utils import (
    distance_and_bearing_xy,
    shortest_path_to_track,
    rotate_to_go,
)


class OperationMode(Enum):
    START_OVER = 1
    RUNNING = 2
    PAUSED = 3
    TURN_AROUND = 4
    FINISHED = 5


class DrivingMode(Enum):
    READY = 0
    FORWARD = 1
    HARD_LEFT_FORWARD = 2
    HARD_RIGHT_FORWARD = 3
    MILD_LEFT_FORWARD = 4
    MILD_RIGHT_FORWARD = 5
    TURN_AROUND = 6
    STOP = 7
    RETURN_TO_ROUTE = 8


class DrivingMixin:
    def ___init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # initialize instance variables: will be overridden by child class
        self.debug = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.prev_linear_speed = 0.0
        self.prev_angular_speed = 0.0
        self.lining_up_requested = False
        self.driving_mode = DrivingMode.READY
        self.cmd_vel_pub = None
        self.twist = Twist()
        self.prev_twist = copy.deepcopy(self.twist)

    def go_driving(self, src_utm, dst_utm, current_utm, current_heading):
        distance_src_to_dst, heading_src_to_dst = distance_and_bearing_xy(
            src_utm, dst_utm
        )
        distance_src_to_current, heading_src_to_current = distance_and_bearing_xy(
            src_utm, current_utm
        )
        distance_current_to_dst, heading_current_to_dst = distance_and_bearing_xy(
            current_utm, dst_utm
        )
        distance_to_track, heading_to_track = shortest_path_to_track(
            src_utm, dst_utm, current_utm
        )

        if distance_current_to_dst < 0.2:  # 20 cm tolerance
            return 0.0  # reached destination, no need to move

        if self.driving_mode == DrivingMode.RETURN_TO_ROUTE:
            if (
                distance_to_track < 0.2
            ):  # if close enough to track, switch back to forward mode
                self.driving_mode = DrivingMode.READY
            else:
                rotation_remaining = rotate_to_go(current_heading, heading_to_track)
                self.turn_and_forward(distance_to_track, rotation_remaining)

        else:
            if (
                distance_to_track > 1.0
            ):  # if off track by more than 1m, prioritize returning to route
                self.driving_mode = DrivingMode.RETURN_TO_ROUTE
            else:
                rotation_remaining = rotate_to_go(
                    current_heading, heading_current_to_dst
                )
                # if need to turn more than 30 degrees, turn in place
                if abs(rotation_remaining) > 30.0:
                    # if abs(rotation_remaining) > 180.0:
                    #     print(
                    #         f"Yay: {current_heading:.2f} -> {heading_current_to_dst:.2f} = {rotation_remaining:.2f}"
                    #     )
                    self.turn(rotation_remaining)
                # otherwise, turn and forward simultaneously
                else:
                    self.steering_forward(distance_current_to_dst, rotation_remaining)

        return distance_current_to_dst

    # def stop_driving(self):
    #     pass

    def stop(self):
        if self.cmd_vel_pub is not None:
            self.twist.linear.x = 0.01
            self.twist.linear.y = 0.01
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)

    def simple_forward(self):
        self.twist.linear.x = self.linear_speed
        self.twist.linear.y = self.linear_speed

        # Only publish if not already moving forward to avoid unnecessary messages
        if self.twist != self.prev_twist and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)

    def forward(self, distance: float):
        if distance < 0.2:  # 20 cm tolerance
            linear_speed = 0.01  # don't move
        elif distance < 0.5:  # slow down when close to target
            linear_speed = 0.2  # least speed to avoid overshooting
        elif distance < 1.0:  # slow down when approaching target
            linear_speed = self.linear_speed * 0.5
        else:
            linear_speed = self.linear_speed

        self.twist.linear.x = linear_speed
        self.twist.linear.y = linear_speed

        # Only publish if not already moving forward to avoid unnecessary messages
        if self.twist != self.prev_twist and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)
            print(
                f"Moving forward with linear speed: {linear_speed:.2f} for distance: {distance:.2f}"
            )

    def steering_forward(self, distance: float, angle: float):
        if distance < 0.5:  # slow down when close to target
            linear_speed = 0.2  # least speed to avoid overshooting
        elif distance < 1.0:  # slow down when approaching target
            linear_speed = self.linear_speed * 0.5
        else:
            linear_speed = self.linear_speed

        if abs(angle) < 3.0:  # 3 degree tolerance
            extra_speed = 0.0  # don't turn
        elif abs(angle) < 10.0:  # slow down when close to target
            extra_speed = 0.1  # least speed to avoid overshooting
        elif abs(angle) < 30.0:  # slow down when approaching target
            extra_speed = 0.2
        else:
            extra_speed = 0.3

        if angle < 0:  # need to turn right, slow down right side
            left_speed = linear_speed
            right_speed = max(0.01, linear_speed - extra_speed)
        elif angle > 0:  # need to turn left, slow down left side
            left_speed = max(0.01, linear_speed - extra_speed)
            right_speed = linear_speed
        else:  # no need to turn, go straight
            left_speed = linear_speed
            right_speed = linear_speed

        self.twist.linear.x = left_speed
        self.twist.linear.y = right_speed

        # Only publish if not already moving forward to avoid unnecessary messages
        if self.twist != self.prev_twist and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)
            print(f"Steering forward: L({left_speed:.2f}), R({right_speed:.2f})")

    def turn(self, angle: float):
        # if angle > 180.0:
        #     print(f"Angle {angle:.2f} is greater than 180 degrees")

        if abs(angle) < 3.0:  # 3 degree tolerance
            angular_speed = 0.01  # don't turn
        elif abs(angle) < 10.0:  # slow down when close to target
            angular_speed = 0.2  # least speed to avoid overshooting
        elif abs(angle) < 30.0:  # slow down when approaching target
            angular_speed = self.angular_speed * 0.5
        else:
            angular_speed = self.angular_speed

        self.twist.linear.x = angular_speed if angle < 0 else 0.01
        self.twist.linear.y = angular_speed if angle > 0 else 0.01

        # Only publish if not already moving forward to avoid unnecessary messages
        if self.twist != self.prev_twist and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)
            print(
                f"Turning with angular speed: {angular_speed:.2f} for angle: {angle:.2f}"
            )

    def turn_and_forward(self, distance: float, angle: float):
        if angle >= 3.0:
            self.turn(angle)
        else:
            self.forward(distance)
