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

        rotation_remaining = rotate_to_go(current_heading, heading_current_to_dst)

        # If we are currently in TURN_AROUND mode, we want to finish turning around before doing anything else
        if self.driving_mode == DrivingMode.TURN_AROUND:
            if abs(rotation_remaining) < 3.0:
                self.driving_mode = DrivingMode.READY
            else:
                self.turn(rotation_remaining)
            return distance_current_to_dst

        # # If we are currently in RETURN_TO_ROUTE mode, we want to prioritize returning to the route before doing anything else
        # if self.driving_mode == DrivingMode.RETURN_TO_ROUTE:
        #     # if close enough to track, switch back to forward mode
        #     if distance_to_track < 0.2:
        #         self.driving_mode = DrivingMode.READY
        #     else:
        #         rotation_remaining = rotate_to_go(current_heading, heading_to_track)
        #         self.turn_and_forward(distance_to_track, rotation_remaining)
        #     return distance_current_to_dst

        # If we have reached the destination (within 20 cm) and are past the destination, we can stop and switch to READY mode
        if (
            distance_src_to_current > distance_src_to_dst
            and distance_current_to_dst < 0.2
        ):
            self.driving_mode = DrivingMode.READY
            return 0.0  # reached destination, no need to move

        # # If off track by more than 1m, prioritize returning to route
        # if distance_to_track > 1.0:
        #     self.driving_mode = DrivingMode.RETURN_TO_ROUTE
        #     return distance_current_to_dst

        # If need to turn more than 60 degrees, prioritize turning around in place before moving forward
        if abs(rotation_remaining) > 60.0:
            self.driving_mode = DrivingMode.TURN_AROUND
            return distance_current_to_dst

        # Otherwise, turn and forward simultaneously towards the destination
        self.steering_forward(
            distance=distance_current_to_dst,
            angle=rotation_remaining,
            traveled=distance_src_to_current,
        )
        return distance_current_to_dst

    # def go_driving_old(self, src_utm, dst_utm, current_utm, current_heading):
    #     distance_src_to_dst, heading_src_to_dst = distance_and_bearing_xy(
    #         src_utm, dst_utm
    #     )
    #     distance_src_to_current, heading_src_to_current = distance_and_bearing_xy(
    #         src_utm, current_utm
    #     )
    #     distance_current_to_dst, heading_current_to_dst = distance_and_bearing_xy(
    #         current_utm, dst_utm
    #     )
    #     distance_to_track, heading_to_track = shortest_path_to_track(
    #         src_utm, dst_utm, current_utm
    #     )
    #
    #     if (
    #         distance_src_to_current > distance_src_to_dst
    #         and distance_current_to_dst < 0.2
    #     ):  # 20 cm tolerance
    #         return 0.0  # reached destination, no need to move
    #
    #     if self.driving_mode == DrivingMode.RETURN_TO_ROUTE:
    #         if (
    #             distance_to_track < 0.2
    #         ):  # if close enough to track, switch back to forward mode
    #             self.driving_mode = DrivingMode.READY
    #         else:
    #             rotation_remaining = rotate_to_go(current_heading, heading_to_track)
    #             self.turn_and_forward(distance_to_track, rotation_remaining)
    #
    #     else:
    #         if (
    #             distance_to_track > 1.0
    #         ):  # if off track by more than 1m, prioritize returning to route
    #             self.driving_mode = DrivingMode.RETURN_TO_ROUTE
    #         else:
    #             rotation_remaining = rotate_to_go(
    #                 current_heading, heading_current_to_dst
    #             )
    #             # if need to turn more than 30 degrees, turn in place
    #             if abs(rotation_remaining) > 30.0:
    #                 # if abs(rotation_remaining) > 180.0:
    #                 #     print(
    #                 #         f"Yay: {current_heading:.2f} -> {heading_current_to_dst:.2f} = {rotation_remaining:.2f}"
    #                 #     )
    #                 self.turn(rotation_remaining)
    #             # otherwise, turn and forward simultaneously
    #             else:
    #                 self.steering_forward(distance_current_to_dst, rotation_remaining)
    #
    #     return distance_current_to_dst

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

    def forward(self, distance: float, traveled: float = 0.0):
        # print(f"Distance to target: {distance:.2f}, Distance traveled: {traveled:.2f}")
        if (
            traveled < 0.5 or distance < 0.5
        ):  # slow down when just starting to move or when close to target
            linear_speed = 0.2
        elif distance < 0.2:  # 20 cm tolerance
            linear_speed = 0.01  # don't move
        elif distance < 1.0:  # slow down when approaching target
            linear_speed = self.linear_speed * 0.5
        else:
            linear_speed = self.linear_speed

        linear_speed = 0.2
        self.twist.linear.x = linear_speed
        self.twist.linear.y = linear_speed

        # Only publish if not already moving forward to avoid unnecessary messages
        if self.twist != self.prev_twist and self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)
        # else:
        #     print(
        #         f"Moving forward with linear speed: {linear_speed:.2f} for distance: {distance:.2f}"
        #     )

    def steering_forward(self, distance: float, angle: float, traveled: float = 0.0):
        # print(f"Distance to target: {distance:.2f}, Distance traveled: {traveled:.2f}")
        #
        if (
            traveled < 0.5 or distance < 0.5
        ):  # slow down when just starting to move or when close to target
            linear_speed = 0.2
        elif distance < 0.2:  # 20 cm tolerance
            linear_speed = 0.01  # don't move
        elif distance < 1.0:  # slow down when approaching target
            linear_speed = self.linear_speed * 0.5
        else:
            linear_speed = self.linear_speed

        # a proportional extra speed based on how far off the angle is, with a max of 0.1 at 60 degrees
        extra_speed = round(0.1 * (angle / 60.0), 2)

        if angle < 0:  # need to turn right, slow down right side
            left_speed = linear_speed
            right_speed = linear_speed + extra_speed
        elif angle > 0:  # need to turn left, slow down left side
            left_speed = linear_speed - extra_speed
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
        # print(f"Angle to target: {angle:.2f} degrees")
        abs_angle = abs(angle)

        if abs_angle < 3.0:  # if within 3 degrees, consider it aligned
            angular_speed = 0.01  # don't turn
        elif abs_angle < 10.0:  # slow down when close to target
            angular_speed = 0.2  # least speed to avoid overshooting
        elif abs_angle < 30.0:  # slow down when approaching target
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

    def turn_and_forward(self, distance: float, angle: float, traveled: float = 0.0):
        if abs(angle) >= 3.0:
            self.turn(angle)
        else:
            self.forward(distance, traveled)
