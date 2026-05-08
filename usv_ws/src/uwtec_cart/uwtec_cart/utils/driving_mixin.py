from enum import Enum
import copy
from geometry_msgs.msg import Twist

from uwtec_cart.utils import (
    distance_and_bearing_xy,
    shortest_path_to_track,
    rotate_to_go,
)


class DrivingMode(Enum):
    READY = 1
    RUNNING = 2
    FINISHED = 3
    # HARD_LEFT_FORWARD = 2
    # HARD_RIGHT_FORWARD = 3
    # MILD_LEFT_FORWARD = 4
    # MILD_RIGHT_FORWARD = 5
    FORWARD = 4
    TURN_AROUND = 5
    TURN_TEST = 6
    RETURN_TO_ROUTE = 7
    STOP = 8
    CALIBRATING = 9
    MOTOR_TEST = 10


class DrivingMixin:
    def ___init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # initialize instance variables: will be overridden by child class
        self.debug = False
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.cmd_vel_pub = None
        self.twist = Twist()
        self.prev_twist = copy.deepcopy(self.twist)

    def go_driving(
        self,
        mode,
        current_utm,
        current_heading,
        start_utm,  # src_utm,
        goal_utm,  # dst_utm,
    ) -> DrivingMode:
        distance_planned, heading_planned = distance_and_bearing_xy(start_utm, goal_utm)
        distance_traveled, _ = distance_and_bearing_xy(start_utm, current_utm)
        distance_remaining, heading_to_goal = distance_and_bearing_xy(
            current_utm, goal_utm
        )
        distance_to_track, heading_to_track = shortest_path_to_track(
            start_utm, goal_utm, current_utm
        )

        if mode == DrivingMode.STOP:
            self.stop()
            mode = DrivingMode.FINISHED

        elif mode == DrivingMode.FORWARD:
            if distance_traveled > distance_planned or distance_remaining < 0.2:
                mode = DrivingMode.FINISHED
            else:
                self.forward(distance_remaining, distance_traveled)

        elif mode == DrivingMode.TURN_TEST:
            rotation_remaining = rotate_to_go(current_heading, heading_planned)
            if abs(rotation_remaining) < 3.0:
                mode = DrivingMode.FINISHED
            else:
                self.turn(angle=rotation_remaining)

        elif mode == DrivingMode.TURN_AROUND:
            rotation_remaining = rotate_to_go(current_heading, heading_to_goal)
            if abs(rotation_remaining) < 3.0:
                mode = DrivingMode.RUNNING
            else:
                self.turn(angle=rotation_remaining)

        # If we are currently in RETURN_TO_ROUTE mode, we want to prioritize returning to the route before doing anything else
        elif mode == DrivingMode.RETURN_TO_ROUTE:
            # if close enough to track, switch back to running mode
            if distance_to_track < 0.2:
                mode = DrivingMode.RUNNING
            else:
                rotation_remaining = rotate_to_go(current_heading, heading_to_track)
                self.turn_and_forward(
                    distance=distance_to_track, angle=rotation_remaining
                )

        elif mode == DrivingMode.RUNNING:
            rotation_remaining = rotate_to_go(current_heading, heading_to_goal)
            if distance_traveled > distance_planned or distance_remaining < 0.2:
                mode = DrivingMode.FINISHED

            # if off track by more than 1m, prioritize returning to route before moving forward
            elif distance_to_track > 1.0:
                mode = DrivingMode.RETURN_TO_ROUTE

            # if need to turn more than 60 degrees, prioritize turning around in place before moving forward
            elif abs(rotation_remaining) > 60.0:
                mode = DrivingMode.TURN_AROUND

            # otherwise, turn and forward simultaneously towards the destination
            else:
                self.steering_forward(
                    distance=distance_remaining,
                    angle=rotation_remaining,
                    traveled=distance_traveled,
                )

        else:
            pass  # do not change mode

        return mode

    def stop(self):
        if self.cmd_vel_pub is not None:
            self.twist.linear.x = 0.01
            self.twist.linear.y = 0.01
            self.cmd_vel_pub.publish(self.twist)
            self.prev_twist = copy.deepcopy(self.twist)

    def simple_forward(self, left_speed: float = 0.0, right_speed: float = 0.0):
        if left_speed == 0.0 or right_speed == 0.0:
            self.twist.linear.x = self.linear_speed
            self.twist.linear.y = self.linear_speed
        else:
            self.twist.linear.x = left_speed
            self.twist.linear.y = right_speed

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
