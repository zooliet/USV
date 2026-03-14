import os
import yaml
import math
from ament_index_python.packages import get_package_share_directory


def signed_angle(degree: float) -> float:
    sign = (
        -1 if (degree > 180 and degree < 360) or (degree < 0 and degree > -180) else 1
    )
    degree = degree % (sign * 360)
    return degree  # positive(0~180) for ccw, negative(0~-180) for cw


def utm_distance(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance


def utm_bearing(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    angle = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 360
    return angle  # normalize to [0, 360)


def distance_and_bearing_xy(
    p1: tuple[float, float], p2: tuple[float, float]
) -> tuple[float, float]:
    distance = utm_distance(p1, p2)
    bearing = utm_bearing(p1, p2)  # 0~360, positive for ccw, negative for cw
    bearing = signed_angle(
        bearing
    )  # convert to -180~180, positive for ccw, negative for cw
    return distance, bearing


def shortest_distance_and_angle_to_line(
    p1: tuple[float, float], p2: tuple[float, float], p0: tuple[float, float]
) -> tuple[float, float]:
    x1, y1 = p1
    x2, y2 = p2
    x0, y0 = p0

    dx = x2 - x1
    dy = y2 - y1

    # Calculate the projection of point (x0, y0) onto the line defined by (x1, y1) and (x2, y2)
    t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)
    # Clamp t to the range [0, 1] to find the closest point on the line segment
    t = max(0, min(1, t))
    # Find the closest point on the line segment
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    # print(f"Closest Point: ({closest_x}, {closest_y})")

    # Calculate distance from (x0, y0) to the closest point
    distance = math.sqrt((x0 - closest_x) ** 2 + (y0 - closest_y) ** 2)

    # Calculate angle from (x0, y0) to the closest point: 0~180 for ccw, 0~-180 for cw
    angle = math.degrees(math.atan2(closest_y - y0, closest_x - x0))
    return distance, angle


def utm_speed(p1, p2, time_diff):
    distance = utm_distance(p1, p2)
    speed = distance / time_diff if time_diff > 0 else 0
    return speed


def calc_offset(heading: float, yaw: float) -> float:
    # heading: 0~360,
    # yaw: -180~180, positive for ccw, negative for cw
    # offset: -180~180, positive for ccw, negative for cw
    offset = signed_angle(heading - yaw)
    return offset


def calc_heading_by_offset(yaw: float, offset: float) -> float:
    # yaw: -180~180, positive for ccw, negative for cw
    # offset: -180~180, positive for ccw, negative for cw
    # heading: 0~360
    heading = (yaw + offset) % 360
    return heading  # normalize to [0, 360)


def calc_goal_heading(heading: float, by: float) -> float:
    # heading: 0~360
    # by: -180~180, positive for ccw, negative for cw
    # goal_heading: 0~360
    goal_heading = (heading + by) % 360
    return goal_heading  # normalize to [0, 360)


def rotate_to_go(current_heading: float, goal_heading: float) -> float:
    deg = (goal_heading - current_heading) % 360
    if deg > 180:
        deg = deg % -360
    return deg  # -180~180, positive for ccw, negative for cw


if __name__ == "__main__":
    print("\nTest the signed_angle function with various degree values:")
    degrees = [0, 90, 180, 270, 360, -90, -180, -270, -360]
    for degree in degrees:
        print(f"Degree: {degree}, Signed Angle: {signed_angle(degree)}")

    print("\nTest the utm_distance function with sample points:")
    points = [((0, 0), (3, 4)), ((1, 1), (4, 5)), ((-1, -1), (-4, -5))]
    for p1, p2 in points:
        print(f"Point 1: {p1}, Point 2: {p2}, Distance: {utm_distance(p1, p2)}")

    print("\nTest the utm_bearing function with sample points:")
    points = [((0, 0), (1, 0)), ((0, 0), (0, 1)), ((0, 0), (-1, 0)), ((0, 0), (0, -1))]
    for p1, p2 in points:
        print(f"Point 1: {p1}, Point 2: {p2}, Bearing: {utm_bearing(p1, p2)} degrees")

    print("\nTest the calc_offset function with various heading and yaw values:")
    test_cases = [
        (0, 0),
        (90, 0),
        (180, 0),
        (270, 0),
        (360, 0),
        (0, -90),
        (90, -90),
        (180, -90),
        (270, -90),
        (360, -90),
        (45, 45),
        (45, -45),
        (315, 45),
        (315, -45),
    ]
    for heading, yaw in test_cases:
        offset = calc_offset(heading, yaw)
        print(
            f"Heading: {heading} degrees, Yaw: {yaw} degrees, Offset: {offset} degrees"
        )

    print(
        "\nTest the calc_heading_by_offset function with various yaw and offset values:"
    )
    test_cases = [
        (0, 0),
        (90, 0),
        (180, 0),
        (270, 0),
        (360, 0),
        (0, -90),
        (90, -90),
        (180, -90),
        (270, -90),
        (360, -90),
        (45, 45),
        (45, -45),
        (315, 45),
        (315, -45),
    ]
    for yaw, offset in test_cases:
        heading = calc_heading_by_offset(yaw, offset)
        print(
            f"Yaw: {yaw} degrees, Offset: {offset} degrees, Heading: {heading} degrees"
        )

    print(
        "\nTest the calc_goal_heading function with various current headings and by values:"
    )
    test_cases = [
        (0, 90),
        (90, 90),
        (180, 90),
        (270, 90),
        (360, 90),
        (0, -90),
        (90, -90),
        (180, -90),
        (270, -90),
        (360, -90),
        (45, 45),
        (45, -45),
        (315, 45),
        (315, -45),
    ]
    for current_heading, by in test_cases:
        goal_heading = calc_goal_heading(current_heading, by)
        print(
            f"Current Heading: {current_heading} degrees, By: {by} degrees, Goal Heading: {goal_heading} degrees"
        )

    print("\nTest the rotate_to_go function with various current and goal headings:")
    test_cases = [
        (0, 90),
        (90, 180),
        (180, 270),
        (270, 360),
        (360, 0),
        (0, 270),
        (90, 0),
        (180, 90),
        (270, 180),
        (360, 270),
        (45, 315),
        (315, 45),
    ]
    for current_heading, goal_heading in test_cases:
        rotation = rotate_to_go(current_heading, goal_heading)
        print(
            f"Current Heading: {current_heading} degrees, Goal Heading: {goal_heading} degrees, Rotation to Go: {rotation} degrees"
        )


def get_waypoints_from_yaml(wps_name):
    wps_yaml_path = os.path.join(
        get_package_share_directory("uwtec_navigation"), "config", wps_name
    )
    coords = []
    with open(wps_yaml_path, "r") as wps_file:
        coords = yaml.safe_load(wps_file)
    return coords, 0


def get_gyro_offset(heading_file_name):
    heading_yaml_path = os.path.join(
        get_package_share_directory("uwtec_navigation"), "config", heading_file_name
    )

    with open(heading_yaml_path, "r") as heading_file:
        data = yaml.safe_load(heading_file)
        offset = data.get("offset", 0.0)
    return offset


def update_ticks(ticks_for_5_secs, ticks_for_1_sec, interval=0.1):
    ticks_for_5_secs = (
        ticks_for_5_secs - 1 if ticks_for_5_secs > 1 else int(5.0 / interval)
    )
    ticks_for_1_sec = (
        ticks_for_1_sec - 1 if ticks_for_1_sec > 1.0 else int(1.0 / interval)
    )
    return ticks_for_5_secs, ticks_for_1_sec
