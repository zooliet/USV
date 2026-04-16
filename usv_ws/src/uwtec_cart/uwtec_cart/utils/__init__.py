import os
import yaml
import math
from ament_index_python.packages import get_package_share_directory


def set_config_value(key, value):
    config_file_path = os.path.join(
        get_package_share_directory("uwtec_cart"), "config", "system.yaml"
    )

    try:
        with open(config_file_path, "r") as config_file:
            config = yaml.safe_load(config_file) or {}
    except FileNotFoundError:
        config = {}

    config[key] = value

    with open(config_file_path, "w") as config_file:
        yaml.safe_dump(config, config_file)


def get_config_value(key, default):
    config_file_path = os.path.join(
        get_package_share_directory("uwtec_cart"), "config", "system.yaml"
    )

    try:
        with open(config_file_path, "r") as config_file:
            config = yaml.safe_load(config_file) or {}
            value = config.get(key, default)
    except Exception as _:
        value = default
    return value


def check_timeout(ticks, seconds, interval):
    mod = int(seconds / interval)
    return ticks % mod == 0


def calc_offset(heading: float, yaw: float) -> float:
    # heading: positive(0 ~ 360) for ccw
    # yaw: positive(0 ~ 360) for ccw
    offset = signed_angle(heading - yaw)
    return offset  # positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw


def calc_heading_from_yaw_and_offset(yaw: float, offset: float) -> float:
    # yaw: positive(0 ~ 360) for ccw
    # offset: positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw
    heading = (yaw + offset) % 360
    return heading  # positive(0 ~ 360) for ccw


def signed_angle(degree: float) -> float:
    sign = (
        -1 if (degree > 180 and degree < 360) or (degree < 0 and degree > -180) else 1
    )
    degree = degree % (sign * 360)
    return degree  # positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw


def utm_distance(src_pos, dst_pos):
    x1, y1 = src_pos
    x2, y2 = dst_pos
    distance_src_to_dst = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance_src_to_dst


def utm_bearing(src_pos, dst_pos):
    x1, y1 = src_pos
    x2, y2 = dst_pos
    delta_x = x2 - x1
    delta_y = y2 - y1
    bearing_rad = math.atan2(delta_y, delta_x)
    bearing_deg = math.degrees(bearing_rad)
    return bearing_deg % 360  # positive(0 ~ 360) for ccw


def distance_and_bearing_xy(
    src_pos: tuple[float, float], dst_pos: tuple[float, float]
) -> tuple[float, float]:
    distance = utm_distance(src_pos, dst_pos)
    bearing = utm_bearing(src_pos, dst_pos)
    # bearing = signed_angle(bearing) # positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw
    return distance, bearing  # 0~360: ccw


def calc_goal_heading(heading: float, by: float) -> float:
    # heading: positive(0 ~ 360) for ccw
    # by: positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw
    goal_heading = (heading + by) % 360
    return goal_heading  # positive(0 ~ 360) for ccw


def rotate_to_go(current_heading: float, goal_heading: float) -> float:
    # current_heading: positive(0 ~ 360) for ccw
    # goal_heading: positive(0 ~ 360) for ccw
    rotation_needed = signed_angle(goal_heading - current_heading)
    return rotation_needed  # positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw


def calc_goal_coordinates(
    current_coordinates: tuple[float, float],
    distance: float,
    goal_heading: float,
) -> tuple[float, float]:
    # current_coordinates: (utm_x, utm_y)
    # distance: in meters
    # goal_heading: positive(0 ~ 360) for ccw
    utm_x, utm_y = current_coordinates
    rad = math.radians(goal_heading)
    delta_x = distance * math.cos(rad)
    delta_y = distance * math.sin(rad)
    goal_utm_x = utm_x + delta_x
    goal_utm_y = utm_y + delta_y
    return goal_utm_x, goal_utm_y


def distance_to_go(src_pos_x, src_pos_y, dst_pos_x, dst_pos_y):
    return utm_distance((src_pos_x, src_pos_y), (dst_pos_x, dst_pos_y))


def shortest_path_to_track(src_pos, dst_pos, current_pos):
    src_pox_x, src_pos_y = src_pos
    dst_pos_x, dst_pos_y = dst_pos
    current_pos_x, current_pos_y = current_pos

    delta_x = dst_pos_x - src_pox_x
    delta_y = dst_pos_y - src_pos_y

    # calculate the projection of current_pos onto the line defined by src_pos and dst_pos
    # it is the parameter that defines the position of the projection on the line
    #
    if delta_x == 0 and delta_y == 0:
        return distance_to_go(src_pox_x, src_pos_y, current_pos_x, current_pos_y), 0.0

    # avoid division by zero
    t = (
        (current_pos_x - src_pox_x) * delta_x + (current_pos_y - src_pos_y) * delta_y
    ) / (delta_x**2 + delta_y**2)
    # clamp t to the range [0, 1] to ensure the projection is on the line segment
    t = max(0, min(1, t))
    # calculate the coordinates of the projection
    projection_x = src_pox_x + t * delta_x
    projection_y = src_pos_y + t * delta_y
    # print(
    #     f"Projection of current position onto line: ({projection_x:.2f}, {projection_y:.2f})"
    # )

    # calculate the distance from current_pos to the projection
    distance_current_to_projection = utm_distance(
        (current_pos_x, current_pos_y), (projection_x, projection_y)
    )
    # calculate the bearing from current_pos to the projection
    heading_current_to_projection = utm_bearing(
        (current_pos_x, current_pos_y), (projection_x, projection_y)
    )
    # # convert bearing to signed angle: positive(0 ~ 180) for ccw, negative(0 ~ -180) for cw
    # heading_current_to_projection = signed_angle(heading_current_to_projection)

    return distance_current_to_projection, heading_current_to_projection


def get_waypoints_from_route_file(route_file):
    route_file_path = os.path.join(
        get_package_share_directory("uwtec_cart"), "routes", route_file
    )
    # read route.yaml file and extract waypoints:
    #  - latitude: 37.123456
    #    longitude: 127.123456
    #  - latitude: 37.654321
    #    longitude: 127.654321
    coords = []
    try:
        with open(route_file_path, "r") as f:
            coords = yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f"Failed to read route file '{route_file}': {e}")

    return coords
