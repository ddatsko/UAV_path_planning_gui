import copy
import subprocess
from geometry_msgs.msg import Point, Point32
from math import cos, pi
from typing import List, Tuple
import os
import rosservice
import rospy
from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference
from copy import deepcopy

METERS_IN_DEGREE = 111319.5
ENERGY_EXECUTABLE_PATH = '/PATH_TO_EXECUTABLE'
#ENERGY_EXECUTABLE_PATH = '/home/mrs/RAL_coverage/energy_with_toppra/build/energy_with_toppra'
#ENERGY_EXECUTABLE_PATH = './energy_calculation'

class PathProperties:
    def __init__(self, energy, time):
        self.energy = energy
        self.time = time


def gps_coordinates_to_meters(x, y, origin_x, origin_y):
    meters_in_long_degree = cos((origin_x / 180) * pi) * METERS_IN_DEGREE
    return y * meters_in_long_degree - origin_y * meters_in_long_degree, \
        x * METERS_IN_DEGREE - origin_x * METERS_IN_DEGREE


def meters_to_gps_coordinates(x, y, origin_x, origin_y):
    meters_in_long_degree = cos((origin_x / 180) * pi) * METERS_IN_DEGREE
    return (y + origin_x * METERS_IN_DEGREE) / METERS_IN_DEGREE, \
           (x + origin_y * meters_in_long_degree) / meters_in_long_degree


def meter_point_from_gps(x, y, origin_x, origin_y) -> Point32:
    new_x, new_y = gps_coordinates_to_meters(x, y, origin_x, origin_y)
    return Point32(new_x, new_y, 20)


def gps_point_from_meters(x, y, origin_x, origin_y) -> (float, float):
    new_x, new_y = meters_to_gps_coordinates(x, y, origin_x, origin_y)
    return new_y, new_x


def get_path_properties(path: List[Tuple[float, float, float]]):
    TEMP_CSV_FILE = ".__temp.csv"
    if os.path.exists(TEMP_CSV_FILE):
        os.remove(TEMP_CSV_FILE)
    with open(TEMP_CSV_FILE, 'w') as f:
        for p in path:
            print(f'{p[0]},{p[1]},', file=f)

    output = subprocess.check_output([ENERGY_EXECUTABLE_PATH, TEMP_CSV_FILE])
    os.remove(TEMP_CSV_FILE)
    return tuple(map(float, output.decode('utf-8').split('\n')[-2].split(',')))


def send_path_to_service(path, service):
    service_list = rosservice.get_service_list()
    if service not in service_list:
        return False, "Service not in service list"
    try:
        follow_path = rospy.ServiceProxy(service, PathSrv)
        path_srv = PathSrvRequest(path)
        res = follow_path(path_srv)
        if not res.success:
            return False, res.message
        else:
            return True, res.message
    except Exception as e:
        return False, "Error while calling service: " + str(e)


# TODO: remove importing own_planner here, as this module should not import any other modules, as it can be imported there,
# which will lead to a circular imports
from own_planner import get_last_own_paths


def compose_path_messages(json_data: dict, paths_points: List[List[List]]) -> List[PathSrvRequest]:
    common_message = PathSrvRequest()

    last_own_paths = get_last_own_paths()
    
    common_message.path.header = last_own_paths[0].header

    # Fill in header fields
    # common_message.path.header.stamp = rospy.Time.now()
    #common_message.path.header.frame_id = "latlon_origin"

    common_message.path.fly_now = False
    common_message.path.use_heading = True

    common_message.path.fly_now = True
    common_message.path.use_heading = True
    common_message.path.stop_at_waypoints = False
    common_message.path.loop = False
    common_message.path.override_constraints = True

    common_message.path.override_max_velocity_horizontal = last_own_paths[0].override_max_velocity_horizontal
    common_message.path.override_max_acceleration_horizontal = last_own_paths[0].override_max_acceleration_horizontal
    common_message.path.override_max_jerk_horizontal = 200
    common_message.path.override_max_jerk_vertical = 200
    common_message.path.override_max_acceleration_vertical = 1
    common_message.path.override_max_velocity_vertical = last_own_paths[0].override_max_velocity_vertical

    common_message.path.relax_heading = False

    res = []
    for path in paths_points:
        points = [Reference(Point(p[1], p[0], float(json_data['altitude'])), 0.0) for p in path]
        message = copy.deepcopy(common_message)
        # message = common_message
        message.path.points = points
        res.append(message.path)
    return res
