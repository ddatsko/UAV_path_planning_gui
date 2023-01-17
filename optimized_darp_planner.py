import json
import subprocess
import os
from utils import gps_coordinates_to_meters

_TEMP_FILE_FILENAME = ".__optimized_darp_input.json"


def _make_json_points(p):
    return {'lat': p[0],
            'long': p[1]}


def _generate_input_file(n_uavs, fly_zone, no_fly_zones, sweeping_step, start_pos):
    input_file_data = {
        'droneNo': int(n_uavs),
        'scanningDensity': int(sweeping_step),
        'polygon': list(map(_make_json_points, fly_zone)),
        'obstacles': list(map(lambda obstacle: list(map(_make_json_points, obstacle)), no_fly_zones)),
        'pathsStrictlyInPoly': False,
        'initialPos': [_make_json_points(start_pos) for _ in range(int(n_uavs))],
        'rPortions': [1 / float(n_uavs)] * int(n_uavs)
    }
    with open(_TEMP_FILE_FILENAME, 'w') as f:
        json.dump(input_file_data, f)


def _add_travelling_to_path(path, start_point):
    # THe trajectory is circular, so just find the closest point and mark it as the start
    closest_point = 0
    min_distance = float('inf')
    transform_origin = path[0]
    for i in range(len(path)):
        p_m = gps_coordinates_to_meters(*(path[i]), *transform_origin)
        distance = (p_m[0] ** 2 + p_m[1] ** 2) ** 0.5
        if distance < min_distance:
            min_distance = distance
            closest_point = i
    path = path[:-1]
    path = [list(reversed(start_point))] + path[closest_point:] + path[:closest_point] + [path[closest_point]] + [list(reversed(start_point))]
    return path



def plan_optimized_darp_paths(json_data):
    # Firstly, generate the input file for the algorithm
    _generate_input_file(json_data['n-uavs'],
                         json_data['fly-zone'],
                         json_data['no-fly-zones'],
                         json_data['sweeping-step'],
                         json_data['start-point'])

    # Now, run the solver
    output = list(filter(lambda x: x, subprocess.check_output(['java', '-jar', 'mCPP-optimized-DARP.jar', _TEMP_FILE_FILENAME]).decode('utf-8').split('\n')))

    os.remove(_TEMP_FILE_FILENAME)

    # Get the paths points from solver
    paths = []
    for i in range(int(json_data['n-uavs'])):
        points = []
        for line in reversed(output):
            try:
                lat, lon = map(float, line[:-1].split(', '))
                points.append([lon, lat])
            except Exception as e:
                paths.append(list(reversed(points)))
                output = output[:-len(points) - 1]
                break
    res = list(map(lambda path: _add_travelling_to_path(path, json_data['start-point']), paths))
    return res