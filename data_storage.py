import os
import pathlib
import shutil
import json


FLY_ZONE_FILENAME = 'fly_zone.csv'
NO_FLY_ZONES_FILENAME_TEMPLATE = 'no_fly_zone_{}.csv'
START_POINT_FILENAME = 'start_point.csv'
PATHS_FOLDER_PREFIX = 'path_'
CONFIG_EXCLUDED_FIELDS = ['fly-zone', 'no-fly-zones', 'start-point']
CONFIG_JSON_FILENAME = 'config.json'
EXPERIMENTS_DIR = 'experiments/'


def _save_points_to_file(polygon, filename):
    with open(filename, 'w') as f:
        for point in polygon:
            print(f'{point[0]}, {point[1]}', file=f)


def _read_points_from_file(filename: str):
    with open(filename, 'r') as f:
        return list(map(lambda line: tuple(map(float, line.split(','))), filter(lambda x: x, f.readlines())))


def save_config(directory, json_data):
    directory = EXPERIMENTS_DIR + directory
    print(json_data)
    if not os.path.exists(directory):
        os.mkdir(directory)

    config_filename = directory + '/' + CONFIG_JSON_FILENAME
    config_file_data = {key: value for key, value in json_data.items()}
    with open(config_filename, 'w') as f:
        json.dump(config_file_data, f)


def read_config(directory) -> dict:
    filename = EXPERIMENTS_DIR + '/' + directory.rstrip('/') + '/' + CONFIG_JSON_FILENAME
    if not os.path.exists(filename):
        return dict()
    with open(filename, 'r') as f:
        data = json.load(f)
        return data




def save_polygon(fly_zone, no_fly_zones, start_point, directory):
    # Make an empty directory <project_name>/polygon or remove all the old files from there
    if not os.path.exists(directory):
        os.mkdir(directory)
    polygon_dir = directory + '/polygon'
    if os.path.exists(polygon_dir):
        for file in os.listdir(polygon_dir):
            os.remove(polygon_dir + '/' + file)
    else:
        os.mkdir(polygon_dir)
    _save_points_to_file(fly_zone, polygon_dir + '/' + FLY_ZONE_FILENAME)
    _save_points_to_file([start_point], polygon_dir + '/' + START_POINT_FILENAME)
    for i in range(len(no_fly_zones)):
        _save_points_to_file(no_fly_zones[i], polygon_dir + '/' + NO_FLY_ZONES_FILENAME_TEMPLATE.format(i))


def save_paths(paths, directory, paths_directory):
    if not os.path.exists(directory):
        os.mkdir(directory)

    if not paths_directory:
        i = 0
        while os.path.exists(new_dir := f"{directory}/{PATHS_FOLDER_PREFIX}{i}"):
            i += 1
            paths_directory = new_dir
    else:
        paths_directory = f"{directory}/{paths_directory}"

    if os.path.exists(paths_directory):
        for file in os.listdir(paths_directory):
            os.remove(f'{paths_directory}/{file}')

    os.makedirs(paths_directory, exist_ok=True)
    for i in range(len(paths)):
        _save_points_to_file(paths[i], f"{paths_directory}/path_{i}.csv")
        i += 1


def load_polygons_from_dir(directory: str):
    directory = EXPERIMENTS_DIR + directory.rstrip('/') + '/polygon/'
    if not os.path.exists(directory):
        return "Invalid directory path", 500
    files = os.listdir(directory)
    if FLY_ZONE_FILENAME not in files:
        return "No fly zone file found in the directory", 500
    if START_POINT_FILENAME not in files:
        return "No start point found in the directory", 500
    fly_zone = _read_points_from_file(directory + FLY_ZONE_FILENAME)
    start_point = _read_points_from_file(directory + START_POINT_FILENAME)[0]

    no_fly_zones = []
    i = 0
    while os.path.exists(filename := (directory + NO_FLY_ZONES_FILENAME_TEMPLATE.format(i))):
        no_fly_zones.append(_read_points_from_file(filename))
        i += 1

    return {'fly-zone': fly_zone, 'no-fly-zones': no_fly_zones, 'start-point': start_point}





