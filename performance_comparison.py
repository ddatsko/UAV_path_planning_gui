import typing

import own_planner
import popcorn_planner
import gtsp_planner
import optimized_darp_planner
from utils import get_path_properties
from math import pi
from data_storage import read_config, load_polygons_from_dir
import os
import pandas as pd
import time
import sys
import yaml
import json
import re

EXPERIMENTS_LOG_FILE = 'experiments.csv'
EXPERIMENTS_LOG_FILE_HEADER = ['algorithm', 'energy', 'path_time', 'path_length', 'n_turns', 'computation_time',
                               'max_energy', 'min_energy', 'n_paths']
EXPERIMENT_NAME_TO_FUNCTION = {
    'own': own_planner.plan_paths_own,
    'gtsp': gtsp_planner.plan_path_gtsp,
    'popcorn': popcorn_planner.plan_paths_wadl,
    'darp': optimized_darp_planner.plan_optimized_darp_paths
}


def _create_experiments_log_file(filename):
    if os.path.exists(filename):
        return
    open(filename, 'w').write(','.join(EXPERIMENTS_LOG_FILE_HEADER) + '\n')


def get_total_paths_metrics(paths):
    energy = time = length = num_of_turns = 0.0
    min_energy = float('inf')
    max_energy = -min_energy
    # print(f"Paths: {paths}")

    for path in paths:
        path_energy, path_time, path_length, turns = get_path_properties(path)
        energy += path_energy
        time += path_time
        length += path_length
        num_of_turns += turns
        min_energy = min(min_energy, path_energy)
        max_energy = max(max_energy, path_energy)

    print({'energy': energy, 'min_energy': min_energy, 'max_energy': max_energy, 'n_paths': len(paths),
            'path_time': time, 'path_length': length, 'n_turns': num_of_turns})
    return {'energy': energy, 'min_energy': min_energy, 'max_energy': max_energy, 'n_paths': len(paths),
            'path_time': time, 'path_length': length, 'n_turns': num_of_turns}


def test_algorithm_many_times(json_data, algorithm: typing.Callable, start_angle, angle_step):
    angle = start_angle
    best_res = {}
    best_paths = []
    while angle < pi:
        print(f"Testing with angle {angle}")
        json_data['init-rotation'] = angle
        print(f"Testing on angle {angle}")
        paths, metrics = run_one_algorithm(json_data, algorithm)
        print("Energy:", metrics['energy'])
        # TODO: maybe, move the criterion on which best result is selected to some user-set parameter
        if not best_res or metrics['max_energy'] < best_res['max_energy']:
            best_paths = paths
            best_res = metrics
        angle += angle_step
    return best_paths, best_res


def run_one_algorithm(json_data, algorithm: typing.Callable) -> (list, dict):
    start = time.time_ns()
    paths = algorithm(json_data)
    end = time.time_ns()
    total_path_metrics = get_total_paths_metrics(paths)
    total_path_metrics['computation_time'] = end - start

    return paths, total_path_metrics


def compare_algorithms(config):
    _create_experiments_log_file(config['output_file'])
    df = pd.read_csv(config['output_file'])

    for experiment in config['experiments']:
        for algorithm in config['algorithms']:
            if algorithm == 'gtsp':
                print(
                    f"\n\n\n Starting GTSP testing on experiment {experiment}. \nRestart the GTSP node with proper sweeping step and press return")
                input()

            print(f"Comparing algorithm {algorithm} on experiment {experiment}")
            json_data = read_config(experiment)

            algorithm_function = None
            for alg_name, alg_function in EXPERIMENT_NAME_TO_FUNCTION.items():
                if algorithm.startswith(alg_name):
                    algorithm_function = alg_function
                    break
            if not algorithm_function:
                print(f"Error. Algorithm name {algorithm} is unknown")
                exit(2)

            # Set the number of uavs if the algorithm name ends with a number
            if match := re.match(r'.*?(\d+)', algorithm):
                json_data['n-uavs'] = int(match.groups(0)[0])

            # exp_json_data = json_data
            exp_json_data = json_data.copy()
            if algorithm in config:
                print("Experiment in config")
                for key, value in config[algorithm].items():
                    if key in exp_json_data.keys():
                        print(f"Altering key {key} to {value}")
                        exp_json_data[key] = value

            if algorithm in config.keys() and 'start_angle' in config[algorithm] and 'angle_step' in config[algorithm]:
                paths, res = test_algorithm_many_times(exp_json_data, algorithm_function, config[algorithm]['start_angle'],
                                                       config[algorithm]['angle_step'])
            else:
                paths, res = run_one_algorithm(exp_json_data, algorithm_function)
            df = df.append({'experiment': experiment, 'algorithm': algorithm, **res}, ignore_index=True)

    df.to_csv(config['output_file'], index=False)


def main():
    if len(sys.argv) == 1:
        print("Error: too few arguments. Usage:\n python performance_comparison.py <config_file.yaml>")
        exit(1)

    with open(sys.argv[1], 'r') as f:
        config = yaml.safe_load(f)
    compare_algorithms(config)


if __name__ == '__main__':
    main()
