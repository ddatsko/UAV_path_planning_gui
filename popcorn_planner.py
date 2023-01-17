from wadl.survey import Survey
from wadl.solver.solver import SolverParameters
from wadl.lib.route import RouteParameters
from wadl.mission import MissionParameters, Mission
from flask import request
import tempfile
import json
import os
import shutil
import logging


CSV_FILENAME = '__polygon.csv'
SURVEY_NAME = '.__TEMP_SURVEY'


def plan_paths_wadl(json_data):
    """
    Plan paths using wadl and reading data from request.data
    :return:
    """
    logger = logging.getLogger()
    logger.setLevel("WARNING")

    fly_zone = json_data["fly-zone"]
    home_point = (json_data["start-point"][0], json_data["start-point"][1])

    with open(CSV_FILENAME, 'w') as f:
        print("fid,lat,lon", file=f)
        i = 1
        for p in fly_zone:
            print(f"{i},{float(p[0])},{float(p[1])}", file=f)
            i += 1

    survey = Survey(SURVEY_NAME)
    survey.setKeyPoints({'home': home_point})

    route_params = RouteParameters()
    route_params['limit'] = 60000
    route_params['speed'] = 9
    route_params['altitude'] = 20

    survey.addTask(CSV_FILENAME, step=float(json_data['sweeping-step']), home=['home'], routeParameters=route_params)

    solver_params = SolverParameters()
    solver_params['subgraph_size'] = 18
    solver_params['SATBound_offset'] = 4
    solver_params['timeout'] = 60

    survey.setSolverParamters(solver_params)

    os.remove(CSV_FILENAME)

    # Delete all the existing planned paths
    for p in filter(lambda x: x.startswith(CSV_FILENAME[:-4]), os.listdir(SURVEY_NAME)):
        shutil.rmtree(SURVEY_NAME + '/' + p)
    try:
        survey.plan()
    except Exception as e:
        pass


    os.sync()
    routes_path = SURVEY_NAME + '/' + list(filter(lambda x: x.startswith(CSV_FILENAME[:-4]), os.listdir(SURVEY_NAME)))[0] + '/routes/'
    routes = os.listdir(routes_path)

    res = []
    for route in routes:
        print("Route:", route)
        with open(routes_path + route, 'r') as f:
            res.append(list(map(lambda line: tuple(reversed(tuple(map(float, line.split(',')[:2])))), list(filter(lambda x: bool(x.strip()), f.readlines()))))[2:-2])
    return res

