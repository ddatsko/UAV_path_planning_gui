from thesis_path_generator.srv import GeneratePaths, GeneratePathsRequest, GeneratePathsResponse
from geometry_msgs.msg import Point32, Polygon
import rospy

last_generated_paths = None

UNIQUE_ALTITUDE_STEP = 0


def _generate_drones_paths_ros(generate_req):
    rospy.wait_for_service("/generate_paths")

    proxy = rospy.ServiceProxy("/generate_paths", GeneratePaths)
    res = proxy(generate_req)

    global last_generated_paths
    last_generated_paths = res.paths_gps

    if not res.success:
        print("Unsuccessful service call")
        return []
    else:
        return [[(p.position.y, p.position.x) for p in path.points] for path in res.paths_gps]


def plan_paths_own(json_data):
    generate_req = GeneratePathsRequest()
    generate_req.fly_zone.points = [Point32(float(x), float(y), 0.0) for x, y in json_data["fly-zone"]]
    generate_req.no_fly_zones = [Polygon([Point32(float(x), float(y), 0.0) for x, y in pol]) for pol in
                                 json_data["no-fly-zones"]]
    generate_req.start_lat = float(json_data["start-point"][0])
    generate_req.start_lon = float(json_data["start-point"][1])
    generate_req.sweeping_step = float(json_data["sweeping-step"])
    generate_req.number_of_drones = int(json_data["n-uavs"])
    generate_req.rotations_per_cell = int(json_data["rotations-per-cell"])
    generate_req.decomposition_rotation = float(json_data["init-rotation"])
    generate_req.drones_altitude = int(json_data["altitude"])
    generate_req.unique_altitude_step = 0
    generate_req.decomposition_method = 1
    generate_req.wall_distance = float(json_data['sweeping-step']) / 2
    generate_req.min_sub_polygons_per_uav = int(json_data['min-pieces-per-uav'])

    # TODO: move this to JS interface
    generate_req.end_point_x_difference = 0

    # TODO: Move this to JS interface too
    generate_req.no_improvement_cycles_before_stop = 400
    generate_req.override_drone_parameters = bool(json_data["override-drone-spec"])
    if generate_req.override_drone_parameters:
        generate_req.drone_area = float(json_data["uav-area"] or 0)
        generate_req.propeller_radius = float(json_data["propeller-radius"] or 0)
        generate_req.number_of_propellers = int(json_data["n-propellers"] or 0)
        generate_req.drone_mass = float(json_data["uav-mass"] or 0)
        generate_req.average_acceleration = float(json_data["acceleration"])
    generate_req.override_battery_model = bool(json_data["override-battery-model"])
    if generate_req.override_battery_model:
        generate_req.battery_cell_capacity = float(json_data["cell-capacity"] or 0)
        generate_req.battery_number_of_cells = int(json_data["cells-in-series"] or 0)
    res = _generate_drones_paths_ros(generate_req)

    return res


def get_last_own_paths():
    return last_generated_paths
