import rospy
from polygon_coverage_msgs.srv import PolygonServiceRequest, PolygonServiceResponse, PolygonService, \
    PlannerService, PlannerServiceResponse, PlannerServiceRequest
from utils import *
from geometry_msgs.msg import Polygon


def _load_gtsp_polygon(polygon_req) -> PolygonServiceResponse:
    rospy.wait_for_service('/coverage_planner/set_polygon')
    proxy = rospy.ServiceProxy('/coverage_planner/set_polygon', PolygonService)
    res = proxy(polygon_req)
    if not res.success:
        raise Exception("Failure while setting polygon")
    return res


def _start_gtsp_planner(planner_req):
    rospy.wait_for_service("/coverage_planner/plan_path")
    proxy = rospy.ServiceProxy("/coverage_planner/plan_path", PlannerService)
    res = proxy(planner_req)
    if not res.success:
        raise Exception("Could not plan paths")
    return res.sampled_plan


def plan_path_gtsp(json_data):
    polygon_req = PolygonServiceRequest()
    origin_x, origin_y = map(float, json_data["fly-zone"][0])

    # Transform all the points into meters and close to the origin point
    # fly_zone = [Point32(0, 0, 10), Point32(0, 100, 10), Point32(100, 100, 10), Point32(100, 0, 10), Point32(0, 0, 10)]
    # no_fly_zones = [Polygon([Point32(20, 20, 10), Point32(20, 60, 10), Point32(60, 60, 10), Point32(60, 20, 10)])]
    fly_zone = [meter_point_from_gps(float(x), float(y), origin_x, origin_y) for x, y in json_data["fly-zone"]]
    no_fly_zones = []
    for pol in json_data["no-fly-zones"]:
        no_fly_zones.append(Polygon())
        for x, y in pol[:-1]:
            no_fly_zones[-1].points.append(meter_point_from_gps(float(x), float(y), origin_x, origin_y))

    # Complete the request message
    polygon_req.polygon.polygon.hull.points = fly_zone[:-1]
    polygon_req.polygon.polygon.holes = no_fly_zones

    print("Planning...")
    # load points to the planning node
    _load_gtsp_polygon(polygon_req)
    print("Points loaded")

    planner_req = PlannerServiceRequest()
    planner_req.start_pose.pose.position = meter_point_from_gps(float(json_data["start-point"][0]),
                                                                float(json_data["start-point"][1]), origin_x, origin_y)
    planner_req.goal_pose = planner_req.start_pose

    print("Starting planner")
    planned_path = _start_gtsp_planner(planner_req)

    return [[gps_point_from_meters(p.transforms[0].translation.x, p.transforms[0].translation.y, origin_x, origin_y)
             for p in planned_path.points]]
