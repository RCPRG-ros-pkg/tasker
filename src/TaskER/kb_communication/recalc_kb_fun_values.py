from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pl_nouns.dictionary_client import DisctionaryServiceClient
from math import sqrt
import tiago_kb.places_xml as kb_p
import tf
import rospy

def recalc_kb_fun_values(da_type, args, cfg_actions, tf, executable_tasks):
    # goal = get_task_params(da_type, args, cfg_actions, executable_tasks)
    goal = ''
    goals = []
    if da_type == "clean":
        for i in range(len(args)):
            if args[i] == "m1":
                goals.append(args[i+1])
            if args[i] == "m2":
                goals.append(args[i+1])
            if args[i] == "m3":
                goals.append(args[i+1])
    else:
        for i in range(len(args)):
            if args[i] == "miejsce":
                goal = args[i+1]
                break

    move_goal = None
    move_goals = None
    locations_cfg = None
    tasks_cfg = cfg_actions["tasks"]

    for task_cfg in tasks_cfg:
        if task_cfg["name"] == da_type:
            locations_cfg = task_cfg["locations"]
            break
    
    if da_type == "clean":
        distances = []
        for goal in goals:
            for location_cfg in locations_cfg:
                if location_cfg["name"] == goal:
                    move_goal = location_cfg["position"]
                    break
            if move_goal is None:
                print("Given location is not defined in the config file, terminating.")
                raise
            else:
                distance = calculate_distance(move_goal, tf)
                distances.append(distance)
        return distances

    else:
        for location_cfg in locations_cfg:
            if location_cfg["name"] == goal:
                move_goal = location_cfg["position"]
                break
        if move_goal is None:
            print("Given location is not defined in the config file, terminating.")
            raise
        else:
            distance = calculate_distance(move_goal, tf)
            return distance

def get_transform():
    ros_time = rospy.Time()
    trans = []
    rot = []
    rate = rospy.Rate(5.0)
    listener = tf.TransformListener()
    rospy.sleep(2)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('/map', '/base_link', ros_time, rospy.Duration(8.0))
            (trans,rot) = listener.lookupTransform('/map', '/base_link', ros_time)
            if len(trans) != 0:
                return trans
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            continue

def calculate_distance(goal, robot_tf):
    end_point = (goal['target_pose.pose.position.x'], goal['target_pose.pose.position.y'])
    start_point = (robot_tf[0], robot_tf[1])
    distance = sqrt((end_point[0]-start_point[0])**2 + (end_point[1]-start_point[1])**2)
    return distance


# rostopic pub /rico_cmd tiago_msgs/Command "query_text: ''
# intent_name: 'RS'
# param_names: ['miejsce']
# param_values: ['warsztat']
# confidence: 0.0
# response_text: ''"

# def get_task_params(da_type, args, cfg_actions, executable_tasks):
#     if da_type in executable_tasks:
#         places_xml_filename = rospy.get_param('/kb_places_xml')
#         # print 'Reading KB for places from file "' + places_xml_filename + '"'
#         kb_places = kb_p.PlacesXmlParser(places_xml_filename).getKB()
#         dictionary = DisctionaryServiceClient()
#         location = args['miejsce']
#         location_m = dictionary.getCases(location).getCase('mianownik')
#         mc_name = 'sim'
#         pl = kb_places.getPlaceByName(location_m, mc_name)
#         location_name = pl.getName() 
#         move_new_goal = None
#         tasks_config = cfg_actions['tasks']
#         for task_config in tasks_config:
#             if task_config['name'] == 'move_new':
#                 locations = task_config['locations']
#                 for location in locations:
#                     if location['name'] == location_name:
#                         move_new_goal = location['position']
#                         return move_new_goal