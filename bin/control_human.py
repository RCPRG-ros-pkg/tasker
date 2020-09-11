#!/usr/bin/env python  
import roslib
import rospy
import rospkg
import math
import tf
import turtlesim.msg
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from TaskER.srv import *
from tiago_msgs.msg import Command
global vel
vel = Twist()
def handle_actor_vel(msg):
    global vel
    vel = msg

if __name__ == '__main__':
    global vel
    vel = Twist()
    rospy.init_node('control_human',anonymous=True)
    actor_frame = rospy.get_param('~actor_frame')
    actor_name = rospy.get_param('~actor_name')
    actor_id = rospy.get_param('~actor_id')
    actor_gender = rospy.get_param('~actor_gender')
    human_transform = rospy.get_param('~actor_init_pose')
    actor_posture = rospy.set_param(actor_name+"/actor_posture", "stand")
    print "HT:"
    print human_transform
    rospy.Subscriber('/%s/vel' % actor_name,
                     Twist,
                     handle_actor_vel
                     )
    br = tf.TransformBroadcaster()
    marker_pub = rospy.Publisher("ellipse", Marker, queue_size=10)
    cmd_pub = rospy.Publisher("/rico_cmd", Command, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "human"
    marker.id = actor_id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    rospack = rospkg.RosPack()

    # list all packages, equivalent to rospack list
    rospack.list() 

    # get the file path for rospy_tutorials
    tasker_path = rospack.get_path('TaskER')
    if actor_gender == "male":
        marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male.dae"
    else:
        marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female.dae"
    marker.pose.position.x = human_transform[0]
    marker.pose.position.y = human_transform[1]
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.5
    marker.pose.orientation.y = 0.5
    marker.pose.orientation.z = 0.5
    marker.pose.orientation.w = 0.5
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.lifetime = rospy.Duration()

    marker_name = Marker()
    marker_name.header.frame_id = "/map"
    marker_name.header.stamp = rospy.Time.now()
    marker_name.ns = "names"
    marker_name.id = actor_id
    marker_name.type = Marker.TEXT_VIEW_FACING
    marker_name.action = Marker.ADD

    marker_name.pose.position.x = human_transform[0]
    marker_name.pose.position.y = human_transform[1]
    marker_name.pose.position.z = 2
    marker_name.pose.orientation.x = 0.0
    marker_name.pose.orientation.y = 0.0
    marker_name.pose.orientation.z = 0.0
    marker_name.pose.orientation.w = 1.0

    marker_name.text = actor_name

    marker_name.scale.z = 0.7

    marker_name.color.r = 0.0
    marker_name.color.g = 1.0
    marker_name.color.b = 0.0
    marker_name.color.a = 1.0

    marker_pub.publish(marker)
    marker_pub.publish(marker_name)
    task_reqested = False
    while not rospy.is_shutdown():
        actor_posture = rospy.get_param(actor_name+"/actor_posture")
        marker_pub.publish(marker)
        if actor_posture == "fell":
            fell_cmd = Command()
            fell_cmd.query_text=""
            fell_cmd.intent_name = "HF"
            fell_cmd.param_names = ["human_name"]
            fell_cmd.param_values = [actor_name]
            marker.ns = "human"
            marker.id = actor_id
            marker.action = Marker.MODIFY
            if actor_gender == "male":
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male_fell.dae"
            else:
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female_fell.dae"
            if task_reqested == False:
                cmd_pub.publish(fell_cmd)
                task_reqested= True
            marker.pose.position.x = human_transform[0]
            marker.pose.position.y = human_transform[1] 
            marker.pose.position.z = -0.5
            marker_name.pose.position.z = 1
            marker.pose.orientation.x = 0.5
            marker.pose.orientation.y = 0.5
            marker.pose.orientation.z = 0.5
            marker.pose.orientation.w = 0.5
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_name.color.a = 1.0
            marker_name.color.r = 1.0
            marker_name.color.g = 0.0
            marker_name.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_pub.publish(marker)
            marker_pub.publish(marker_name)
        elif actor_posture == "sit":
            marker.ns = "human"
            marker.id = actor_id
            marker.action = Marker.MODIFY
            if actor_gender == "male":
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male_sit2.dae"
            else:
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female_sit.dae"
            marker.pose.position.x = human_transform[0]
            marker.pose.position.y = human_transform[1] 
            marker.pose.position.z = -0.1
            marker_name.pose.position.z = 1.1
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.7071068 
            marker.pose.orientation.w = 0.7071068
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_name.color.a = 1.0
            marker_name.color.r = 0.0
            marker_name.color.g = 1.0
            marker_name.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_pub.publish(marker)
            marker_pub.publish(marker_name)
        else:
            actor_posture = rospy.set_param(actor_name+"/actor_posture", "stand")
            task_reqested = False
            human_transform[0] = human_transform[0] + math.cos(human_transform[2])*vel.linear.x/10+ math.sin(human_transform[2])*vel.linear.y/10
            human_transform[1] = human_transform[1] + math.cos(human_transform[2])*vel.linear.y/10+ math.sin(human_transform[2])*vel.linear.x/10
            human_transform[2] = human_transform[2] + vel.angular.z/10
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            br.sendTransform((human_transform[0], human_transform[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, human_transform[2]),
                         rospy.Time.now(),
                         actor_name,
                         "map")
            marker.ns = "human"
            marker.id = actor_id
            marker.action = Marker.MODIFY
            if actor_gender == "male":
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Male/male.dae"
            else:
                marker.mesh_resource = "file://"+tasker_path+"/makehuman/Female/female.dae"
            marker.pose.position.x = human_transform[0]
            marker.pose.position.y = human_transform[1]
            marker_name.pose.position.x = human_transform[0] 
            marker_name.pose.position.y = human_transform[1] 
            marker_name.pose.position.z = 2
            marker.pose.position.z = 0
            marker.pose.orientation.x= tf.transformations.quaternion_from_euler(1.54, 0, human_transform[2]+1.54)[0]
            marker.pose.orientation.y= tf.transformations.quaternion_from_euler(1.54, 0, human_transform[2]+1.54)[1]
            marker.pose.orientation.z= tf.transformations.quaternion_from_euler(1.54, 0, human_transform[2]+1.54)[2]
            marker.pose.orientation.w= tf.transformations.quaternion_from_euler(1.54, 0, human_transform[2]+1.54)[3]
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_name.color.a = 1.0
            marker_name.color.r = 0.0
            marker_name.color.g = 1.0
            marker_name.color.b = 0.0
            marker.lifetime = rospy.Duration()
            marker_pub.publish(marker)
            marker_pub.publish(marker_name)
        rospy.sleep(0.1)
