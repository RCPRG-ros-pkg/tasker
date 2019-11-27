#!/usr/bin/env python  
import roslib
import rospy

import tf
import turtlesim.msg
from geometry_msgs.msg import Twist

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
    human_transform = rospy.get_param('~actor_init_pose')
    print "HT:"
    print human_transform
    rospy.Subscriber('/%s/vel' % actor_frame,
                     Twist,
                     handle_actor_vel
                     )
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
    	human_transform[0] = human_transform[0] + vel.linear.x/10
    	human_transform[1] = human_transform[1] + vel.linear.y/10
    	vel.linear.x = 0
    	vel.linear.y = 0
    	br.sendTransform((human_transform[0], human_transform[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     actor_frame,
                     "map")
    	rospy.Rate(10)