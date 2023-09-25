#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

# Initialize the node called move_robot_publisher to be a 
# publisher node to the topic /cmd_vel

rospy.init_node('move_robot_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(2) # 2hz
move = Twist() # create a Twist message object
move.linear.x = 0.5
move.angular.z = 10

while not rospy.is_shutdown():
    pub.publish(move)
    rate.sleep()