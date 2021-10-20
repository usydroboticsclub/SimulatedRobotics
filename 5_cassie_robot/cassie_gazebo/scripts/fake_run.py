#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


rospy.init_node('running_bot', anonymous=True)
cvBridge = CvBridge()

left_pub = rospy.Publisher('/cassie/knee_joint_left_joint_position_controller/command', Float64, queue_size=10)
right_pub = rospy.Publisher('/cassie/knee_joint_right_joint_position_controller/command', Float64, queue_size=10)

rate = rospy.Rate(3)  # 1hz
commandPosition=0
while not rospy.is_shutdown():
    commandPosition = 1-commandPosition
    left_pub.publish(commandPosition)
    right_pub.publish(1-commandPosition)
    rate.sleep()
