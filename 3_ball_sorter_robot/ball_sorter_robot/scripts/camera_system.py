#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

"""
Uncomment each section slowly, then use 
`rosrun ball_sorter_robot camera_system.py` to run the code.
"""

"""
First, some setup stuff.
"""
rospy.init_node('ball_keyboard_control', anonymous=True)
cvBridge = CvBridge()

"""
We will create a camrea that looks at a specific area on the screen, 
and displays the image on screen.
"""
ball_color = "green"

def image_callback(data):
    global ball_color
    # Decode and display the image
    cv_image = cvBridge.imgmsg_to_cv2(data, "bgr8")


    # Detect whether it is a green ball or a red ball

    lower_green = np.array([50,100,100])
    upper_green = np.array([106,255,255])

    lower_blue = np.array([110,100,100])
    upper_blue = np.array([130,255,255])

    frame = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

    blue_mask = cv2.inRange(frame, lower_blue, upper_blue)
    green_mask = cv2.inRange(frame, lower_green, upper_green)
    if np.sum(blue_mask)>cv_image.shape[0] * cv_image.shape[1]/4:
        ball_color="blue"
    if np.sum(green_mask)>cv_image.shape[0] * cv_image.shape[1]/4:
        ball_color="green"

    cv2.putText(cv_image,ball_color, (20, 100), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255))
    print (np.sum(green_mask))
    print (np.sum(blue_mask))
    print (cv_image.shape[0] * cv_image.shape[1]/4)
    cv2.imshow("camera_see", cv_image)
    cv2.imshow("b_mask", blue_mask)
    cv2.imshow("g_mask", green_mask)
    cv2.waitKey(2)

    # Tilt the sorter in the correct direction
    if ball_color == "green":
        commandPosition=-0.4
    else:
        commandPosition=0.4
    arm_pub.publish(commandPosition)

image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, image_callback)
arm_pub = rospy.Publisher('/ball_sorter/joint1_position_controller/command', Float64, queue_size=10)
while not rospy.is_shutdown():
    rospy.spin()

