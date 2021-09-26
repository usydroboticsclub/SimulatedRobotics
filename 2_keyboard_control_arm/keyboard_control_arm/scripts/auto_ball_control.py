#!/usr/bin/python

from os import error
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

# Ball monitor
ballX = 0
ballY = 0
ballZ = 0
ballDespawnCountdown=2


def positionCallback(data):
    global ballX
    global ballY
    global ballZ
    global ballDespawnCountdown
    ballDespawned=True # assume despawned
    for (i, name) in enumerate(data.name):
        if name == "sphere":
            ballX = data.pose[i].position.x
            ballY = data.pose[i].position.y
            ballZ = data.pose[i].position.z
            ballDespawned=False
    if ballDespawned:
        ballDespawnCountdown=2


if __name__ == '__main__':
    # Initialize ROS
    rospy.init_node('ball_keyboard_control', anonymous=True)

    rospy.Subscriber("/gazebo/model_states", ModelStates, positionCallback)

    pub = rospy.Publisher(
        '/moveable_arm/joint1_position_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    commandPosition = 0

    commandP = 0.3
    commandD = 2
    commandDMem = 0

    while not rospy.is_shutdown():
        err = 4.5 - ballY
        if ballY > 7 or ballY < 0 or ballDespawnCountdown>0:
            # Give up, reset
            commandPosition=0
            ballDespawnCountdown -= 1
        else:
            # Calculate PID
            commandPosition = commandP * err

            commandPosition = commandPosition - commandD * (commandDMem - err)
        
        commandDMem = err
        print (commandPosition)
        pub.publish(commandPosition)
        rate.sleep()
