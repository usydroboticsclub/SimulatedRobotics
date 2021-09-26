#!/usr/bin/python

import rospy
import curses
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState


needsAck = False
commandPosition = 0
lastCommandPosition = 0
def callback(data):
    global needsAck
    global commandPosition
    global lastCommandPosition
    if lastCommandPosition != data.set_point:
        if needsAck:
            print "OK\r"
            needsAck=False
        lastCommandPosition = data.set_point 


if __name__ == '__main__':
    try:
        # Enable single character io
        stdscr = curses.initscr()
        curses.noecho()
        stdscr.keypad(1)

        # Initialize ROS
        rospy.init_node('ball_keyboard_control', anonymous=True)
        
        rospy.Subscriber("/moveable_arm/joint1_position_controller/state", JointControllerState, callback)
        pub = rospy.Publisher('/moveable_arm/joint1_position_controller/command', Float64, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        print ("Press Up Arrow to increase angle; Down Arrow to decrease angle.")
        while not rospy.is_shutdown():
            if needsAck:
                print ".",
            else:
                # get a key press
                c = stdscr.getch()
                if c==curses.KEY_DOWN:
                    commandPosition = commandPosition + 0.01
                elif c == curses.KEY_UP:
                    commandPosition = commandPosition - 0.01
                pub.publish(commandPosition)
                print "Publishing %f..." % commandPosition,
                needsAck=True
            rate.sleep()
    finally:
        curses.nocbreak()
        stdscr.keypad(0)
        curses.echo()
        curses.endwin()

