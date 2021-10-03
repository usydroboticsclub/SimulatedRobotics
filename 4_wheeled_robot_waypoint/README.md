# ROS Integration

In this tutorial, we'll be using an existing simulated robotics platform (turtlebot3) to examine how ROS packages can be reused.

1. Open a terminal / cmd, and `cd ` into the `4_wheeled_robot_waypoint` folder. 
3. Run `docker build -f Dockerfile.build . -t usrc_tutorial_4_image`.
    - You will need to have built the image from tutorial 1 to do this step.
4. Run the container, using `docker run --name=usrc_gazebo_container -p 5900:5900 -it usrc_tutorial_4_image:latest`.
    - You will likely need to run `docker container rm usrc_gazebo_container` to clean out the previous container.
5. In DOCKER, open a shell and run `roslaunch turtlebot3_gazebo turtlebot3_world.launch`.
    - You should see a small robot in a playing field. 
6. Open another shell and run `roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`. 
    - You should see a window showing a laser scan and a map. You can toggle each layer on the left to see the different components.
7. Next, again in a new shell, run `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`, and wait for the terminal to give you keyboard control. Play around with the robot a bit.
8. Stop your bot by pressing S, then CTRL-C to stop the keyboard controller.
9. Also stop your laser scan terminal from step 6.
9. Run `rostopic pub -1 cmd_vel geometry_msgs/Twist -- '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.3]'`. You may have to move your bot a bit if it has crashed into a wall. It should turn a bit.
    - Can you figure out a command to get it to stop?
10. Run `roslaunch turtlebot3_navigation turtlebot3_navigation.launch`. 
    - You should see a ghost map.
11. Align the nagivation map with your robot position, using the green '2D Pose Estimate' button at the top of the screen. 
    - Click the button, then click and drag the arrow on the map to move your bot. You can make multiple attempts.
    - Try and get your bot in the right position first, then click on your bot exactly to start the arrow and set the angle.
12. Create a 2D Nav Goal using the red arrow next to the green arrow. Try and make it go around a corner!
13. Run `rospack list`. This shows you all the packages you've been using today - all work that other people have done for us! This is the power of the ROS community :)

# Exercises
- Create a `spiral.py` file that publishes to the `cmd_vel` topic to make the robot move in a spiral. (You may need to do the ROS tutorials to build up the competencies in order to do this.)  Create a branch `4_spiral_bot` and submit a pull request.

## USRC Weekly Workshops
- Send us a picture of your rqt_graph output.

## Challenges 
- Create a `stop.py` file that counts the number of red pixels on screen and makes the robot stop if there are more than 50% red pixels. Then, make a `.launch` file to test it. Create a branch `4_stop_at_red` and submit a pull request.
- CASSIE is a bipedal research robot. Its URDF files are here: https://github.com/UMich-BipedLab/Cassie_Model. Create a simulation that has a walking CASSIE robot. Create a branch `4_cassie` and submit a pull request.
- The ATHLETE (All-Terrain Hex-Limbed Extra-Terrestrial Explorer) is a robot with 6 legs with wheels, developed by JPL. Its URDF files are here: https://github.com/gkjohnson/urdf-loaders. Create a simulation with a ATHLETE robot. Create a branch `4_athlete` and submit a pull request.
- BAXTER was designed to be a coworking robot. Its URDF files are here: https://github.com/RethinkRobotics/baxter_common. 
- SPOT the robot dog: https://github.com/clearpathrobotics/spot_ros
- DARPA Robotics Challenge: https://github.com/osrf/drcsim
- A Prius: https://github.com/osrf/car_demo
- Figure out Visual SLAM using the ORB_SLAM2 ros package.
- Figure out Visual SLAM + Object Recognition.

If you do any of the above, please share with us! 