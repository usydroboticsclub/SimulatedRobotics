# ROS Integration

In this tutorial, we'll be using an existing simulated robotics platform (turtlebot3) to examine how ROS packages can be reused.

1. Open a terminal / cmd, and `cd ` into the `4_wheeled_robot_waypoint` folder. 
3. Run `docker build -f Dockerfile.build . -t usrc_tutorial_4_image`.
    - You will need to have built the image from tutorial 1 to do this step.
4. Run the container, using `docker run --name=usrc_gazebo_container -p 5900:5900 -it usrc_tutorial_4_image:latest`.
    - You will likely need to run `docker container rm usrc_gazebo_container` to clean out the previous container.
5. In DOCKER, open a shell and run `roslaunch turtlebot3_world + the lidar.launch`.
    - You should see a small robot in a playing field. 
5. In DOCKER, open a shell and run `rviz the .rviz file`. You should see a new window pop up called RVIZ, and you should see the robot using its sensor to pick up data.
6. In the RVIZ window, uncheck the MAP layer. You should see the black and white overlay on the right hand side removed and replaced with just the green dots. This is the raw laser scanner data. Some of the other visualizations aren't in use yet.
7. Open a new shell, and run `roslaunch the waypoint follower`. Create a marker, and watch the robot reach the marker. -- trail here too pls
8. In yet another shell, run `rqt_graph`. Near the top, there will be a drop-down box that says `Nodes only` - change it to `nodes and topics`. Each oval is a node and each rectangle is a topic. Nodes can listen to and publish into topics. 
9. Run the `camera node`, then edit RVIZ so that it shows the camera.
10. Stop the `waypoint follower node`. Run `rostopic publish cmd vel`. You should see the robot move forward. (If the robot is in a corner, give it the opposite velocity to make it back out).
11. Run `rospack list`. This shows you all the packages you've been using today - all work that other people have done for us! This is the power of the ROS community :)

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