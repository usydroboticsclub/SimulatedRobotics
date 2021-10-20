# OpenCV in Gazebo

In this tutorial, we'll be integrating OpenCV into our environment so we can simulate a camera sensor.

1. Open a terminal / cmd, and `cd ` into the `3_ball_sorter_robot` folder. 
3. Run `docker build -f Dockerfile.build . -t usrc_tutorial_3_image`.
    - You will need to have built the image from tutorial 2 to do this step.
4. Run the container, using `docker run --rm --name=usrc_gazebo_container -p 5900:5900 -it usrc_tutorial_3_image:latest`.
    - You will likely need to run `docker container rm usrc_gazebo_container` to clean out the previous container.
5. Open a VNC, then in DOCKER, open a shell and run `roslaunch ball_sorter_robot spawnBallSorter.launch`.
    - You will have to pan/zoom the camera a lot usng the left and right mouse buttons.
    - You should see a ball sorter, and blue and green balls being spawned periodically, rolling down the ramp.
6. In Docker, run `roslaunch ball_sorter_robot spawnSorterControl.launch`, 

# Exercises
- Find and edit the following values to see what happens:
    - The size of the ball
    - The force that the controller applies
    - The location of the camera
- Edit the `ball_sorter.urdf` file so that the camera is closer to the ball spawn point. Does it affect the way the system behaves? 
- Edit the `scripts/ball_spawner.launch` to spawn four ball colours (e.g. purple, yellow, green, blue - red is dangerous), and modify the code so that two colours go in one bin and two go in the other bin.

## USRC Weekly Workshops
- Send us a picture of all the blue balls in the blue tray and all the green balls in the green tray.

## Challenges (in order of difficulty)
- CHALLENGE: Create a Surface-to-Air projectile interceptor, complete with URDF file, spawner file, launch file and camera system. Put your result in the `3_camera_SAM` branch.
- CHALLENGE: Create a bot that takes a panorama of its surroundings. Put your result in the `3_camera_panorama` branch.
- CHALLENGE: Create a robot with a camera attached to a movable claw, that grabs blue balls within an area underneath it and puts them into a box. Put your result in the `3_camera_claw` branch.

# Notes
## XACRO files
Xacro files are files that can be parsed into urdf files. They allow you to define some macros, making it easier for you to change your urdf more easily when you need to.


rosrun gazebo_ros spawn_model -z 10.0 -file /root/catkin_ws/src/ball_sorter_robot/urdf/robot.urdf -urdf -model rb

rostopic pub -1 /ball_sorter/joint1_position_controller/command std_msgs/Float64 "data: 0"

