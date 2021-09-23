# Ros and Robot control
1. Open a terminal / cmd, and `cd ` into the `2_keyboard_control_arm` folder. 
3. Run `docker build -f Dockerfile.build . -t usrc_tutorial_2_image`.
    - You will need to have built the previous container to do this step.
4. Run the container, using `docker run --name=usrc_gazebo_container -p 5900:5900 -it usrc_tutorial_2_image:latest`.
    - You will likely need to run `docker container rm usrc_gazebo_container` to clean out the previous container.
5. In DOCKER, open a shell and run `roslaunch usrc_tutorial spawnBowling.launch`.

ROS control loop -- to control an existing robot
- robot state publisher
- Gazebo interface
- Python file, ros nodes
- 

## Resources
http://wiki.ros.org/urdf/Tutorials

rosrun gazebo_ros spawn_model -z 10.0 -file /root/catkin_ws/src/keyboard_control_arm/urdf/beam_balance.urdf -urdf -model KCA
rosrun gazebo_ros spawn_model -z 12.0 -x 1 -file /root/catkin_ws/src/usrc_tutorial/urdf/bowling.urdf -urdf -model BP
docker cp keyboard_control_arm/urdf/beam_balance.urdf usrc_gazebo_container:/root/catkin_ws/src/keyboard_control_arm/urdf/beam_balance.urdf
check_urdf /root/catkin_ws/src/keyboard_control_arm/urdf/beam_balance.urdf