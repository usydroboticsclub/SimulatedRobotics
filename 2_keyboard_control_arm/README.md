# Ros and Robot control
1. Open a terminal / cmd, and `cd ` into the `2_keyboard_control_arm` folder. 
3. Run `docker build -f Dockerfile.build . -t usrc_tutorial_2_image`.
    - You will need to have built the previous container to do this step.
4. Run the container, using `docker run --name=usrc_gazebo_container -p 5900:5900 -it usrc_tutorial_2_image:latest`.
    - You will likely need to run `docker container rm usrc_gazebo_container` to clean out the previous container.
5. In DOCKER, open a shell and run `roslaunch keyboard_control_arm spawnBeamBalance.launch`.
    - You should see an arm, and a ball being spawned and deleted at 10 second intervals.
5. In DOCKER, open a shell and run `roslaunch keyboard_control_arm spawnBeamController.launch`. Leave it running.
6. Finally, open one more shell (still in docker) and run `rostopic pub -1 /moveable_arm/joint1_position_controller/command std_msgs/Float64 "data: 0"`
    - The arm should try and hold itself up.

## Controlling the arm through a file
6. Next, run  `rosrun keyboard_control_arm ball_control.py`. You will be prompted with a control interface.
    - Try to balance the ball so it does not hit the ground.
7. Hit CTRL+C to end the ball_control. Then, run `rosrun keyboard_control_arm auto_ball_control.py`. The ball should automatically be balanced.

## Exercises
- Edit `auto_ball_control.py` so that the ball ends up centered at 4 units along the arm, instead of 3.5.
- Edit the `beam_balance.urdf` file so that the arm is twice as long.
- Read up on PID values, then edit the `config.yaml` file's `p,i,d` values so that the beam balance responds faster.
- Combine the `spawnBeamBalance.launch` file and `spawnBeamController.launch` files into a single file.

## Challenges (in order of difficulty)
Contribute to the club by submitting a pull request!

- CHALLENGE: Edit `ball_spawner.py` so that it respawns the ball when the ball goes off the arm, OR the ball has achieved the target position for 10 seconds straight.
- CHALLENGE: Write inline documentation for each line of the launch files, py files or urdf files. You don't have to do all of them - just pick a file. Something like: The following line does X. Without the following line, Y won't work. (Submit a Pull Request with your solutions!)
- CHALLENGE: Use dynamic_reconfigure (and its gui counterpart, rqt_reconfigure) to tune the arm PID so the arm isn't jerky. Submit a PR with a modified `Dockerfile.build` and how-to instructions.
    - CHALLENGE: Modify auto_ball_control.py so that it uses ros parameters and therefore can be controlled using dynamic_reconfigure. Submit a PR with the modified auto_ball_control.py code.
- CHALLENGE: Create a section in this readme that uses RQT to inspect the nodes and topics. Include a screenshot, and modify the `Dockerfile.build` so others can run the command.
- CHALLENGE: Write a version of `auto_ball_control.py` that bounces the ball, not just balances it. (Submit a Pull Request with your solutions!)
- CHALLENGE: Create a robot with a controllable forearm and an upper arm, using the `beam_balance.urdf` file as a guide. Also create a launch file, yaml file and controller launch file. (Submit a Pull Request with your solutions!)

## Live workshop proof-of-work
1. Download OBS screen recorder.
2. Record your bot balancing a ball.

## Resources
http://wiki.ros.org/urdf/Tutorials

## Useful commands
- `check_urdf /root/catkin_ws/src/keyboard_control_arm/urdf/beam_balance.urdf`
- `rosrun gazebo_ros spawn_model -z 10.0 -file /root/catkin_ws/src/keyboard_control_arm/urdf/beam_balance.urdf -urdf -model KCA`
- `docker cp keyboard_control_arm/urdf/beam_balance.urdf usrc_gazebo_container:/root/catkin_ws/src/keyboard_control_arm/urdf/beam_balance.urdf`