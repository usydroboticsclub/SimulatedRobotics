# Installing the simulation environment

The simulation environment uses docker and gazebo:

* Simulation (runs on )
* Gazebo (runs on)
* Docker (runs on)
* Your operating system (runs on)
* Your processor

Normally you're only running the bottom two layers, so with this amount of layers, the simulation might be a bit slow! Be patient.

## What to do
### Installing everything
1. Download Docker: https://www.docker.com/products/docker-desktop
    - The community edition will be fine, no need to pay for it :D
    - If you're on windows, you should also activate WSL, by:
        - going to start > type in CMD > right click, run as administrator, 
        - and then `wsl --install`. 
        - More info here: [https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers](https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-containers)
2. Open a terminal / cmd, and `cd ` into the `1_docker_gazebo` folder. 
3. Run `docker build -f Dockerfile.build . -t gazebo_image`. This creates a docker image called `gazebo_image`. 
4. Run the container, using `docker run --name=usrc_gazebo_container -p 5900:5900 -it gazebo_image:latest`. This creates a container from the docker image.
    - You'll see a bunch of text come up. That's normal, don't worry. 
    - The command window should stick around, that's also normal.
    - ... unless you see something like `docker: Error response from daemon: Conflict. The container name "/usrc_gazebo_container" is already in use by container`. In that case, run `docker container rm usrc_gazebo_container` then try again.
5. Install Vnc Viewer from https://www.realvnc.com/en/connect/download/viewer/. (It's free!)
6. Open VNC viewer. In the top bar, type `localhost:5900`.
    - Ignore the error message about encrypted connection that comes up.
    - VNC will likely crash at some point during the tutorial. If it does, go back to your `docker run` shell and hit enter a few times, and then type `/openvnc.sh` again to get the vnc back. If you don't do this, you'll get something like `Failed to connect to the container`.
7. You should see a screen with an ubuntu system inside it.
8. Right click anywhere on the screen. Go to Applications > Shells > Bash.
    - You may have to wait a few seconds for the system to start up if you can't right click immediately.
9. An Inside Terminal should show up. In the inside terminal, run `gazebo`.

### Messing with gazebo
1. A blank screen with some grid lines should show up. In the top-centre of the screen, there is a cube. Click the cube, then click anywhere in the main screen, and it will spawn a cube.
2. Spawn another cube, so you have two cubes.
3. Pan around the screen by left-click and dragging.
4. Zoom in and out by right-click and dragging.
5. If you've got a mouse, use middle click + drag to rotate the view.
6. In the top-left of the screen next to the mouse icon, there is a quad-arrow. Click the quad-arrow, then click a cube. A blue, green and red arrow set should show up.
7. Click the head (>) of the blue arrow and drag it in the arrow direction. This will lift the cube. Release the blue arrow, and your cube should drop to the ground.
8. Press the pause at the bottom of the screen to pause the simulation.
9. Move the cube so that it is stacked on top of the other cube, but on an edge, so it will fall over.
10. Press play at the bottom of the screen, and watch the cube fall over.

### Spawning things from the command line using ROS
Spawning cubes is fun and all, but two disadvantages are clear: first, you can only spawn cubes, and second, you have to use your pesky mouse to click on the screen. What if there's no screen? What if you want to spawn 100 cubes at a time? We must find another solution!

Let's use a system called ROS to control gazebo, and tell gazebo to spawn models from a command line (and then eventually a script!)
1. We've actually already installed ROS on this system in the `Dockerfile.build`, so you can use it straight away.
2. Close the gazebo window in your DOCKER.
3. In your DOCKER terminal, run `roslaunch usrc_tutorial spawnBowling.launch`.
    - Pasting the normal way won't work in the docker terminal. Instead, use `shift + insert` to paste (The shortcut may be different for mac).
    - You should see a gazebo window with a single cylinder appear. Let's add some more!
4. Open a new terminal inside DOCKER. You may want to reduce the size of your gazebo window.
5. Run `rosrun gazebo_ros spawn_model -z 10.0 -file /root/catkin_ws/src/usrc_tutorial/urdf/bowling.urdf -urdf -model cylinder2`. You should see another cylinder fall from the sky and land on your first cylinder.

If you're unacquainted with ROS, then saying 'ROS is a way to spawn things in Gazebo' is like saying 'A swiss army knife is for spreading butter': while technically true, there are plenty of things a swiss army knife can do aside from spreading butter, and if you paid a pretty penny to get a swiss army knife, then you might as well used all the bells and whistles on it! We'll be seeing ROS much more in this course, but if you're extra keen, you can check out the ROS tutorials here: [http://wiki.ros.org/ROS/Tutorials]

### Editing the world
Next, we're going to edit the world launch file to set up our bowling arena. 
1. Open `spawnBowling.launch` and copy the last few lines (starting with `<node>` and then the one after it) back into the `spawnBowling.launch` a few times.
2. Change the `name=urdf_pin1`, `-model pin1` name and the `-x`, `-y` and `-z` numbers for each copy to make a nice bowling pattern. Each line must have a different model name, and a different node name.
3. Move the new `spawnBowling.launch` file into your docker container with the following command (run from a NEW, HOST terminal, NOT THE ONE you ran the DOCKER RUN): `docker cp spawnBowling.launch usrc_gazebo_container:/root/catkin_ws/src/usrc_tutorial/launch/spawnBowling.launch`
4. Hit CTRL-C on your old DOCKER terminal that was running the `roslaunch usrc_tutorial spawnBowling.launch`. This will take some time to stop fully, be patient.
5. Run `roslaunch usrc_tutorial spawnBowling.launch` in that same terminal again.

### Go bowling!
1. Create a sphere in the world, the same way you created the cube in the start of this tutorial.
2. This time, instead of using the arrows to move your sphere, you will be adding a force.
    - On the left hand side of gazebo, there will be a 'world' tab. Under Models, right click 'unit_sphere', then click 'Add Force/Torque'.
    - Set the Y-force to 100N and then press 'Apply Force'.
    - Have fun!

### Shut it down
On the original window that you ran `docker run` in, type in `exit` and hit enter.

# Exercises
- Edit the `spawnBowling.launch` file so a 10-pin bowling setup is created. Then use a sphere to knock it over!
- Modify the `bowling.urdf` file so that spawning bowling objects spawns rectangular prisms.
- Try some of the tutorials below!

## USRC Weekly workshops
- Create a 10 pin bowling set, either manually or using the urdf file. Then, take a screenshot and upload it to our Discord with your SID and name.

## Tutorials
These tutorials are a lot more involved but a lot more in depth. Many of them don't use docker, so just open a docker machine using `docker run --name=usrc_gazebo_container -p 5900:5900 -it gazebo_image:latest` to get access to a shell.

- Using your docker container, try the ROS tutorials: http://wiki.ros.org/ROS/Tutorials
    - You can skip the first tutorial (Installing and configuring your ROS environment) as it requires an ubuntu machine - instead start on the second tutorial by running the command from before and using a shell in that environment.
- Also consider some of these Gazebo tutorials: http://gazebosim.org/tutorials
- Also try the Docker tutorials: https://www.docker.com/101-tutorial
# Notes
## Docker, explained
Operating systems have specific quirks. E.g. Windows programs won't run on Mac.

Since Mac and Windows became a thing, people have been asking: Why not make a level playing field where you can build an app that just _runs_ on any computer?

Various solutions were devised:
- Virtual machines: Spin up another entire machine on your base machine. Advantage: Can run Mac apps on Windows, Windows apps on mac, without re-coding the apps. Disadvantage: Have to get an entire Windows 'virtual hard drive' that can be 10s of gb and take up gbs of RAM.
- Webapps: Think google docs. These run the same as long as you're running google chrome. Great for things that just need your screen, but if you want access to the file system then you're out of luck.

Docker is a slimmed down linux VM, that leverages the linux ancestry of mac and windows to remove the need for as much RAM, whilst giving access to low level hardware and the file system.

### Images and Containers
Docker uses Images to save on RAM and disk space. A docker image is a template. A docker container comes from an image and is a live virtual machine.

## Gazebo, explained
Gazebo is a physics engine that does a lot of maths to allow you to simulate worlds for robotics. 

Gazebo was built on linux because other robotics stuff is built on linux. 

Other robotics stuff (ROS) were built on linux because linux is a) free, so you dont have to pay licensing fees, and b) lightweight, because your robot probably doesn't have a mouse, keyboard or a screen.

