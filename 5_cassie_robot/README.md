# CASSIE
In this tutorial we will simulate a bipedal robot called CASSIE.
When cassie was available for purchase (before 2019), it cost less than $300,000. 
So, since you probably don't have that much money casually sitting in the bank, we're going to run a simulated cassie today.

Credits to UMich-BipedLab for producing the cad model package for cassie.
Credits to ConstructSim team for producing a simulation-ready version of cassie URDFs.

1. Open a terminal / cmd, and `cd ` into the `5_cassie_robot` folder. 
2. Run `docker build -f Dockerfile.build . -t usrc_tutorial_5_image`.
3. Run the container, using `docker run --rm --name=usrc_gazebo_container -p 5900:5900 -it usrc_tutorial_5_image:latest`.
    - You will likely need to run `docker container rm usrc_gazebo_container` to clean out the previous container.
4. Run `roslaunch cassie_gazebo main.launch`. 
    - The robot should collapse onto the ground. Whoops. Hope you bought some robot insurance.
5. Run `roslaunch cassie_gazebo pinned.launch`. In this version, we've pinned cassie to a point, so it won't fall over.
6. Now, let's run `rosrun cassie_gazebo cassie_run.py` to make our robot appear to run.
 

## For maintainers
docker cp cassie_gazebo usrc_gazebo_container:/root/catkin_ws/src