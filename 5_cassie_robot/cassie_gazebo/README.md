This repository was originally cloned from https://bitbucket.org/theconstructcore/cassie_biped.

Credits to the construct core for the work done to make cassie work in gazebo.

The video explanation is here: https://www.youtube.com/watch?v=2RgyONB1jI0

A few modifications were made:
- Empty-world instead of spawn-robot-world was used to reduce dependency count.
- spawn-multiple-xacro replaced with spawn-urdf to reduce dependency count.
- Rearrange xacro so that it works with gazebo9's xacro version (the macros have to be called after they're defined, so move to end of file)
- change robot_description to cassie_robot_description  in cassie_control so that the state publisher works.
- Add somewhat realistic limits to the joints in cassie.