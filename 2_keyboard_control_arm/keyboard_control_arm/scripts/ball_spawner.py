#!/usr/bin/python
import sys
import rospy
import random
import os

from gazebo.srv import *
from geometry_msgs.msg import *
import tf.transformations as tft

if __name__ == "__main__":

	rospy.init_node("object_spawner")

	rospy.sleep(5)
	rospy.wait_for_service("/gazebo/spawn_urdf_model")

    rate = rospy.Rate(0.1) # 0.1Hz
    model_name = "sphere"
    xml_string=""

    while not rospy.is_shutdown():

        x=0
        y=random.random()*3+2
        z=0
	    rospy.loginfo("Trying to spawn sphere at %d, %d, %d" % (x,y,z))
		srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        # spawn new model
		req = SpawnModelRequest()
		req.model_name = model_name # model name from command line input
		req.model_xml = xml_string
		req.initial_pose = object_pose


		# Delete previous sphere, so we can keep spawning them
		srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
		req = DeleteModelRequest()
		req.model_name = model_name
		exists = True
		try:
			res = srv_delete_model(req)
		except rospy.ServiceException, e:
			exists = False
			rospy.logdebug("Model %s does not exist in gazebo.", model_name)

		if exists:
			rospy.loginfo("Model %s already exists in gazebo. Model will be updated.", name)


		res = srv_spawn_model(req)
	
		# evaluate response
		if res.success == True:
			rospy.loginfo(res.status_message + " " + name)
		else:
			print "Error: model %s not spawn. error message = "% name + res.status_message
		
	sim = rospy.get_param('/use_sim_time')
			
	if sim is True:
    
		rospy.loginfo('Running in simulation, publishing to /sim_spawned topic')
      
		pub = rospy.Publisher('/sim_spawned', EmptyMsg,latch=True)
		pub.publish(EmptyMsg())
		pub.publish(EmptyMsg())
		pub.publish(EmptyMsg())
      
		rospy.spin()
