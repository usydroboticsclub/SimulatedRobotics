#!/usr/bin/python
import sys
import rospy
import random
import os
import os.path as path

from gazebo_msgs.srv import *
from geometry_msgs.msg import *
import tf.transformations as tft

if __name__ == "__main__":

    rospy.init_node("object_spawner")
    rospy.loginfo("Init node ok")

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.loginfo("got service")

    rate = rospy.Rate(0.1)  # 0.1Hz - once per 10s
    model_name = "sphere"
    xml_strings = ["",""]
    filepath = [
        path.dirname(path.dirname(path.abspath(__file__)))+"/urdf/sphere_green.urdf",
        path.dirname(path.dirname(path.abspath(__file__)))+"/urdf/sphere_blue.urdf",
    ]
    for i,p in enumerate(filepath):
        with open(p) as f:
            xml_strings[i] = str(f.read())

    spawn_number = 0

    while not rospy.is_shutdown():
        rospy.loginfo("spin started")
        # Delete previous sphere, so we can keep spawning them
        srv_delete_model = rospy.ServiceProxy(
            'gazebo/delete_model', DeleteModel)
        req = DeleteModelRequest()
        req.model_name = model_name+"_"+str(spawn_number)
        exists = True
        try:
            res = srv_delete_model(req)
        except rospy.ServiceException, e:
            exists = False
            rospy.logdebug(
                "Model %s does not exist in gazebo.", model_name)
        x = -7
        y = 0
        z = 8
        rospy.loginfo("wait started")
        rospy.sleep(1)
        rospy.loginfo("wait ended")

        object_pose = Pose()
        object_pose.position.x = float(x)
        object_pose.position.y = float(y)
        object_pose.position.z = float(z)
        object_pose.orientation=Quaternion(*tft.quaternion_from_euler(0,0,0))

        rospy.loginfo("Trying to spawn sphere at %d, %d, %d" % (x, y, z))
        srv_spawn_model = rospy.ServiceProxy(
            '/gazebo/spawn_urdf_model', SpawnModel)
        # spawn new model
        req = SpawnModelRequest()
        req.model_name = model_name+"_"+str(spawn_number)  # model name from command line input
        req.model_xml = xml_strings[random.randint(0,1)]
        req.initial_pose = object_pose
        res = srv_spawn_model(req)

        spawn_number+=1
        spawn_number%=5
        # evaluate response
        if res.success == True:
            rospy.loginfo("Spawn ok")
        else:
            print("Error: model not spawn. error message = " + res.status_message)
        rate.sleep()
