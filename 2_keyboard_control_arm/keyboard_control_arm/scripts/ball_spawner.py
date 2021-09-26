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

    rate = rospy.Rate(0.05)  # 0.05Hz - once per 20s
    model_name = "sphere"
    xml_string = ""
    filepath = path.dirname(path.dirname(path.abspath(__file__)))+"/urdf/sphere.urdf"
    with open(filepath) as f:
        xml_string = str(f.read())

    spawned_once = False

    while not rospy.is_shutdown():
        rospy.loginfo("spin started")
        if spawned_once:
            # Delete previous sphere, so we can keep spawning them
            srv_delete_model = rospy.ServiceProxy(
                'gazebo/delete_model', DeleteModel)
            req = DeleteModelRequest()
            req.model_name = model_name
            exists = True
            try:
                res = srv_delete_model(req)
            except rospy.ServiceException, e:
                exists = False
                rospy.logdebug(
                    "Model %s does not exist in gazebo.", model_name)
        x = 0
        y = random.random()*3+2
        z = 5
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
        req.model_name = model_name  # model name from command line input
        req.model_xml = xml_string
        req.initial_pose = object_pose
        res = srv_spawn_model(req)

        spawned_once=True
        # evaluate response
        if res.success == True:
            rospy.loginfo("Spawn ok")
        else:
            print("Error: model not spawn. error message = " + res.status_message)
        rate.sleep()
