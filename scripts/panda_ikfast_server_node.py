#! /usr/bin/python

import os
import rospy
from panda_ikfast.panda_ikfast_solver import PandaIKFast

if __name__ == "__main__":
    rospy.init_node("panda_ikfast_server")

    env_xml = "/home/sisu/catkin_ws/src/panda_ikfast/xml/world.env.xml"

    PandaIKFast = PandaIKFast(env_xml=env_xml)
    PandaIKFast.run_handle_object_server()
    PandaIKFast.run_panda_ikfast_server()

    rate = rospy.Rate(1)
    rospy.spin()
