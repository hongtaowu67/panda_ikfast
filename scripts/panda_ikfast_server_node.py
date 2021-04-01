#! /usr/bin/python

import os
import rospy
from panda_ikfast.panda_ikfast_solver import PandaIKFast

if __name__ == "__main__":
    rospy.init_node("panda_ikfast_server")

    cwd = os.getcwd()

    env_xml = "../xml/world.env.xml"
    obj_path = None
    PandaIKFast = PandaIKFast(env_xml, obj_path)
    PandaIKFast.run_panda_ikfast_server()

