#! /usr/bin/python

import rospy
from panda_ikfast.panda_ikfast_solver import PandaIKFast

if __name__ == "__main__":
    rospy.init_node("panda_ikfast_server")

    env_xml = "/home/hongtao/panda_ws/src/panda_ikfast/xml/world.env.xml"
    obj_path = "/home/hongtao/Dropbox/ISRR2021_bear/training_data/0216_5/mesh.obj"

    PandaIKFast = PandaIKFast(env_xml, obj_path)
    PandaIKFast.run_panda_ikfast_server()

