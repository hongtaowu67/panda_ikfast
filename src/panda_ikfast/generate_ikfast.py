import openravepy as orpy

from panda_ikfast.srv import *

IK_CHECK_COLLISION = orpy.IkFilterOptions.CheckEnvCollisions

if __name__ == "__main__":
    env_xml = "/home/sisu/catkin_ws/src/panda_ikfast/xml/world.env.xml"
    # Environment
    env = orpy.Environment()
    env.SetViewer('qtcoin')
    env.Load(env_xml)
    env.SetCollisionChecker(orpy.RaveCreateCollisionChecker(env, 'ode'))
    
    # Robot
    robot = env.GetRobot('panda')

    manip_name= 'panda_arm_hand'
    manip = robot.SetActiveManipulator(manip_name)

    ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(
        robot, iktype=orpy.IkParameterization.Type.Transform6D)
    if (not ikmodel.load()):
        robot_name = robot.GetName()
        print('Robot:[' + robot_name +
                '] IKFast not found. Generating IKFast solution...')
        ikmodel.autogenerate()
        print('Finish generating IK model for [{}]'.format(robot_name))


