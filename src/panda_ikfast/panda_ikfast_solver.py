"""
IKFast service for the Panda robot
Author: Hongtao Wu, Johns Hopkins University
Date: Apr 1, 2021
"""

import numpy as np
import openravepy as orpy
import trimesh

import rospy

import utils

from panda_ikfast.srv import *

IK_CHECK_COLLISION = orpy.IkFilterOptions.CheckEnvCollisions

class PandaIKFast(object):
    """
    Class to get IK solutions for Panda robot with IKFast (OpenRave)
    """
    def __init__(self, env_xml, obj_path=None):
        """
        Setting up OpenRave and ROS.

        @type  env_xml: string
        @param env_xml: xml file of the environment
        """
        #### Openrave setting ####
        # Environment
        self.env = orpy.Environment()
        # self.env.SetViewer('qtcoin')
        self.env.Load(env_xml)
        self.env.SetCollisionChecker(orpy.RaveCreateCollisionChecker(self.env, 'ode'))
        
        # Robot
        self.robot = self.env.GetRobot('panda')

        self.manip_name= 'panda_arm_hand'
        self.manip = self.robot.SetActiveManipulator(self.manip_name)
        self.setup_ikfast()
        self.robot.Enable(True)

        self.open_gripper(1.0)
        self.disable_gripper()

        self.robot_home_config = [0.0, 0.2, 0, -1.3, 0, 1.571, 0.785]
        self.robot_q_max = self.robot.GetDOFLimits()[0][0:7]
        self.robot_q_min = self.robot.GetDOFLimits()[1][0:7]
        self.q_lim_const = (self.robot_q_max - self.robot_q_min) * (self.robot_q_max - self.robot_q_min)

        self.robot_go_home()

        # Object
        if obj_path:
            self.body = self.load_object(obj_path)
        ###########################
    
    def robot_go_home(self):
        """
        Move the robot to home configuration.
        """
        self.robot.SetActiveDOFValues(self.robot_home_config)

    def setup_ikfast(self):
        """
        Generate/Load the IKFast model.
        """
        ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(
            self.robot, iktype=orpy.IkParameterization.Type.Transform6D)
        if (not ikmodel.load()):
            robot_name = self.robot.GetName()
            print('Robot:[' + robot_name +
                  '] IKFast not found. Generating IKFast solution...')
            ikmodel.autogenerate()
            print('Finish generating IK model for [{}]'.format(robot_name))
    
    def get_ik_solutions(self, quat, pos):
        """
        Get the IK solutions for a pose.

        @type  quat: list
        @param quat: quaternion in (qw, qx, qy, qz)
        @rtype: numpy.ndarray
        @return: list of ik solutions
        """
        rotm = utils.quat2rotm(quat)
        T = np.eye(4)
        T[:3, :3] = rotm
        T[:3, 3] = pos

        ik_solutions = self.manip.FindIKSolutions(T, IK_CHECK_COLLISION)

        return ik_solutions

    def load_object(self, obj_path):
        """
        Load object into the env.

        @type  obj_path: string
        @param obj_path: path to the object (.ply .obj)
        @rtype: openravepy.KinBody
        @return: object
        """
        mesh = trimesh.load(obj_path)
        
        with self.env:
            body = orpy.RaveCreateKinBody(self.env, '')
            body.InitFromTrimesh(trimesh=orpy.TriMesh(mesh.vertices, mesh.faces), draw=True)
            body.SetName("chair")
            self.env.AddKinBody(body)
        
        body.Enable(True)

        return body

    def check_collision(self, joint_values):
        """
        Check robot collision.

        @type  joint_values: numpy.ndarray
        @param joint_values: joint configuration value
        @rtype: bool
        @return: If the robot is in collision, then True; otherwise False
        """
        self.robot.SetActiveDOFValues(joint_values)
        is_collision = self.env.CheckCollision(self.robot)
        is_collision |= self.robot.CheckSelfCollision()

        return is_collision

    def open_gripper(self, scale):
        """
        Open the gripper. Franka Emika Panda robot.
        
        @type  scale: float
        @param scale: open scale, [0, 1]
        """
        limit = self.robot.GetActiveDOFLimits()[1][-1]
        curr_config = self.robot.GetActiveDOFValues()

        assert len(curr_config) == 7 or len(curr_config) == 8

        if len(curr_config) == 7:
            goal_config = curr_config + [scale * limit]
        else:
            goal_config = curr_config
            goal_config[-1] = scale * limit

        self.robot.SetActiveDOFValues(goal_config)

    def disable_gripper(self):
        """
        Disable the DOF at the hand.
        
        @type  robot: openrave.Robot
        @param robot: Robot to be configure
        """
        self.robot.SetActiveDOFs(self.manip.GetArmIndices())

    def rank_ik_sols(self, ik_list):
        """
        Rank the ik solutions based on joint limits and manipulability

        @type  ik_list: numpy.ndarray
        @param ik_list: list of ik solution
        @rtype: numpy.ndarray
        @return: index of the ik in decending score order
        @rtype: numpy.ndarray
        @return: score of each ik. The order of the score is the same
                as the passed-in ik_list.
        """
        score = np.zeros(len(ik_list))
        with self.robot:
            for ik_idx, ik in enumerate(ik_list):
                self.robot.SetActiveDOFValues(ik)
                J_trans = self.manip.CalculateJacobian()
                J_rot   = self.manip.CalculateAngularVelocityJacobian()

                # 6x7 Jacobian
                J = np.vstack([J_rot, J_trans])

                # Manipulability
                manipulability = np.sqrt(np.linalg.det(np.matmul(J, J.transpose())))
                # Joint limit score
                a = self.robot_q_max - ik
                b = ik - self.robot_q_min
                lim_score = np.sum(self.q_lim_const / (a * b))

                score[ik_idx] = manipulability / lim_score
        
        rank = np.argsort(score)
        return rank[-len(rank):][::-1], score

    def handle_panda_ikfast(self, req):
        """
        ROS service handler
        """
        quat = req.quat
        pos = req.pos

        ik_sols = self.get_ik_solutions(quat, pos)
        ik_rank, _ = PandaIKFast.rank_ik_sols(ik_sols)

        ik_sols = ik_sols.flatten()

        return [ik_sols, ik_rank]

    def run_panda_ikfast_server(self):
        """
        Initialize the ROS services
        """
        self.s = rospy.Service("panda_ikfast", PandaIK, self.handle_panda_ikfast)
        rospy.sleep(0.5)
        rospy.loginfo("panda_ikfast service ready")
        rate = rospy.Rate(1)
        rospy.spin()

    
# if __name__ == "__main__":
#     env_xml = "/home/hongtao/panda_ws/src/panda_ikfast/xml/world.env.xml"
#     obj_path = "/home/hongtao/Dropbox/ISRR2021_bear/training_data/0216_5/mesh.obj"
#     PandaIKFast = PandaIKFast(env_xml, obj_path)
    
#     quat = np.array([0.521993830667, -0.315017999999, 0.764402798267, 0.20970088799])
#     pos  = np.array([-0.245948463939, 0.251846808876, 0.233295196863])
#     ik_sols = PandaIKFast.get_ik_solutions(quat, pos)
#     ik_rank, score = PandaIKFast.rank_ik_sols(ik_sols)

#     for ik_idx in ik_rank:
#         print("{}: {}".format(score[ik_idx], ik_sols[ik_idx]))