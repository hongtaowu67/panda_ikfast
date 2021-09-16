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
    def __init__(self, env_xml):
        """
        Setting up OpenRave and ROS.

        @type  env_xml: string
        @param env_xml: xml file of the environment
        """
        # Environment
        self.env = orpy.Environment()
        self.env.SetViewer('qtcoin')
        self.env.Load(env_xml)
        self.env.SetCollisionChecker(orpy.RaveCreateCollisionChecker(self.env, 'ode'))

        with self.env:
            body = orpy.RaveCreateKinBody(self.env, '')
            body.InitFromBoxes(np.array([[0.6, 0.0, -0.06, 1.0, 0.8, 0.1]]))
            body.SetName("table")
            self.env.AddKinBody(body)
        
        body.Enable(True)
        
        # Robot
        self.robot = self.env.GetRobot('panda')

        # self.manip_name= 'panda_arm_hand'
        self.manip_name = 'panda_arm_hand_no_finger'
        self.manip = self.robot.SetActiveManipulator(self.manip_name)
        self.setup_ikfast()
        self.robot.Enable(True)

        self.open_gripper(1.0)
        self.disable_gripper()

        self.robot_home_config = [0.0, -np.pi / 4, 0, -2 * np.pi / 3, 0, np.pi / 3, np.pi / 4]
        self.robot_q_max = self.robot.GetDOFLimits()[0][0:7]
        self.robot_q_min = self.robot.GetDOFLimits()[1][0:7]
        self.q_lim_const = (self.robot_q_max - self.robot_q_min) * (self.robot_q_max - self.robot_q_min)

        self.robot_go_home()

        # Object
        self.obj = None
    
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

    def load_object(self, obj_path, obj_quat, obj_pos):
        """
        Load object into the env.

        @type  obj_path: string
        @param obj_path: path to the object (.ply .obj)
        @type  obj_quat: list
        @param obj_path: object quaternion (w, x, y, z)
        @type  obj_path: list
        @param obj_path: object position
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

        # Setting the object pose
        obj_quat = obj_quat
        obj_pos = obj_pos
        T_obj = np.eye(4)
        T_obj[:3, :3] = utils.quat2rotm(obj_quat)
        T_obj[:3, 3]  = obj_pos
        self.obj.SetTransform(T_obj)

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

        # Setting the object pose
        if self.obj:
            obj_quat = req.obj_quat
            obj_pos = req.obj_pos
            T_obj = np.eye(4)
            T_obj[:3, :3] = utils.quat2rotm(obj_quat)
            T_obj[:3, 3]  = obj_pos
            self.obj.SetTransform(T_obj)

        rospy.loginfo("Computing IK for ({}, {})".format(quat, pos))

        ik_sols = self.get_ik_solutions(quat, pos)
        rospy.loginfo("Got {} sets of IK solutions".format(ik_sols.shape))
        ik_rank, ik_scores = self.rank_ik_sols(ik_sols)

        ik_sols = ik_sols.flatten()

        return [ik_sols, ik_rank]

    def run_panda_ikfast_server(self):
        """
        Initialize the ROS services
        """
        self.s = rospy.Service("panda_ikfast", PandaIK, self.handle_panda_ikfast)
        rospy.sleep(0.5)
        rospy.loginfo("panda_ikfast service ready!")
        
    
    def handle_load_object(self, req):
        """
        load object handler
        """
        obj_path = req.object_mesh_path
        obj_quat = req.object_quat
        obj_pos  = req.object_pos

        if obj_path:
            self.obj = self.load_object(obj_path, obj_quat, obj_pos)

        return LoadObjectResponse("Successfully load object!")

    def run_load_object_server(self):
        """
        Initialize the load object service
        """
        self.load_object_server = rospy.Service("load_object", LoadObject, self.handle_load_object)
        rospy.loginfo("load_object service ready!")
