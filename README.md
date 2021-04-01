# Panda IKFast
ROS package of Franka Emika Panda with IKFast

## Introduction
This ROS package implement IKFast for Franka Emika Panda. The package is formulated in a service-client fashion. The client passes in (quaternion, position) and get a list of inverse kinematics (IK) solutions. The solution is also ranked by considering the manipulability and distance to the joint limit. The IK solutions and rank list are returned to the client. This code has been tested on python 2.7 and ROS kinetic.

## Dependency
1. [OpenRAVE](http://openrave.org/): [Guide to install OpenRAVE on 16.04](https://scaron.info/teaching/installing-openrave-on-ubuntu-16.04.html)
2. [Trimesh](https://github.com/mikedh/trimesh)
```
pip install trimesh
```
3. Numpy
```
pip install numpy
```
4. [ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## Usage

### Robot DAE file
The robot DAE file is in _xml/_. You can also generate your own .dae file. The guide to generate .dae files from .xacro/.urdf files can be found in the [MoveIt! tutorial](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html).

### Object file
If you want to include an object in the scene, specify the path to the object in *scripts/panda_ikfast_server_node.py*. Both .ply and .obj file are supported.

### Running the code
```
cd ~/catkin_ws/src
git clone https://github.com/hongtaowu67/panda_ikfast
cd ~/catkin_ws
catkin_make
rosrun panda_ikfast panda_ikfast_server_node.py
```
__Note: The return ik_sols is a 1-dim array. The length is 7xn. And each 7 elements is an IK solution. The rank of the solutions is in ik_ranks (descending order).__