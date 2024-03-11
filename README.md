# NBV Object Recontruction

### Prerequisite
We recommend using a conda environment and installing the following dependencies inside the created conda workspace.

- Our code uses ROS Noetic, first user should make sure that the correct version of ROS is installed. You can install ROS Noetic by following instructions in http://wiki.ros.org/noetic/Installation/Ubuntu
- We use pyrep for communication between CoppeliaSim and Python. You can install it using [Pyrep](https://github.com/stepjam/PyRep)  [Arxiv](https://arxiv.org/abs/1906.11176)
- We use DQ Robotics for rigid motion operations, robot modelling and control, and description of geometric primitives. You can install it using [DQ_Robotics](https://dqrobotics.github.io) [Arxiv](https://arxiv.org/abs/1910.11612).
- We use objects from ShapeNet datasets. It can be accessed by [ShapeNet](https://shapenet.org) [Arxiv](https://arxiv.org/abs/1512.03012)

-Python version is 3.8.16 and pytorch version is 1.13.1.

### Installation
After installing dependecies, you can install our code into your ROS workspace. 

```
cd your_ws/src
git clone our link
cd ..
catkin build
```
### Package Description

We provide two packages:

**focus_point_calculator:** This ROS package handles information gain calculation for NBV and focus point estimation.
**nbv_coppelia:** This package launches coppeliaSim, handles the robot control and publishing required ROS topics.

### Running The Code
```
cd your_ws/src
python3 object_reconstruction_youbot.py
```
This code should start the simulation and all required ROS topics. Upon successfull start, it will run the object reconstruction process for 57 objects. The results will be saved in Results directory.








