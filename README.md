# NBV Object Recontruction

### Prerequisite
We recommend using a Conda environment and installing the following dependencies within the created Conda workspace.

- Our code uses ROS Noetic. Before proceeding, users should ensure that the correct version of ROS is installed. You can install ROS Noetic by following the instructions provided at http://wiki.ros.org/noetic/Installation/Ubuntu.
- We use pyrep for communication between CoppeliaSim and Python. You can install it using [Pyrep](https://github.com/stepjam/PyRep)  [Arxiv](https://arxiv.org/abs/1906.11176)
- We use DQ Robotics for rigid motion operations, robot modelling and control, and description of geometric primitives. You can install it using [DQ_Robotics](https://dqrobotics.github.io) [Arxiv](https://arxiv.org/abs/1910.11612).
- We use objects from ShapeNet datasets. It can be accessed by [ShapeNet](https://shapenet.org) [Arxiv](https://arxiv.org/abs/1512.03012)

-Python version is 3.8.16 and Pytorch version is 1.13.1.

### Installation
After installing dependencies, you can proceed to install our code into your ROS workspace:

```
cd your_ws/src
git clone https://github.com/fthdrsn/NBV_object_recontruction.git
cd ..
catkin build
```
### Package Description

We provide two packages:

**focus_point_calculator:** This ROS package handles the calculation of information gain for NBV (Next-Best-View) and focus point estimation.

**nbv_coppelia:** This package launches CoppeliaSim, manages robot control, and publishes the required ROS topics.

### Running The Code
```
cd your_ws/src
python3 object_reconstruction_youbot.py
```
This code initiates the simulation and all necessary ROS topics. Upon successful initialization, it starts executing the object reconstruction process for 57 objects. The results will be saved in the "Results" directory.








