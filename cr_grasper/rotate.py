import pybullet as p
import pybullet_data
from time import sleep, time
from mathutils import Vector #https://github.com/majimboo/py-mathutils
from pyquaternion import Quaternion
import numpy as np
from math import pi
import random

#physicsClient = p.connect(p.DIRECT)  #p.DIRECT for non-graphical version
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

# This is to change the visualizer window settings
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, enable=0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable=0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable=0)
# change init camera distance/location to view scene
p.resetDebugVisualizerCamera(cameraDistance=.75, cameraYaw=100, cameraPitch=-20, cameraTargetPosition=[0, 0, 0])


#rID = p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/RobotURDFs/gripper_description/pr2_gripper.urdf", useFixedBase=True)
rID = p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/RobotURDFs/barrett_hand_description/urdf/bh.urdf", globalScaling=1, useFixedBase=True)
#rID = p.loadSDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/pybullet_examples/bullet3/data/gripper/wsg50_one_motor_gripper.sdf")[0]



dist = .5


def add_debug_lines(rID):
    p.addUserDebugLine([0, 0, 0], [.75, 0, 0], [1, 0, 0], parentObjectUniqueId=rID, parentLinkIndex=-1, lineWidth=500)
    p.addUserDebugLine([0, 0, 0], [0, .75, 0], [0, 1, 0], parentObjectUniqueId=rID, parentLinkIndex=-1, lineWidth=500)
    p.addUserDebugLine([0, 0, 0], [0, 0, .75], [0, 0, 1], parentObjectUniqueId=rID, parentLinkIndex=-1, lineWidth=500)


def grasp(handId):
    """
    closes the gripper uniformly + attempts to find a grasp
    this is based on time + not contact points because contact points could just be a finger poking the object
    relies on grip_joints - specified by user/config file which joints should close
    TODO: make grasp mirror barret hand irl = that means make the base joint close before tip joint in hand
    """
    print("finding grasp")
    finish_time = time() + 3
    while time() < finish_time:
        p.stepSimulation()
        for joint in [3,6,9]:
            p.setJointMotorControl2(bodyUniqueId=handId, jointIndex=joint, controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=.3, force=100)

def relax(handID):
    """
    return all joints to neutral/furthest extended, based on urdf specification
    """
    print("relaxing hand")
    joint = 0
    num = p.getNumJoints(handID)
    while joint < num:
        p.resetJointState(handID, jointIndex=joint, targetValue=0.0)
        joint = joint + 1


#pos =[.1,.2,.3]
#quat = [.1, .2, .3, .4]

#pos =[0,0,0]
#quat = [0,0,0,1]
#test = [([dist, 0, 0], [0, -1, 0, 1]), ([-dist, 0, 0], [0, 1, 0, 1]), ([0, dist, 0], [1, 0, 0, 1]), ([0, -dist, 0], [-1, 0, 0, 1]), ([0, 0, -dist], [0, 0, 0, 1])]

poses = (((0.10223707190067914, -1.2520430285736452e-17, 0.10223707190067913), (-2.3432602026631496e-17, 0.9238795325112867, -5.657130561438501e-17, -0.3826834323650899)), ((-0.10223707190067913, 0.0, 0.10223707190067911) , (0.9238795325112867, -3.313870358775352e-17, 0.3826834323650899, -3.313870358775352e-17)), ((0.10223707190067913, -1.252043028573645e-17, 0.10223707190067911) , (8.971000920213853e-17, 0.9238795325112867, -9.706101561122023e-18, -0.3826834323650899)))


rot_vec = [0,0,1]

i = 1
iter = pi/4
for pose in poses:
    pos = pose[0]
    quat = pose[1]

    print("original")
    p.removeAllUserDebugItems()
    p.addUserDebugText("Original", [-.07, .07, .07], textColorRGB=[0, 1, 0], textSize=1)
    p.resetBasePositionAndOrientation(rID, pos, quat)
    add_debug_lines(rID)
    relax(rID)
    grasp(rID)
    sleep(.1)

    for i in range(0,8):
        p.stepSimulation()

        print("rotate wrist for new grasp")
        quat_w = quat[3]
        quat_x = quat[0]
        quat_y = quat[1]
        quat_z = quat[2]

        current_quat = Quaternion(quat_w, quat_x, quat_y, quat_z)
        #rot_quat = Quaternion(axis=(np.array(pos)), radians= random.uniform(-pi/2, pi/2))
        #rot_quat = Quaternion(axis=(np.array(pos)), radians=pi / 2 + (iter * i))
        rot_quat = Quaternion(axis=(np.array((-pos[0], -pos[1], -pos[2]))), radians=pi / 2 + (iter * i))
        #rot_quat = Quaternion(axis=(np.array(pos)), radians=pi / 2 + (iter * i))
        #delta_quat = current_quat + rot_quat
        delta_quat = rot_quat * current_quat
        pyb_quat = (delta_quat[1], delta_quat[2], delta_quat[3], delta_quat[0])

        print("addition")
        p.removeAllUserDebugItems()
        p.addUserDebugText("Addition", [-.07, .07, .07], textColorRGB=[1, 0, 0], textSize=1)
        p.resetBasePositionAndOrientation(rID, pos, (delta_quat[1], delta_quat[2], delta_quat[3], delta_quat[0]))
        add_debug_lines(rID)
        relax(rID)
        grasp(rID)
        sleep(.01)
        """
        print("mult")
        p.removeAllUserDebugItems()
        p.addUserDebugText("mult", [-.07, .07, .07], textColorRGB=[1, 0, 0], textSize=1)
        p.resetBasePositionAndOrientation(rID,pos, (mult_quat[1], mult_quat[2], mult_quat[3], mult_quat[0]))
        add_debug_lines(rID)
        relax(rID)
        grasp(rID)
        sleep(.1)
        """

