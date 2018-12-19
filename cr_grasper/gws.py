import pybullet as p
import pybullet_data
from time import sleep, time
from mathutils import Vector #https://github.com/majimboo/py-mathutils
from math import sqrt
from numpy import cross, diagonal
from numpy.linalg import norm
import numpy as np
from scipy.spatial import ConvexHull


#physicsClient = p.connect(p.DIRECT)  #p.DIRECT for non-graphical version
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

# This is to change the visualizer window settings
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, enable=0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable=0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable=0)
# change init camera distance/location to view scene
p.resetDebugVisualizerCamera(cameraDistance=.5, cameraYaw=45, cameraPitch=-20, cameraTargetPosition=[0.0, 0.0, 0.0])


rID = p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/RobotURDFs/barrett_hand_description/urdf/bh.urdf", basePosition=[0,0,-.1], globalScaling=1, useFixedBase=True)
oID = p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/ObjectURDFs/cube/cube_small.urdf", globalScaling=1, useFixedBase=True)

p.addUserDebugLine([0, 0, 0], [.2, 0, 0], [1, 0, 0], parentObjectUniqueId=rID, parentLinkIndex=-1, lineWidth=500)
p.addUserDebugLine([0, 0, 0], [0, .2, 0], [0, 1, 0], parentObjectUniqueId=rID, parentLinkIndex=-1, lineWidth=500)
p.addUserDebugLine([0, 0, 0], [0, 0, .2], [0, 0, 1], parentObjectUniqueId=rID, parentLinkIndex=-1, lineWidth=500)

active_grasp_joints = [3,6,9]
max_grasp_force = 100
target_grasp_velocity = .3

def grasp(handId):
    """
    closes the gripper uniformly + attempts to find a grasp
    this is based on time + not contact points because contact points could just be a finger poking the object
    relies on grip_joints - specified by user/config file which joints should close
    TODO: make grasp mirror barret hand irl = that means make the base joint close before tip joint in hand
    """
    print("finding grasp")
    finish_time = time() + 2
    while time() < finish_time:
        p.stepSimulation()
        for joint in active_grasp_joints:
            p.setJointMotorControl2(bodyUniqueId=handId, jointIndex=joint, controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=target_grasp_velocity, force=max_grasp_force)


def relax(handID):
    """
    return all joints to neutral/furthest extended, based on urdf specification
    """
    print("relaxing hand")
    joint = 0
    num = p.getNumJoints(handID)
    while joint < num:
        p.resetJointState(bodyUniqueId=handID, jointIndex=joint, targetValue=0.0)
        joint = joint + 1



def get_obj_info(oID):
    obj_data = p.getCollisionShapeData(oID, -1)[0]
    geometry_type = obj_data[2]
    print("geometry type: " + str(geometry_type))
    dimensions = obj_data[3]
    print("dimensions: "+ str(dimensions))
    local_frame_pos = obj_data[5]
    print("local frome position: " + str(local_frame_pos))
    local_frame_orn = obj_data[6]
    print("local frame oren: " + str(local_frame_orn))
    # TODO: what about not mesh objects?
    diagonal = sqrt(dimensions[0]**2+dimensions[1]**2+dimensions[2]**2)
    print("diagonal: ", diagonal)
    max_radius = diagonal/2
    return local_frame_pos, max_radius

def gws(rID, oID):
    print("eval gws")
    local_frame_pos, max_radius = get_obj_info(oID)
    #sim uses center of mass as a reference for the Cartesian world transforms in getBasePositionAndOrientation
    obj_pos, obj_orn = p.getBasePositionAndOrientation(oID)
    force_torque = []
    contact_points = p.getContactPoints(rID, oID)
    for point in contact_points:
        contact_pos = point[6]
        normal_vector_on_obj = point[7]
        normal_force_on_obj = point[9]
        force_vector = np.array(normal_vector_on_obj)*normal_force_on_obj


        radius_to_contact = np.array(contact_pos) - np.array(obj_pos)
        torque_numerator = cross(radius_to_contact, normal_vector_on_obj)
        torque_vector = torque_numerator/max_radius

        #force_mag = norm(normal_vector_size)
        #torque_mag = norm(torque)
        #lmda = 1
        force_torque.append(np.concatenate([force_vector, torque_vector]))

    return force_torque



#VOLUME QUALITY

"""
take the qhull of the 6 dim vectors [fx, fy, fz, tx, ty, tz] created by gws
the volume of this qhull is the metric 
"""

def volume(force_torque):
    convex_hull = ConvexHull(points = force_torque)
    return convex_hull





get_obj_info(oID)

while True:
    relax(rID)
    grasp(rID)
    p.stepSimulation()
    force_torque = gws(rID, oID)
    print(force_torque)
    #print(np.array(force_torque).shape)
    #vol = volume(force_torque)
    #print(vol)
    #print(vol.volume)


    while True:
        p.stepSimulation()


