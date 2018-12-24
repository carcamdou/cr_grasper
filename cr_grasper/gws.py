import pybullet as p
import pybullet_data
from time import sleep, time
from mathutils import Vector #https://github.com/majimboo/py-mathutils
from math import sqrt, pi
from numpy import cross, diagonal
import numpy as np
from scipy.spatial import ConvexHull, distance
from pyquaternion import Quaternion

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
target_grasp_velocity = .7

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



def get_obj_info(oID): #TODO: what about not mesh objects?
    obj_data = p.getCollisionShapeData(oID, -1)[0]
    geometry_type = obj_data[2]
    #print("geometry type: " + str(geometry_type))
    dimensions = obj_data[3]
    #print("dimensions: "+ str(dimensions))
    local_frame_pos = obj_data[5]
    #print("local frome position: " + str(local_frame_pos))
    local_frame_orn = obj_data[6]
    #print("local frame oren: " + str(local_frame_orn))
    diagonal = sqrt(dimensions[0]**2+dimensions[1]**2+dimensions[2]**2)
    #print("diagonal: ", diagonal)
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
        torque_numerator = cross(radius_to_contact, force_vector)
        torque_vector = torque_numerator/max_radius

        #force_mag = norm(normal_vector_size)
        #torque_mag = norm(torque)
        #lmda = 1
        force_torque.append(np.concatenate([force_vector, torque_vector]))

    return force_torque


def get_new_normals(force_vector, normal_force, sides, radius):
    return_vectors = []
    #get arbitrary vector to get cross product which should be orthogonal to both
    vector_to_cross = np.array((force_vector[0]+1,force_vector[1]+2,force_vector[2]+3))
    orthg = np.cross(force_vector, vector_to_cross)
    orthg_vector = (orthg/np.linalg.norm(orthg))*radius
    rot_angle = (2*pi)/sides
    split_force = normal_force/sides

    for side_num in range(0,sides):
        rotated_orthg = Quaternion(axis=force_vector, angle=(rot_angle*side_num)).rotate(orthg_vector)
        new_vect = force_vector+np.array(rotated_orthg)
        norm_vect = (new_vect/np.linalg.norm(new_vect))*split_force
        return_vectors.append(norm_vect)

    return return_vectors


def gws_pyramid_extension(rID, oID, pyramid_sides = 6, pyramid_radius = .01):
    print("gws pyramid creation")
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
        if np.linalg.norm(force_vector) > 0:
            new_vectors = get_new_normals(force_vector, normal_force_on_obj, pyramid_sides, pyramid_radius)

            radius_to_contact = np.array(contact_pos) - np.array(obj_pos)

            for pyramid_vector in new_vectors:
                torque_numerator = cross(radius_to_contact, pyramid_vector)
                torque_vector = torque_numerator/max_radius
                force_torque.append(np.concatenate([pyramid_vector, torque_vector]))

    return force_torque


def volume(force_torque):
    """
    get qhull of the 6 dim vectors [fx, fy, fz, tx, ty, tz] created by gws (from contact points)
    get the volume
    """
    convex_hull = ConvexHull(points = force_torque)
    return convex_hull.volume


def eplison(force_torque):
    """
    get qhull of the 6 dim vectors [fx, fy, fz, tx, ty, tz] created by gws (from contact points)
    get the distance from centroid of the hull to the closest vertex
    """
    hull = ConvexHull(points=force_torque)
    centroid = []
    for dim in range(0,6):
        centroid.append(np.mean(hull.points[hull.vertices, dim]))
    shortest_distance = 500000000
    closest_point = None
    for point in force_torque:
        point_dist = distance.euclidean(centroid, point)
        if point_dist < shortest_distance:
            shortest_distance = point_dist
            closest_point = point

    return shortest_distance



get_obj_info(oID)

while True:
    relax(rID)
    grasp(rID)
    p.stepSimulation()
    #force_torque = gws(rID, oID)
    force_torque = gws_pyramid_extension(rID, oID)
    #print(force_torque)
    #print(np.array(force_torque).shape)
    vol = volume(force_torque)
    print("volume: ", vol)
    ep = eplison(force_torque)
    print("eplison: ", ep)

    while True:
        p.stepSimulation()


