"""

Carlyn C. Dougherty
ccd2134@columbia.edu
Robotics Research, Fall 2018

"""

# TODO: do the list planner w grasps around the object - then return whether they work
# load object. get grasps in a circle of angles around the object (from 0 to 360 in a circle)
# spit back finger joints, position, orientation
# rotate the hand around the base axis to get new grasps
# hand distance - detect the object colision between the hand and then then move a slight bit away to find optimal distance to object
# remember to change everything back to scaling of 1 before you give back numbers that make no sense
# then load in new objects - work on getting them into the
# figure out another hand
# then work on simulated annealing (which is better grasps)


########################################################################################################################


import pybullet as p
from math import pi, sqrt
import pybullet_data
from time import sleep, time
import astropy.coordinates
import random
from transforms3d import euler


"""#####################################################################################################################
                                    PYBULLET HOUSEKEEPING + GUI MAINTENANCE 
#####################################################################################################################"""

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

#This is to change the visualizer window settings
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,enable=0)
p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,enable=0)
p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW ,enable=0)
#change init camera distance/location to view scene
p.resetDebugVisualizerCamera(cameraDistance=.5, cameraYaw=135, cameraPitch=-20, cameraTargetPosition=[0.0, 0.0, 0.0])


"""#####################################################################################################################
                                       GLOBAL VARIABLES - to be put into config file
#####################################################################################################################"""

#loading
robot_path = "barrett_hand_ros/barrett_hand_description/urdf/bh.urdf"
object_path = "cube_small.urdf"
#object_path ="teddy_vhacd.urdf"


#finding hand positions
#TODO: make init hand position sufficiently far for the first one, and then update the h_dist based on first contact of the first location
h_dist = .19 #how far from the origin should the hand be at the start (just needs to be beyond the length of the object)
margin = .01 #how far from touching do you want the palm to be when attempting grips

#grasping
t_vel = .3 #target velocity for joints when grasping
maxForce = 100 #max force allowed
time_limit = 3.0 #how long given to find a grasp
#TODO: make this part of config file - joints to move should depend on user
grip_joints = [3,6,9] #which joints in the hand to use - specified by num in the URDF


"""#####################################################################################################################
                                                    UTILIES
#####################################################################################################################"""


def rand_coord():
    """
    Gets random polar coordinate by generating 2 random #s between -pi/2 and pi/2 to serve as theta and phi
    """
    rand_theta =random.uniform(-pi/2,pi/2)
    rand_phi = random.uniform(-pi/2,pi/2)
    return rand_theta, rand_phi


"""#####################################################################################################################
                                           URDF/OBJECT MANAGEMENT
#####################################################################################################################"""

def reset_hand(handID = None, handPos = [0, 0, -h_dist] , handOr = [0, 0, 0, 1],
               robotPath = object_path, fixed = True):
    """
    loads a new instance of the hand into the scene - this disrupts all current physics, so "resets" the scene
    if you send in the "handID", it will delete the old instance before populating a new one
    TODO: give the hand some sensitivity for grasping - force sensors for fingertips?
    """
    if handID is not None:
        p.removeBody(handID)
    hID = p.loadURDF(robot_path, basePosition=handPos, baseOrientation=handOr, useFixedBase=fixed, globalScaling=1)
    return hID

def reset_ob(obID= None, obPos = [0, 0, 0], fixed = True):
    """
    if you send in the "obID", it will delete the old instance before populating a new one
    orientation here is ignored because the hand orientation changes, so the object doesnt also have to rotate
    """
    if obID is not None:
        p.removeBody(obID)
    oID = p.loadURDF(object_path, obPos, globalScaling=1.5, useFixedBase=fixed)
    return oID

"""#####################################################################################################################
                                            HAND ORIENTATION + LOCATION
#####################################################################################################################"""

def hand_dist(cubeID, handID, pos, oren):
    """
    actually does tthe movement to have hand touch object to judge distance

    returns position of the hand when it touches the object
    """
    #print("reset hand for non-fixed base")
    reset_hand(handID, handPos=pos, handOr=oren, fixed=False)
    relax(handID) #want fingers splayed to get distance
    neg_pos = [-pos[0], -pos[1], -pos[2]]
    has_contact = 0
    while not has_contact: #while still distance between hand/object
        p.applyExternalForce(handID, 1, neg_pos, pos, p.WORLD_FRAME) #move hand toward object
        p.stepSimulation()
        contact_points = p.getContactPoints(handID, cubeID)  # get contact between cube and hand
        has_contact = len(contact_points) #any contact stops the loop
    t_pos, t_oren = p.getBasePositionAndOrientation(handID)
    p.removeBody(handID) #clean up
    return t_pos #only need the position of the object


def adjust_point_dist(theta_rad, phi_rad, hID, oID, carts, quat):
    """
    move the hand w/fingers splayed until it touches the object
    should touch in center/palm - this should be the best for an initial grasp

    returns set of position coordinates representing how far from the object the hand should be (touching + a margin)
    """

    touching_pos = hand_dist(oID, hID, carts, quat)
    t_dist = sqrt(touching_pos[0]**2+touching_pos[1]**2+touching_pos[2]**2)
    m_dist = t_dist + margin #add a small margin to the contact point to allow for legal grasps

    carts = astropy.coordinates.spherical_to_cartesian(m_dist, theta_rad, phi_rad)
    flip_carts = (-carts[0],-carts[1],-carts[2]) #associated coords have it facing away from the object - move to other side

    return flip_carts


def get_given_point(dist, theta_rad, phi_rad, hID, oID):

    """
    get the coords and the quat for the hand based on distance from origin and angles

    returns a single (position, orientation) pair

    For the transform3d euler to quat: (seems like their z is our x, their y is our y, their x is our z)
    #Rotate about the current z-axis by ϕ. Then, rotate about the new y-axis by θ
    """
    carts = astropy.coordinates.spherical_to_cartesian(dist, theta_rad, phi_rad)
    neg_carts = (-carts[0],-carts[1],-carts[2]) #associated coords have it facing away from the object - move to other side
    quat = euler.euler2quat(phi_rad + pi, pi / 2 - theta_rad, pi, axes='sxyz')  # the pi in the z brings it to face "up"

    #move the hand w/fingers splayed until it touches the object, that is the dist to try for a grip
    close_carts = adjust_point_dist(theta_rad, phi_rad, hID, oID, neg_carts, quat)

    return (close_carts, quat)

def circle_set(hID, cID, dist = h_dist, n = 20, theta = pi/4, phi = pi):
    """
    move the hand around the object in a reasonable way

    returns an array of (position, orientation) pairs
    """

    print("Giving points")
    #2 random #s between -pi/2 and pi/2
    theta_rad = -theta
    increment = 2*pi/n
    set = []
    for i in range(0,(n+1)):
        print(i)
        #TODO: dist here needs to be programmatic
        set.append(get_given_point(dist=dist, theta_rad=-theta, phi_rad=(-phi) + increment * i, hID=hID, oID=cID))

    return set

def get_rand_point(dist):
    """
    get a random point dist away from the origin and facing the object

    returns a single (position, orientation) pair

    For the transform3d euler to quat: (seems like their z is our x, their y is our y, their x is our z)
    #Rotate about the current z-axis by ϕ. Then, rotate about the new y-axis by θ
    """
    theta_rad, phi_rad = rand_coord()
    carts = astropy.coordinates.spherical_to_cartesian(dist, theta_rad, phi_rad)
    neg_carts = (-carts[0],-carts[1],-carts[2])
    quat = euler.euler2quat(phi_rad + pi, pi / 2 - theta_rad, pi, axes='sxyz')  # the pi in the z brings it to face "up"

    return (neg_carts, quat)

def rand_set(dist = h_dist, n = 10):
    """
    get n pairs for the hand randomly distributed dist away from the origin
    returns an array of (position, orientation) pairs
    """

    print("Getting set of random points")
    set = []
    for i in range(n):
        set.append(get_rand_point(h_dist))

    return set

def test_points(dist = h_dist):
    """
    some non-programmatic hand points ones that actually work
    returns an array of (position, orientation) pairs
    """
    test = [([dist, 0, 0], [0, -1, 0, 1]), ([-dist, 0, 0], [0, 1, 0, 1]), ([0, dist, 0], [1, 0, 0, 1]),
            ([0, -dist, 0], [-1, 0, 0, 1]), ([0, 0, -dist], [0, 0, 0, 1])]
    return test

"""#####################################################################################################################
                                            GRIPPER FUNCTIONS/MOVEMENT
#####################################################################################################################"""

def grasp(handId):
    """
    closes the gripper uniformly + attempts to find a grasp
    this is based on time + not contact points because contact points could just be a finger poking the object
    relies on grip_joints - specified by user/config file which joints should close
    TODO: make grasp mirror barret hand irl = that means make the base joint close before tip joint in hand
    """
    print("finding grasp")
    finish_time = time() + time_limit
    while time() < finish_time:
        p.stepSimulation()
        for joint in grip_joints:
            p.setJointMotorControl2(bodyUniqueId=handId, jointIndex=joint, controlMode=p.VELOCITY_CONTROL, targetVelocity=t_vel, force = maxForce)


def relax(handID):
    """
    return all joints to neutral/furthest extended, baseed on urdf specification
    """
    print("relaxing hand")
    joint = 0
    num = p.getNumJoints(handID)
    while joint < num:
        p.resetJointState(bodyUniqueId = handID, jointIndex = joint, targetValue = 0.0)
        joint = joint + 1


"""#####################################################################################################################
                                        POSITION/ORIENTATION DATA - GRAP MEMORY 
#####################################################################################################################"""

#TODO: edit this to more closely align with other standards

class Grasp:

    def __init__(self, position, orientation, joints):
        self.position = position
        self.orientation = orientation
        self.joints = joints

    def __repr__(self):
        return "Position: " + str(self.position) + " , Orientation: " + str(self.orientation) + " , Joints: " + str(self.joints) + " "

    def __str__(self):
        return "Position: " + str(self.position) + " , Orientation: " + str(self.orientation) + " , Joints: " + str(self.joints) + " "


def get_robot_config(handID):
    pos, oren = p.getBasePositionAndOrientation(handID)
    joints = {}
    num = p.getNumJoints(handID)
    for joint in range(0, num):
        joints[joint] = p.getJointState(handID, joint)
    return Grasp(pos, oren, joints)



"""#####################################################################################################################
                                            GRASP EVALUATION FUNCTIONS
#####################################################################################################################"""


def check_grip(cubeID, handID):
    #TODO: make direction of motion consistant (up and away from origin?)
    #TODO: modify to take in hand and cube position/orientation + load into environment w/gravity before shaking
    """
    check grip by adding in gravity
    """
    print("checking strength of current grip")
    mag = 1
    pos, oren = p.getBasePositionAndOrientation(handID)
    #pos, oren = p.getBasePositionAndOrientation(cubeID)
    time_limit = .5
    finish_time = time() + time_limit
    p.addUserDebugText("Grav Check!", [-.07, .07, .07], textColorRGB=[0, 0, 1], textSize=1)
    while time() < finish_time:
        p.stepSimulation()
        #add in "gravity"
        p.applyExternalForce(cubeID, linkIndex = -1, forceObj = [0, 0, -mag], posObj=pos, flags=p.WORLD_FRAME)
    contact = p.getContactPoints(cubeID,handID) #see if hand is still holding obj after gravity is applied
    print("contacts", contact)
    if len(contact) >0:
        p.removeAllUserDebugItems()
        p.addUserDebugText("Grav Check Passed!", [-.07, .07, .07], textColorRGB=[0, 1, 0], textSize=1)
        sleep(.3)
        print("Good Grip to Add")
        return get_robot_config(handID)

    else:
        p.removeAllUserDebugItems()
        p.addUserDebugText("Grav Check Failed!", [-.07, .07, .07], textColorRGB=[1, 0, 0], textSize=1)
        sleep(.3)
        return None

#TODO: other grasp measurement metrics - simulated annealing, etc


"""#####################################################################################################################
                                        MAIN MAIN MAIN MAIN MAIN MAIN
#####################################################################################################################"""

print("grasp!")
handID = reset_hand()
cubeID = reset_ob()
#floorID = p.loadURDF("plane.urdf", [0, 0, -2]) #the ground!


#hand_set = test_points()
#hand_set = rand_set()
hand_set = circle_set(hID = handID, cID = cubeID, n=5)
print(hand_set)

handID = reset_hand()
cubeID = reset_ob(cubeID, [0,0,0])

good_grips = []


pos = 0
for each in hand_set:
    print("position #: ", pos)

    sleep(.2)
    relax(handID)
    p.resetBasePositionAndOrientation(handID, each[0], each[1])
    cubeID = reset_ob(cubeID, [0, 0, 0], fixed=False)
    grasp(handID)
    sleep(.01)
    good_grips.append(check_grip(cubeID, handID))
    p.removeAllUserDebugItems()
    pos +=1


print("Num Good Grips: ", len(good_grips))
print("Grips:")
for grip in good_grips:
    print(grip)



"""#####################################################################################################################
                                       USEFUL INFORMATION, MAYBE 
########################################################################################################################


Num joints:  11
joint # 3
(3, b'bh_j32_joint', 0, 7, 6, 1, 100.0, 1.0, 0.0, 2.44, 30.0, 2.0, b'bh_finger_32_link', (0.0, 0.0, -1.0), (-0.0040000006556510925, 0.0, 0.033900000154972076), (-0.7071080610451548, 3.121199146877591e-17, -3.12121044560798e-17, 0.7071055013256238), 2)
joint # 6
(6, b'bh_j12_joint', 0, 10, 9, 1, 100.0, 1.0, 0.0, 2.44, 30.0, 2.0, b'bh_finger_12_link', (0.0, 0.0, -1.0), (-0.0040000006556510925, 0.0, 0.033900000154972076), (-0.7071080610451548, 0.0, 0.0, 0.7071055013256238), 5)
joint # 9
(9, b'bh_j22_joint', 0, 13, 12, 1, 100.0, 1.0, 0.0, 2.44, 30.0, 2.0, b'bh_finger_22_link', (0.0, 0.0, -1.0), (-0.0040000006556510925, 0.0, 0.033900000154972076), (-0.7071080610451548, 3.121199146877591e-17, -3.12121044560798e-17, 0.7071055013256238), 8)


joint lower limit: 0.0
joint upper limit: 2.44

(0, b'hand_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'hand_base_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), -1)
(1, b'bh_base_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'bh_base_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0)
(2, b'bh_j31_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'bh_finger_31_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.05040000006556511), (0.0, 0.0, 0.7071067966408574, 0.7071067657322373), 1)
(3, b'bh_j32_joint', 0, 7, 6, 1, 100.0, 1.0, 0.0, 2.44, 30.0, 2.0, b'bh_finger_32_link', (0.0, 0.0, -1.0), (-0.008000001311302185, 0.0, 0.06780000030994415), (-0.7071080610451548, 3.121199146877591e-17, -3.12121044560798e-17, 0.7071055013256238), 2)
(4, b'bh_j33_joint', 0, 8, 7, 1, 100.0, 1.0, 0.0, 0.84, 30.0, 2.0, b'bh_finger_33_link', (0.0, 0.0, -1.0), (-0.1398719996213913, 0.006000000052154064, 0.0), (0.0, 0.0, 0.0, 1.0), 3)
(5, b'bh_j11_joint', 0, 9, 8, 1, 100.0, 1.0, 0.0, 3.1416, 30.0, 2.0, b'bh_finger_11_link', (0.0, 0.0, -1.0), (-0.05000000074505806, 0.0, 0.05040000006556511), (0.0, 0.0, -0.7071080610451548, 0.7071055013256238), 1)
(6, b'bh_j12_joint', 0, 10, 9, 1, 100.0, 1.0, 0.0, 2.44, 30.0, 2.0, b'bh_finger_12_link', (0.0, 0.0, -1.0), (-0.008000001311302185, 0.0, 0.06780000030994415), (-0.7071080610451548, 0.0, 0.0, 0.7071055013256238), 5)
(7, b'bh_j13_joint', 0, 11, 10, 1, 100.0, 1.0, 0.0, 0.84, 30.0, 2.0, b'bh_finger_13_link', (0.0, 0.0, -1.0), (-0.1398719996213913, 0.006000000052154064, 0.0), (0.0, 0.0, 0.0, 1.0), 6)
(8, b'bh_j21_joint', 0, 12, 11, 1, 100.0, 1.0, 0.0, 3.1416, 30.0, 2.0, b'bh_finger_21_link', (0.0, 0.0, 1.0), (0.05000000074505806, 0.0, 0.05040000006556511), (0.0, 0.0, -0.7071080610451548, 0.7071055013256238), 1)
(9, b'bh_j22_joint', 0, 13, 12, 1, 100.0, 1.0, 0.0, 2.44, 30.0, 2.0, b'bh_finger_22_link', (0.0, 0.0, -1.0), (-0.008000001311302185, 0.0, 0.06780000030994415), (-0.7071080610451548, 3.121199146877591e-17, -3.12121044560798e-17, 0.7071055013256238), 8)
(10, b'bh_j23_joint', 0, 14, 13, 1, 100.0, 1.0, 0.0, 0.84, 30.0, 2.0, b'bh_finger_23_link', (0.0, 0.0, -1.0), (-0.1398719996213913, 0.006000000052154064, 0.0), (0.0, 0.0, 0.0, 1.0), 9)


points = points_on_circumference(center=(0, 0), r=.15, n=5)


for each in points:
    p.resetBasePositionAndOrientation(handID, [each[0], each[1], 0], [0, 0, 0, 1])

planeId = p.loadURDF("plane.urdf", [0, 0, -0.5]) #the ground!


"""

