import pybullet as p
import time
import math

p.connect(p.GUI)

p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/pybullet_examples/bullet3/data/plane.urdf")
cubeId = p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/RobotURDFs/barrett_hand_description/urdf/bh.urdf",0,0,1)
p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
cid = p.createConstraint(cubeId,-1,-1,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,1])
p.changeConstraint(cid,pivot,jointChildFrameOrientation=orn, maxForce=50)
#print (cid)
#print (p.getConstraintUniqueId(0))
prev = [0,0,1]
a = -math.pi
while 1:
	a=a+0.01
	if (a>math.pi):
		a=-math.pi
	time.sleep(.01)
	p.setGravity(0,0,-10)
	pivot=[a,0,1]
	orn = p.getQuaternionFromEuler([a,0,0])
	p.changeConstraint(cid,pivot,jointChildFrameOrientation=orn, maxForce=50)

p.removeConstraint(cid)