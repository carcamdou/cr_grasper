import pybullet as p

physicsClient = p.connect(p.DIRECT)  #p.DIRECT for non-graphical version

rID = p.loadURDF("/Users/carlyndougherty/PycharmProjects/cr_grasper/RobotURDFs/finger_description/urdf/RH8D.urdf")

jointNum = p.getNumJoints(rID)

for joint in range(0,jointNum):
    print(p.getJointInfo(rID, joint))
