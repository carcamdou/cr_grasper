import xmltodict
from collections import namedtuple
import numpy as np
import rospy
import roslib

Pose = namedtuple('Pose', ['xyz', 'rpy'], verbose=True)

def get_pose_array(filter_array, palm, rpy=[0,0,0]):
	pa = []

	for i in range(len(filter_array)):
		loc_string = filter_array[i]['@params'].split(',')
		loc = np.array(loc_string, dtype=float)

		xyz = np.mean(loc.reshape((2,3)),0)/1000
		if palm:
			#0.08 eyeballed - could be entirely wrong
			#Based off of bh_base_link transform
			xyz = [xyz[0], xyz[1], 0.075]
		else:
			xyz = [xyz[1], xyz[0], xyz[2]]

		pa.append(Pose(xyz=xyz, rpy=rpy))
		
	return pa

def get_path(package_name, resource_name):
    resources = roslib.packages.find_resource(package_name, resource_name)
    if len(resources) == 0:
        rospy.logerr("Failed to find resource %s in package %s"%(resource_name, package_name))
        return ""
    else:
		return resources[0]

tactile_filename = get_path('graspit', 'BarrettBH8_280_Tactile.xml')

with open(tactile_filename) as fd:
    doc = xmltodict.parse(fd.read())

palm_sensors = get_pose_array(doc['robot']['filter'], True)

barrett_sensors = {}
barrett_sensors['palm'] = palm_sensors

for index, chain_num in enumerate(range(len(doc['robot']['chain'])), 1):

	finger_sensors = get_pose_array(doc['robot']['chain'][chain_num]['filter'], False)
	barrett_sensors['link{}'.format(index)] = finger_sensors


text_poses = ""
for i, pose in enumerate(barrett_sensors['palm'], 1):
	text_poses += '<joint name="${{name}}_palm_sensor{}_joint" type="fixed">\n'.format(i)
	text_poses += '\t<origin xyz="{} {} {}" rpy="{} {} {}"/>\n'.format(pose.xyz[0], pose.xyz[1], pose.xyz[2], pose.rpy[0], pose.rpy[1], pose.rpy[2])
	text_poses += '\t<parent link="${name}_base_link"/>\n'
	text_poses += '\t<child link="${{name}}_palm_sensor_{}_link"/>\n'.format(i)
	text_poses += '</joint>\n'
	text_poses += '<link name="${{name}}_palm_sensor_{}_link"/>\n'.format(i)

for link_num in range(1, 4):
	for i, pose in enumerate(barrett_sensors['link{}'.format(link_num)], 1):	
		text_poses += '<joint name="${{name}}_link{}_sensor{}_joint" type="fixed">\n'.format(link_num, i)
		text_poses += '\t<origin xyz="{} {} {}" rpy="{} {} {}"/>\n'.format(pose.xyz[0], pose.xyz[1], pose.xyz[2], pose.rpy[0], pose.rpy[1], pose.rpy[2])
		text_poses += '\t<parent link="bh_finger_{}4_link"/>\n'.format(link_num)
		text_poses += '\t<child link="${{name}}_link{}_sensor{}_link"/>\n'.format(link_num, i)
		text_poses += '</joint>\n'
		text_poses += '<link name="${{name}}_link{}_sensor{}_link"/>\n'.format(link_num, i)


import pyperclip
pyperclip.copy(text_poses)