#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy 
from pyHand_api import *

from std_msgs.msg import String
from bhand_controller.msg import State, TactileArray, Service
from bhand_controller.srv import Actions, SetControlMode
from sensor_msgs.msg import JointState

import time, threading
import math

DEFAULT_FREQ = 100.0
MAX_FREQ = 250.0


CONTROL_MODE_POSITION = "POSITION"
CONTROL_MODE_VELOCITY = "VELOCITY"

WATCHDOG_VELOCITY = 0.1	# Max time (seconds) between velocity commands before stopping the velocity

# class to save the info of each joint
class JointHand:
	
	def __init__(self, id, name):
		self.id = id
		self.joint_name = name
		self.desired_position = 0.0
		self.desired_velocity = 0.0
		self.real_position = 0.0
		self.real_velocity = 0.0
		
	
# Class control the Barrett Hand
class BHand:
	
	def __init__(self, args):
		
		self.port = args['port']
		self.topic_state_name = args['topic_state']
		self.desired_freq = args['desired_freq'] 
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(rospy.get_name(), self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
		self.tactile_sensors = args['tactile_sensors'] 
		self.real_freq = 0.0
		
		joint_ids_arg = args['joint_ids']
		joint_names_arg = args['joint_names']
		# Control mode
		self.control_mode = args['control_mode']
		if self.control_mode != CONTROL_MODE_POSITION and self.control_mode != CONTROL_MODE_VELOCITY:
			rospy.loginfo('%s: init: setting control mode by default to %s'%(rospy.get_name(), CONTROL_MODE_POSITION))
			self.control_mode = CONTROL_MODE_POSITION
		
		self.state = State.INIT_STATE
		# flag to control the initialization of CAN device 
		self.can_initialized = False
		# flag to control the initialization of the hand
		self.hand_initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		
		self.time_sleep = 1.0
		# BHand library object
		self.hand = pyHand(self.port)
		# Current state of the joints
		self.joint_state = JointState()
		# Msg to publish the tactile array
		self.msg_tact_array = TactileArray()
		# State msg to publish
		self.msg_state = State()
		# Timer to publish state
		self.publish_state_timer = 1
		# Flag active after receiving a new command
		self.new_command = False
		# Flag active for reading temperature
		self.temp_command = False
		# Used to read one temperature sensor every time
		self.temp_current_sensor = 0
		# Total number of temperature sensors 
		self.temp_sensor_number = 8
		# Frequency to read temperature
		self.temp_read_timer = 5# seconds
		# Counts the number of CAN errors before switching to FAILURE
		self.can_errors = 0
		# Max. number of CAN errors 
		self.max_can_errors = 5
		# Flag to perform a failure recover
		self.failure_recover = False
		# Timer to try the FAILURE RECOVER
		self.failure_recover_timer = 5	
		# Timer to save the last command 
		self.timer_command = time.time()
		self.watchdog_command = float(WATCHDOG_VELOCITY)
		
		# List where saving the required actions
		self.actions_list = []
		# Mode of the hand to perform the grasp
		self.grasp_mode = Service.SET_GRASP_1
		
		#print 'Control mode = %s'%self.control_mode
		
		# Data structure to link assigned joint names with every real joint
		# {'ID': ['joint_name', index], ...}
		#self.joint_names = {'F1': ['j12_joint',  0], 'F1_TIP': ['j13_joint', 1] , 'F2': ['j22_joint', 2], 'F2_TIP': ['j23_joint', 3],  'F3': ['j32_joint', 4], 'F3_TIP': ['j33_joint', 5], 'SPREAD_1': ['j11_joint', 6], 'SPREAD_2': ['j21_joint', 7]}
		# {'ID': ['joint_name'], ...}
		#'j33_joint' -> F3-Fingertip
		#'j32_joint' -> F3-Base
		#
		#'j11_joint' -> F1-Spread
		#'j12_joint' -> F1-Base
		#'j13_joint' -> F1-Fingertip
		#
		#'j21_joint' -> F2-Spread
		#'j22_joint' -> F2-Base
		#'j23_joint' -> F2-Fingertip
		
		self.joint_names = {}
		
		for i in range(len(joint_ids_arg)):
			self.joint_names[joint_ids_arg[i]] = [joint_names_arg[i], i]
		
		print 	self.joint_names
		
		# {'ID': [Puck_temp, motor_temp], ..}
		self.temperatures = {'F1': [0.0,  0.0], 'F2': [0.0,  0.0], 'F3': [0.0,  0.0], 'SPREAD': [0.0,  0.0]}
		
		# Inits joint state (FOR PUBLISHING)
		k = 0
		for i in self.joint_names:
			self.joint_state.name.append(self.joint_names[i][0])
			self.joint_names[i][1] = k # Updates the index
			k = k + 1
			#self.joint_state.name = ['j33_joint', 'j32_joint', 'j21_joint', 'j22_joint', 'j23_joint', 'j11_joint', 'j12_joint', 'j13_joint']
			self.joint_state.position.append(0.0)
			self.joint_state.velocity.append(0.0)
			self.joint_state.effort.append(0.0)
		
		#  Saves the desired position of the joints {'joint_name': value, ...}
		self.desired_joints_position = {self.joint_names['F1'][0]: 0.0, self.joint_names['F2'][0]: 0.0, self.joint_names['F3'][0]: 0.0, self.joint_names['SPREAD_1'][0]: 0.0, self.joint_names['SPREAD_2'][0]: 0.0}
		self.desired_joints_velocity = {self.joint_names['F1'][0]: 0.0, self.joint_names['F2'][0]: 0.0, self.joint_names['F3'][0]: 0.0, self.joint_names['SPREAD_1'][0]: 0.0, self.joint_names['SPREAD_2'][0]: 0.0}
		# Saves the last sent velocity
		self.sent_joints_velocity = {self.joint_names['F1'][0]: 0.0, self.joint_names['F2'][0]: 0.0, self.joint_names['F3'][0]: 0.0, self.joint_names['SPREAD_1'][0]: 0.0, self.joint_names['SPREAD_2'][0]: 0.0}
		
		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_read_temp = threading.Timer(self.temp_read_timer, self.readTemp)
		
		rospy.loginfo('%s: port %s, freq = %d, topic = %s, tactile_sensors = %s'%(rospy.get_name() , self.port, self.desired_freq, self.topic_state_name, self.tactile_sensors))
	
	
	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		if self.can_initialized or self.hand.initialize() == True:
			self.can_initialized = True
			return 0
		else:
			self.hand.can_uninit()
			rospy.logerr('%s: Error initializing the hand'%rospy.get_name() )
			return -1
		
		
	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0
			
		#self._enable_disable_service = rospy.Service('%s/enable_disable'%rospy.get_name(), enable_disable, self.enableDisable)
		# PUBLISHERS
		self._state_publisher = rospy.Publisher('%s/state'%rospy.get_name(), State, queue_size=5)
		self._joints_publisher = rospy.Publisher('/joint_states', JointState, queue_size=5)
		self._tact_array_publisher = rospy.Publisher('%s/tact_array'%rospy.get_name(), TactileArray, queue_size=5)
		# SUBSCRIBERS
		self._joints_subscriber = rospy.Subscriber('%s/command'%rospy.get_name(), JointState, self.commandCallback)
		
		# SERVICES
		self._hand_service = rospy.Service('%s/actions'%rospy.get_name(), Actions, self.handActions)
		self._set_control_mode_service = rospy.Service('%s/set_control_mode'%rospy.get_name(), SetControlMode, self.setControlModeServiceCb)
	
		self.ros_initialized = True
		
		self.publishROSstate()
		
		return 0
		
		
	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.can_initialized:
			return -1
		rospy.loginfo('%s::shutdown'%rospy.get_name())
		self.setControlMode('POSITION')
		# Performs Hand shutdown
		self.hand.can_uninit()
		# Cancels current timers
		self.t_publish_state.cancel()
		self.t_read_temp.cancel()
		
		
		self.can_initialized = False
		self.hand_initialized = False
		
		return 0
	
	
	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1
		
		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()
		self._joints_publisher.unregister()
		self._tact_array_publisher.unregister()
		self._joints_subscriber.unregister()
		self._set_control_mode_service.shutdown()
		self._hand_service.shutdown()
		
		self.ros_initialized = False
		
		return 0
			
	
	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False
		
		return 0
	
	
	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()
		
		if self.running:
			return 0
			
		self.running = True
		
		self.controlLoop()
		
		return 0
	
	
	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''
		
		while self.running and not rospy.is_shutdown():
			t1 = time.time()
			
			if self.state == State.INIT_STATE:
				self.initState()
				
			elif self.state == State.STANDBY_STATE:
				self.standbyState()
				
			elif self.state == State.READY_STATE:
				self.readyState()
				
			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()
				
			elif self.state == State.FAILURE_STATE:
				self.failureState()
				
			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()
				
			self.allState()
			
			t2 = time.time()
			tdiff = (t2 - t1)
			
			
			t_sleep = self.time_sleep - tdiff
			#print 't_sleep = %f secs'%(t_sleep)
			if t_sleep > 0.0:
				#print 't_sleep = %f secs'%(self.time_sleep)
				rospy.sleep(t_sleep)
			
			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)
		
		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%rospy.get_name())
		
		return 0
		
	
	
	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
		#self._state_publisher.publish(self.getState())
		
		
		t = rospy.Time.now()
		
		self.joint_state.header.stamp = t
		self._joints_publisher.publish(self.joint_state)
		
		if self.tactile_sensors:
			self.msg_tact_array.header.stamp = t
			self._tact_array_publisher.publish(self.msg_tact_array)
			
		return 0
		
	
	
	def initState(self):
		'''
			Actions performed in init state
		'''
		error = 0
		if not self.can_initialized:
			ret = self.setup()
			if ret == -1:
				error = 1
				
		if self.hand_initialized and self.can_initialized:
			self.switchToState(State.STANDBY_STATE)
		
		self.canError(error)
		
		return
	
	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		#self.hand.close_grasp()
		self.switchToState(State.READY_STATE)
		
		return
	
	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		f1_index = self.joint_names['F1'][1]
		f2_index = self.joint_names['F2'][1]
		f3_index = self.joint_names['F3'][1]
		f1_tip_index = self.joint_names['F1_TIP'][1]
		f2_tip_index = self.joint_names['F2_TIP'][1]
		f3_tip_index = self.joint_names['F3_TIP'][1]
		spread1_index = self.joint_names['SPREAD_1'][1]
		spread2_index = self.joint_names['SPREAD_2'][1]
		errors = 0
		
		
		try:
			# Reads position
			self.hand.read_packed_position(SPREAD)
			self.hand.read_packed_position(FINGER1)
			self.hand.read_packed_position(FINGER2)
			self.hand.read_packed_position(FINGER3)
			
			# Reads strain
			self.hand.read_strain(FINGER1)
			self.hand.read_strain(FINGER2)
			self.hand.read_strain(FINGER3)
			
			# Reads temperature
			if self.temp_command:
				self.hand.read_temp(FINGER1)
				self.hand.read_temp(FINGER2)
				self.hand.read_temp(FINGER3)
				self.hand.read_temp(SPREAD)
				self.hand.read_therm(FINGER1)
				self.hand.read_therm(FINGER2)
				self.hand.read_therm(FINGER3)
				self.hand.read_therm(SPREAD)
				self.initReadTempTimer()
			
			# Reads tactile sensor cells
			if self.tactile_sensors:
				self.hand.read_full_tact(SPREAD)
				self.hand.read_full_tact(FINGER1)
				self.hand.read_full_tact(FINGER2)
				self.hand.read_full_tact(FINGER3)
				
				
		except Exception, e:
			rospy.logerr('%s::readyState: error getting info: %s'%(rospy.get_name(), e))
			errors = errors + 1
		
		# Predefined actions
		if len(self.actions_list) > 0:
			action = self.actions_list[0]
			# Removes action from list 
			self.actions_list.remove(action)
			
			# Actions performed in CONTROL POSITION
			if self.control_mode == CONTROL_MODE_VELOCITY:
				self.setControlMode(CONTROL_MODE_POSITION)
			
			if action == Service.CLOSE_GRASP:
				self.closeFingers(3.14)
			
			if action == Service.CLOSE_HALF_GRASP:
				self.closeFingers(1.57)
				
			elif action == Service.OPEN_GRASP:
				self.openFingers()
				
			elif action == Service.SET_GRASP_1:
				if self.joint_state.position[f1_index] > 0.1 or self.joint_state.position[f2_index] > 0.1 or self.joint_state.position[f3_index] > 0.1:
					#rospy.logerr('BHand::ReadyState: Service SET_GRASP 1 cannot be performed. Rest of fingers have to be on zero position')
					self.openFingers()
					time.sleep(2.0)
				
				self.desired_joints_position['SPREAD_1'] = 0.0
				self.desired_joints_position['SPREAD_2'] = 0.0
				self.hand.move_to(SPREAD, self.hand.rad_to_enc(self.desired_joints_position['SPREAD_1'], BASE_TYPE), False)
				self.grasp_mode = action
					
			elif action == Service.SET_GRASP_2:
				if self.joint_state.position[f1_index] > 0.1 or self.joint_state.position[f2_index] > 0.1 or self.joint_state.position[f3_index] > 0.1:
					#rospy.logerr('BHand::ReadyState: Service SET_GRASP 2 cannot be performed. Rest of fingers have to be on zero position')
					self.openFingers()
					time.sleep(2.0)
				#else:
				self.desired_joints_position['SPREAD_1'] = 3.14
				self.desired_joints_position['SPREAD_2'] = 3.14
				self.hand.move_to(SPREAD, self.hand.rad_to_enc(self.desired_joints_position['SPREAD_1'], BASE_TYPE), False)
				self.grasp_mode = action
			
		# NO pre-defined actions			
		else: 
			if self.control_mode == CONTROL_MODE_POSITION:
			
				# Moves joints to desired pos
				if self.new_command:
					try:
						f1_joint = self.joint_names['F1'][0]		
						if self.desired_joints_position[f1_joint] != self.joint_state.position[f1_index]:
							#print 'moving joint %s to position  %f'%(self.joint_state.name[0], self.desired_joints_position['j12_joint'])
							self.hand.move_to(FINGER1, self.hand.rad_to_enc(self.desired_joints_position[f1_joint], BASE_TYPE), False)
						
						f2_joint = self.joint_names['F2'][0]
						if self.desired_joints_position[f2_joint] != self.joint_state.position[f2_index]:
							#print 'moving joint %s to position  %f'%(self.joint_state.name[2], self.desired_joints_position['j22_joint'])
							self.hand.move_to(FINGER2, self.hand.rad_to_enc(self.desired_joints_position[f2_joint], BASE_TYPE), False)
							
						f3_joint = self.joint_names['F3'][0]
						if self.desired_joints_position[f3_joint] != self.joint_state.position[f3_index]:
							#print 'moving joint %s to position  %f'%(self.joint_state.name[4], self.desired_joints_position['j32_joint'])
							self.hand.move_to(FINGER3, self.hand.rad_to_enc(self.desired_joints_position[f3_joint], BASE_TYPE), False)
							
						spread1_joint = self.joint_names['SPREAD_1'][0]
						spread2_joint = self.joint_names['SPREAD_2'][0]
						if self.desired_joints_position[spread1_joint] != self.joint_state.position[spread1_index]:
							#print 'moving joint %s to position  %f'%(self.joint_state.name[6], self.desired_joints_position['jspread_joint'])
							self.hand.move_to(SPREAD, self.hand.rad_to_enc(self.desired_joints_position[spread1_joint], SPREAD_TYPE), False)
						elif self.desired_joints_position[spread2_joint] != self.joint_state.position[spread2_index]:
							#print 'moving joint %s to position  %f'%(self.joint_state.name[6], self.desired_joints_position['jspread_joint'])
							self.hand.move_to(SPREAD, self.hand.rad_to_enc(self.desired_joints_position[spread2_joint], SPREAD_TYPE), False)
						
					except Exception, e:
						rospy.logerr('%s::readyState: error sending command: %s'%(rospy.get_name(), e))
						errors = errors + 1
					
					self.new_command = False
			
			else:
				# VELOCITY CONTROL
				if ((time.time() - self.timer_command) >= self.watchdog_command):
					try:
						#rospy.loginfo('BHand::readyState: Watchdog velocity')
						self.desired_joints_velocity[self.joint_names['F1'][0]] = 0.0
						self.desired_joints_velocity[self.joint_names['F2'][0]] = 0.0
						self.desired_joints_velocity[self.joint_names['F3'][0]] = 0.0
						self.desired_joints_velocity[self.joint_names['SPREAD_1'][0]] = 0.0
						self.desired_joints_velocity[self.joint_names['SPREAD_2'][0]] = 0.0
						self.setJointVelocity('F1')
						self.setJointVelocity('F2')
						self.setJointVelocity('F3')
						self.setJointVelocity('SPREAD_1')
					except Exception, e:
						rospy.logerr('%s::readyState: error sending command: %s'%(rospy.get_name(), e))
						errors = errors + 1
				# Moves joints to desired pos
				if self.new_command:
			
					try:
						self.setJointVelocity('F1')
						self.setJointVelocity('F2')
						self.setJointVelocity('F3')
						self.setJointVelocity('SPREAD_1')
						
						
					except Exception, e:
						rospy.logerr('%s::readyState: error sending command: %s'%(rospy.get_name(), e))
						errors = errors + 1
					
					self.new_command = False
			
		time.sleep(0.002)
		
		# Reads messages from CAN bus
		if self.hand.process_can_messages() != 0:
			# If it doesn't read any msg, counts as error
			self.canError(1)
		
		# Updating variables
		# Position
		self.joint_state.position[f1_index], self.joint_state.position[f1_tip_index] = self.hand.get_packed_position(FINGER1)
		self.joint_state.position[f2_index], self.joint_state.position[f2_tip_index] = self.hand.get_packed_position(FINGER2)
		self.joint_state.position[f3_index], self.joint_state.position[f3_tip_index] = self.hand.get_packed_position(FINGER3)
		self.joint_state.position[spread1_index], e = self.hand.get_packed_position(SPREAD)
		self.joint_state.position[spread2_index] = self.joint_state.position[spread1_index]
		
		# Strain
		self.joint_state.effort[f1_index] = self.joint_state.effort[f1_tip_index] = self.strain_to_nm(self.hand.get_strain(FINGER1))
		self.joint_state.effort[f2_index] = self.joint_state.effort[f2_tip_index] = self.strain_to_nm(self.hand.get_strain(FINGER2))
		self.joint_state.effort[f3_index] = self.joint_state.effort[f3_tip_index] = self.strain_to_nm(self.hand.get_strain(FINGER3))
		
		# Temperature
		self.msg_state.temp_f1[1] = self.hand.get_temp(FINGER1)
		self.msg_state.temp_f2[1] = self.hand.get_temp(FINGER2)
		self.msg_state.temp_f3[1] = self.hand.get_temp(FINGER3)
		self.msg_state.temp_spread[1] = self.hand.get_temp(SPREAD)
		self.msg_state.temp_f1[0] = self.hand.get_therm(FINGER1)
		self.msg_state.temp_f2[0] = self.hand.get_therm(FINGER2)
		self.msg_state.temp_f3[0] = self.hand.get_therm(FINGER3)
		self.msg_state.temp_spread[0] = self.hand.get_therm(SPREAD)
		
		# Tactile Sensors
		self.msg_tact_array.finger1 = self.hand.get_full_tact(FINGER1)
		self.msg_tact_array.finger2 = self.hand.get_full_tact(FINGER2)
		self.msg_tact_array.finger3 = self.hand.get_full_tact(FINGER3)
		self.msg_tact_array.palm = self.hand.get_full_tact(SPREAD)
		
		'''	e21, e22 = self.hand.get_packed_position(FINGER2, 1)
			e31, e32 = self.hand.get_packed_position(FINGER3, 1)
			espread, e_ = self.hand.get_packed_position(SPREAD, 1)
			
			self.joint_state.position[f2_index] = self.hand.enc_to_rad(e21, BASE_TYPE)
			self.joint_state.position[f2_tip_index] = self.hand.enc_to_rad(e22, TIP_TYPE)
			self.joint_state.position[f3_index] = self.hand.enc_to_rad(e31, BASE_TYPE)
			self.joint_state.position[f3_tip_index] = self.hand.enc_to_rad(e32, TIP_TYPE)
			self.joint_state.position[spread1_index] = self.joint_state.position[spread2_index] = self.hand.enc_to_rad(espread, SPREAD_TYPE)
		'''
		#print 'End loop'
		# Check the CAN bus status
		self.canError(errors)
		
		return
	
	
	def setJointVelocity(self, finger):
		'''
			Sets the joint velocity of the desired joint
			Takes the velocity value from the attribute desired_joints_velocity
			@param joint: finger of the hand (F1, F2, F3, SPREAD)
			@type joint: string
		'''
		if self.joint_names.has_key(finger):
			joint = self.joint_names[finger][0]	
						
			if self.desired_joints_velocity[joint] != self.sent_joints_velocity[joint]:
					
				value = self.hand.rad_to_enc(self.desired_joints_velocity[joint], BASE_TYPE) / 1000
				self.sent_joints_velocity[joint] = self.desired_joints_velocity[joint]	# Saves the last sent ref 
				#print 'Moving Joint %s at %f rad/s (%d conts/sec)'%(joint, self.desired_joints_velocity[joint], value)
				
				if finger == 'F1':
					self.hand.set_velocity(FINGER1, value)
				elif finger == 'F2':
					self.hand.set_velocity(FINGER2, value)
				elif finger == 'F3':
					self.hand.set_velocity(FINGER3, value)
				elif finger == 'SPREAD_1' or finger == 'SPREAD_2':
					self.hand.set_velocity(SPREAD, value)
						
	
	def shutdownState(self):
		'''
			Actions performed in shutdown state 
		'''
		print 'shutdownState'
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)
		
		return
	
	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''
		
		return
	
	def failureState(self):
		'''
			Actions performed in failure state
		'''
		if self.failure_recover:
			# Performs Hand shutdown
			self.hand.can_uninit()
			self.can_initialized = False
			self.switchToState(State.INIT_STATE)
			self.failure_recover = False
			self.can_errors = 0
			
		return
	
	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.state = new_state
			rospy.loginfo('BHand::switchToState: %s', self.stateToString(self.state))
			
			if self.state == State.READY_STATE:
				# On ready state it sets the desired freq.
				self.time_sleep = 1.0 / self.desired_freq
				self.initReadTempTimer()
			elif self.state == State.FAILURE_STATE:
				# On other states it set 1 HZ frequency
				self.time_sleep = 1.0
				self.initFailureRecoverTimer(self.failure_recover_timer)
			else:
				# On other states it set 1 HZ frequency
				self.time_sleep = 1.0
		
		return
		
	def allState(self):
		'''
			Actions performed in all states
		'''
		self.rosPublish()
		
		return
	
	
	def stateToString(self, state):
		'''
			@param state: state to convert
			@type state: bhand_controller.msg.State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'
				
		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'
			
		elif state == State.READY_STATE:
			return 'READY_STATE'
			
		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'
			
		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'
			
		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'
	
	
	def actionsToString(self, action):
		'''
			@param action: action to convert
			@type state: bhand_controller.msg.Service
			@returns the equivalent string of the action
		'''
		if action == Service.INIT_HAND:
			return 'INIT_HAND'
				
		elif action == Service.CLOSE_GRASP:
			return 'CLOSE_GRASP'
			
		elif action == Service.OPEN_GRASP:
			return 'OPEN_GRASP'
			
		elif action == Service.SET_GRASP_1:
			return 'SET_GRASP_1'
			
		elif action == Service.SET_GRASP_2:
			return 'SET_GRASP_2'
		
		elif action == Service.CLOSE_HALF_GRASP:
			return 'CLOSE_HALF_GRASP'
			
		else:
			return 'UNKNOWN_ACTION'
	
	
	def commandCallback(self, data):
		'''
			Function called when receive a new value
			@param data: state to convert
			@type data: sensor_msgs.JointState
		'''
		bingo = False
		
		for i in range(len(data.name)):
			if self.desired_joints_position.has_key(data.name[i]):
				self.desired_joints_position[data.name[i]] = data.position[i]
				self.desired_joints_velocity[data.name[i]] = data.velocity[i]
				bingo = True
				
		
		if bingo:
			self.new_command = True
			self.timer_command = time.time()
		#print 'commandCallback'
	
	
	def readTemp(self):
		'''
			Function periodically to enable the read of temperature
		'''
		
		self.temp_current_sensor = self.temp_current_sensor + 1
		
		if self.temp_current_sensor > self.temp_sensor_number:
			self.temp_current_sensor = 1
		
		self.temp_command = True
		
		
	def initReadTempTimer(self):
		'''
			Function to start the timer to read the temperature
		'''
		self.temp_command = False
		
		self.t_read_temp = threading.Timer(self.temp_read_timer, self.readTemp)
		self.t_read_temp.start()
	
		
	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state = self.state
		self.msg_state.state_description = self.stateToString(self.state)
		self.msg_state.desired_freq = self.desired_freq
		self.msg_state.real_freq = self.real_freq
		self.msg_state.hand_initialized = self.hand_initialized 
		self.msg_state.control_mode = self.control_mode 
		self._state_publisher.publish(self.msg_state)
		
		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()
	
	
	def strain_to_nm(self, x):
		'''
		Converts from raw encoder unit reading to Newton-meters.

		@param x: value to convert

		@returns nm: the torque value in Newton-meters. 
		'''
		p1 = 2.754e-10
		p2 = -1.708e-06
		p3 = 0.003764
		p4 = -2.85
		nm= p1*x**3 + p2*x**2 + p3*x + p4
		
		return nm
		
		
	def handActions(self, req):
		'''
		Handles the callback to Actions ROS service. Allows a set of predefined actions like init_hand, close_grasp, ...

		@param req: action id to perform
		@type req: bhand_controller.srv.Actions

		@returns: True or false depending on the result
		'''
		if req.action == Service.INIT_HAND:
			if self.state != State.INIT_STATE:
				rospy.logerr('%s::handActions: action INIT_HAND not allowed in state %s'%(rospy.get_name() ,self.stateToString(self.state)))
				
				return False
			else:
				ret = self.hand.init_hand()
				if ret:
					self.hand_initialized = True
					rospy.loginfo('%s::handActions: INIT HAND service OK'%rospy.get_name() )
					return True
				else:
					rospy.logerr('%s::handActions: error on INIT_HAND service'%rospy.get_name() )
					# Set the max number of errors to perform the CAN reinitialization
					self.canError(self.max_can_errors + 1)
					return False
			
		if self.state != State.READY_STATE:
			rospy.logerr('%s::handActions: action not allowed in state %s'%(rospy.get_name(), self.stateToString(self.state)))
			return False
		else:
			rospy.loginfo('%s::handActions: Received new action %s'%(rospy.get_name(), self.actionsToString(req.action)))
			self.actions_list.append(req.action)
			
		return True


	def canError(self, n):
		'''
		Manages the CAN errors ocurred

		@param n: number of produced
		@type n: int

		@returns: True or false depending on the result
		'''
		
		self.can_errors = self.can_errors + n	
		
		#rospy.loginfo('%s::canError: %d errors on CAN bus'%(rospy.get_name(),self.can_errors)) 
		
		if self.can_errors > self.max_can_errors:
			rospy.logerr('%s::canError: Errors on CAN bus'%rospy.get_name())
			self.switchToState(State.FAILURE_STATE)
	
	
	def initFailureRecoverTimer(self, timer = 5):
		'''
			Function to start the timer to recover from FAILURE STATE
		'''
		t = threading.Timer(timer, self.failureRecover)
		t.start()
	
		
	def failureRecover(self):		
		'''
			Sets the recovery flag TRUE. Function called after timer
		'''
		self.failure_recover = True
	
	
	def setControlModeServiceCb(self, req):
		'''
			Sets the hand control mode
			@param req: Requested service
			@type req: bhand_controller.srv.SetControlMode
		'''
		if self.state != State.READY_STATE:
			rospy.logerr('%s:setControlModeServiceCb: bhand is not in a valid state to change the mode'%rospy.get_name() )
			return False
			
		if req.mode != CONTROL_MODE_POSITION and req.mode != CONTROL_MODE_VELOCITY:
			rospy.logerr('%s:setControlModeServiceCb: invalid control mode %s'%(rospy.get_name(), req.mode))
			return False
			
		else:
			if self.setControlMode(req.mode):
				rospy.loginfo('%s:setControlModeServiceCb: control mode %s set successfully'%(rospy.get_name(), req.mode))
				return True
				
			else:
				return False
	
	
	def setControlMode(self, mode):		
		'''
			Configures the hand to work under the desired control mode
			@param mode: new mode
			@type mode: string
		'''
		if self.control_mode == mode:
			return True
				
		if mode == CONTROL_MODE_POSITION:
			self.hand.set_mode(HAND_GROUP, 'IDLE')
			
			
		elif mode == CONTROL_MODE_VELOCITY:
			self.hand.set_mode(HAND_GROUP, 'VEL')
			
		self.control_mode = mode
	
	
	def openFingers(self):
		'''
			Opens all the fingers
		'''		
		self.desired_joints_position['F1'] = 0.0
		self.desired_joints_position['F2'] = 0.0
		self.desired_joints_position['F3'] = 0.0
		self.hand.move_to(FINGER1, self.hand.rad_to_enc(self.desired_joints_position['F1'], BASE_TYPE), False)
		self.hand.move_to(FINGER2, self.hand.rad_to_enc(self.desired_joints_position['F2'], BASE_TYPE), False)
		self.hand.move_to(FINGER3, self.hand.rad_to_enc(self.desired_joints_position['F3'], BASE_TYPE), False)
				
	
	def closeFingers(self, value):
		'''
			Closes all the fingers
		'''		
		self.desired_joints_position['F1'] = value
		self.desired_joints_position['F2'] = value
		self.desired_joints_position['F3'] = value
		self.hand.move_to(FINGER1, self.hand.rad_to_enc(self.desired_joints_position['F1'], BASE_TYPE), False)
		self.hand.move_to(FINGER2, self.hand.rad_to_enc(self.desired_joints_position['F2'], BASE_TYPE), False)
		self.hand.move_to(FINGER3, self.hand.rad_to_enc(self.desired_joints_position['F3'], BASE_TYPE), False)
		
		
		
def main():

	rospy.init_node("bhand_node")
	
	
	_name = rospy.get_name()
	
	arg_defaults = {
	  'port':  '/dev/pcan32',
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'tactile_sensors': True,
	  'control_mode': 'POSITION',
	  'joint_ids': [ 'F1', 'F1_TIP', 'F2', 'F2_TIP', 'F3', 'F3_TIP', 'SPREAD_1', 'SPREAD_2'],
	  'joint_names': ['bh_j12_joint', 'bh_j13_joint', 'bh_j22_joint', 'bh_j23_joint', 'bh_j32_joint', 'bh_j31_joint', 'bh_j11_joint', 'bh_j21_joint']
	}
	
	args = {}
	
	for name in arg_defaults:
		try:
			if rospy.search_param(name): 
				args[name] = rospy.get_param('%s/%s'%(_name, name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerror('bhand_node: %s'%e)
			
	#print args
	bhand_node = BHand(args)
	
	rospy.loginfo('bhand_node: starting')

	bhand_node.start()


if __name__ == "__main__":
	main()
