# Copyright (c) 2014, Robotnik Automation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import rospkg
import threading

import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot, QBasicTimer, SIGNAL
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget, QPixmap, QMessageBox, QStandardItemModel,QStandardItem
from rqt_gui_py.plugin import Plugin
import tf_conversions.posemath as pm
import tf

import time
import math

from bhand_controller.msg import State, TactileArray, Service
from bhand_controller.srv import Actions, SetControlMode
from barrett_tactile_msgs.msg import TactileInfo
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rospy.exceptions import ROSException

from tact_maps import tact_to_finger1_map, tact_to_finger2_map, tact_to_finger3_map, tact_to_palm_map

#angles
max_base_spread = 3.14
max_finger_spread = 2.44

MAX_VELOCITY = 0.1 # rad/s
BHAND_VELOCITY_COMMANDS_FREQ = 50 # Frequency of vel publications in ms

class BHandGUI(Plugin):
	
	def __init__(self, context):
		super(BHandGUI, self).__init__(context)
		self.setObjectName('BHandGUI')
		
		self.read_ros_params()
		self._publisher = None

		self._widget = QWidget()
		
		# variable to store the sensor data when receives it
		self._bhand_data = State()
		self._joint_data = JointState()
		self._tact_data = None
			
		rp = rospkg.RosPack()
		
		# Flag to enable sending commands
		self.enable_commands = False

		# Threshold for tactile high
		self.THRESHOLD = 1.0

		self.threshold_values = {}
		self.threshold_values["num_values"] = 0
		self.threshold_values["link1"] = []
		self.threshold_values["link2"] = []
		self.threshold_values["link3"] = []
		self.threshold_values["palm"]  = []
		
		#Variable inits
		# DESIRED POSITION
		self.base_spread = 0.0
		self.finger1_spread = 0.0
		self.finger2_spread = 0.0
		self.finger3_spread = 0.0
		# DESIRED VELOCITY
		self.base_spread_vel = 0.0
		self.finger1_spread_vel = 0.0
		self.finger2_spread_vel = 0.0
		self.finger3_spread_vel = 0.0
		self.max_vel = MAX_VELOCITY
		self.vel_factor = self.max_vel/100.0 # For the slider
		
		self.red_string = "background-color: rgb(255,0,0)"
		self.orange_string = "background-color: rgb(255,128,0)"
		self.yellow_string = "background-color: rgb(255,255,0)"
		self.green_string = "background-color: rgb(128,255,0)"
		self.black_string = "background-color: rgb(0,0,0)"
		
		self.palm_factor = max_base_spread/99
		self.finger_factor = max_finger_spread/99
		
		self.state_string = " "

		self.listener = tf.TransformListener()
		

		# UI
		ui_file = os.path.join(rp.get_path('rqt_bhand'), 'resource', 'BHandGUI.ui')
		loadUi(ui_file, self._widget)
		self._widget.setObjectName('BHandGUI')
		
		pixmap_red_file = os.path.join(rp.get_path('rqt_bhand'), 'resource', 'red.png')
		pixmap_green_file = os.path.join(rp.get_path('rqt_bhand'), 'resource', 'green.png')
		self._pixmap_red = QPixmap(pixmap_red_file)
		self._pixmap_green = QPixmap(pixmap_green_file)
		self._widget.qlabel_state_connection.setPixmap(self._pixmap_red) # Shows agvs_controller comm state
		self._widget.qlabel_jointstate_connection.setPixmap(self._pixmap_red) # Shows agvs_controller comm state
		
				
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))       
		
		# Adds this widget to the context
		context.add_widget(self._widget)

		# Try to connect to the topic
		self._topic = '/%s/state'%self.bhand_node_name 
		self._joint_states_topic = '/joint_states'
		self._tact_topic = '/%s/tact_array'%self.bhand_node_name 
		self._command_topic = '/%s/command'%self.bhand_node_name 
		self._tactile_info_topic = '/%s/tactile_info'%self.bhand_node_name
		self._set_mode_service_name = '/%s/set_control_mode'%self.bhand_node_name
		self._actions_service_name = '/%s/actions'%self.bhand_node_name

		# Saves the desired value
		self.desired_ref = JointState()
		
		#self.joint_names = ['j11_joint', 'j32_joint', 'j12_joint', 'j22_joint', 'j23_joint', 'j13_joint', 'j33_joint']
		#self.joint_names =  ['bh_j12_joint', 'bh_j13_joint', 'bh_j22_joint', 'bh_j23_joint', 'bh_j32_joint', 'bh_j31_joint', 'bh_j11_joint', 'bh_j21_joint']
		
		#self.joint_ids = ['F1', 'F1_TIP', 'F2', 'F2_TIP', 'F3', 'F3_TIP', 'SPREAD_1', 'SPREAD_2']
		# Intermediate structure to save the read positions, vels and efforts
		# Ej.: {'j1': [ 0, 0, 0 ]}
		self.joint_state_pointer = {} 
		for i in range(len(self.joint_ids)):
			self.joint_state_pointer[self.joint_ids[i]] = {'joint': self.joint_names[i], 'values': [0, 0, 0]}
			# Initializes joints data for receiving the read values
			self._joint_data.name.append(self.joint_names[i])
			self._joint_data.position.append(0.0)
			self._joint_data.velocity.append(0.0)
			self._joint_data.effort.append(0.0)
		
		# SUBSCRIPTIONS
		try:
			self._subscriber = rospy.Subscriber(self._topic, State, self._receive_state_data)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)
			
		try:
			self._joint_subscriber = rospy.Subscriber(self._joint_states_topic, JointState, self._receive_joints_data)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)
			
		try:
			self._tact_subscriber = rospy.Subscriber(self._tact_topic, TactileArray, self._receive_tact_data)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)
		
		# PUBLICATIONS
		try:
			self._publisher_command = rospy.Publisher(self._command_topic, JointState, queue_size=10)
		except ROSException, e:
			rospy.logerr('BHandGUI: Error creating publisher for topic %s (%s)'%(self._command_topic, e))

		try:
			self._publisher_tactile_info = rospy.Publisher(self._tactile_info_topic, TactileInfo, queue_size=10)
		except ROSException, e:
			rospy.logerr('BHandGUI: Error creating publisher for topic %s (%s)'%(self._command_topic, e))
		
		# SERVICES
		try:
			self._service_bhand_actions = rospy.ServiceProxy(self._actions_service_name, Actions)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting service (%s)'%e)
			
		try:
			self._service_set_mode = rospy.ServiceProxy(self._set_mode_service_name, SetControlMode)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting service (%s)'%e)
		
		
			
		self.fixed_fingers = 0
		
		# HANDLERS
		# Adds handlers to 'press button' event
		self._widget.pushButtonStart.clicked.connect(self.start_button_pressed)
		self._widget.pushButton_stop.clicked.connect(self.stop)
		self._widget.pushButton_close.clicked.connect(self.close_button_pressed)
		self._widget.pushButton_open.clicked.connect(self.open_button_pressed)
		self._widget.pushButton_mode1.clicked.connect(self.mode1_button_pressed)
		self._widget.pushButton_mode2.clicked.connect(self.mode2_button_pressed)
		self._widget.horizontalSlider.valueChanged.connect(self.slider_1_changed)
		self._widget.horizontalSlider_2.valueChanged.connect(self.slider_2_changed)
		self._widget.horizontalSlider_3.valueChanged.connect(self.slider_3_changed)
		self._widget.horizontalSlider_4.valueChanged.connect(self.slider_4_changed)
		self._widget.horizontalSlider_v_spread.valueChanged.connect(self.slider_v_spread_changed)
		self._widget.horizontalSlider_v_f1.valueChanged.connect(self.slider_v_f1_changed)
		self._widget.horizontalSlider_v_f2.valueChanged.connect(self.slider_v_f2_changed)
		self._widget.horizontalSlider_v_f3.valueChanged.connect(self.slider_v_f3_changed)
		
		self.connect(self._widget.checkBox_enable_control_vel, SIGNAL("stateChanged(int)"),  self.enable_command_vel_changed)
		self.connect(self._widget.checkBox_enable_control_pos, SIGNAL("stateChanged(int)"),  self.enable_command_pos_changed)
		self._widget.checkBox_enable_control_vel.setChecked(False)
		
		#self._widget.radioButtonPosition.clicked.connect(self.radio_button_position_clicked)
		#self._widget.radioButtonVelocity.clicked.connect(self.radio_button_velocity_clicked)
		self.connect(self._widget.radioButtonPosition, SIGNAL("clicked(bool)"),  self.radio_button_position_clicked)
		self.connect(self._widget.radioButtonVelocity, SIGNAL("clicked(bool)"),  self.radio_button_velocity_clicked)
		
		
		self._widget.checkBox.stateChanged.connect(self.fingers_check_box_pressed)
		
		
		self._init_timers()
	
	
	def read_ros_params(self):
		'''
			Read ROS params from server
		'''
		_name = rospy.get_name()
		
		self.bhand_node_name = rospy.get_param('%s/bhand_node_name'%_name, 'bhand_node')
		# Reads the configuration of the joints ids
		self.joint_ids  = rospy.get_param('%s/joint_ids'%(self.bhand_node_name), ['F1', 'F1_TIP', 'F2', 'F2_TIP', 'F3', 'F3_TIP', 'SPREAD_1', 'SPREAD_2'])
		self.joint_names  = rospy.get_param('%s/joint_names'%(self.bhand_node_name), ['bh_j12_joint', 'bh_j13_joint', 'bh_j22_joint', 'bh_j23_joint', 'bh_j32_joint', 'bh_j31_joint', 'bh_j11_joint', 'bh_j21_joint'])
		
		rospy.loginfo('%s::read_ros_params: bhand_node_name = %s'%(_name, self.bhand_node_name))
		rospy.loginfo('%s::read_ros_params: joint_ids = %s'%(_name, self.joint_ids))
		rospy.loginfo('%s::read_ros_params: joint_names = %s'%(_name, self.joint_names))
		
		
		
	# inits the timers used to control the connection, ...
	def _init_timers(self):
		self._topic_connected = False
		self._topic_joint_states_connected = False
		
		self._topic_timer = time.time()
		self._topic_joint_states_timer = time.time()
		
		
		
		
		self._topic_timeout_connection = 5 # seconds
		# Creates a basic timer and starts it
		self._timer = QBasicTimer()
		self._timer.start(1, self)
		# Creates a timer to send command refs periodically
		self._timer_commands = QTimer()
		self.connect(self._timer_commands, SIGNAL("timeout()"),  self.timeout_command_timer)
		
	def start_button_pressed(self):
		'''
			Handles the press button event to call initialization service
		'''
		self.send_bhand_action(Service.INIT_HAND)
		'''
			rospy.wait_for_service(self._actions_service_name)
			try:
				action = rospy.ServiceProxy(self._actions_service_name, Actions)
				resp1 = action(Service.INIT_HAND)
				return resp1
			except rospy.ServiceException, e:
				rospy.logerr("Service call failed: %s"%e)
		'''
	
	def close_button_pressed(self):
		'''
			Handles the press button event to call close hand service
		'''
		#self._widget.checkBox_enable_control_pos.setChecked(False)
		self._widget.radioButtonPosition.setChecked(True)
		self.send_bhand_action(Service.CLOSE_GRASP)
	
	
	def open_button_pressed(self):
		'''
			Handles the press button event to call open hand service
		'''
		self.send_bhand_action(Service.OPEN_GRASP)
	
	
	def mode1_button_pressed(self):
		'''
			Handles the press button event to call set mode 1  service
		'''
		self.send_bhand_action(Service.SET_GRASP_1)
	
	
	def mode2_button_pressed(self):
		'''
			Handles the press button event to call set mode 2 service
		'''
		self.send_bhand_action(Service.SET_GRASP_2)
		
		
	
	def slider_1_changed(self):
		self.base_spread = self._widget.horizontalSlider.value() * self.palm_factor
		if self.enable_commands and self._bhand_data.control_mode == 'POSITION':
			self.send_position_command()
		
		
		
	def slider_2_changed(self):
		self.finger3_spread = self._widget.horizontalSlider_2.value() * self.finger_factor
		if self.fixed_fingers == 1:
			self.finger1_spread = self.finger2_spread = self.finger3_spread
			self._widget.horizontalSlider_3.setSliderPosition(self._widget.horizontalSlider_2.value())
			self._widget.horizontalSlider_4.setSliderPosition(self._widget.horizontalSlider_2.value())
		if self.enable_commands and self._bhand_data.control_mode == 'POSITION':
			self.send_position_command()
		
	def slider_3_changed(self):
		self.finger1_spread = self._widget.horizontalSlider_3.value() * self.finger_factor
		if self.fixed_fingers == 1:
			self.finger3_spread = self.finger2_spread = self.finger1_spread
			self._widget.horizontalSlider_2.setSliderPosition(self._widget.horizontalSlider_3.value())
			self._widget.horizontalSlider_4.setSliderPosition(self._widget.horizontalSlider_3.value())
		if self.enable_commands and self._bhand_data.control_mode == 'POSITION':
			self.send_position_command()
		
	def slider_4_changed(self):
		self.finger2_spread = self._widget.horizontalSlider_4.value() * self.finger_factor
		if self.fixed_fingers == 1:
			self.finger1_spread = self.finger2_spread = self.finger2_spread
			self._widget.horizontalSlider_2.setSliderPosition(self._widget.horizontalSlider_4.value())
			self._widget.horizontalSlider_3.setSliderPosition(self._widget.horizontalSlider_4.value())
		if self.enable_commands and self._bhand_data.control_mode == 'POSITION':
			self.send_position_command()
	
	def slider_v_spread_changed(self):
		'''
			Handler for slider v_spread
		'''
		self.base_spread_vel = self._widget.horizontalSlider_v_spread.value() * self.vel_factor
		
	
	def slider_v_f1_changed(self):
		'''
			Handler for slider v_f1
		'''
		self.finger1_spread_vel = self._widget.horizontalSlider_v_f1.value() * self.vel_factor
		if self._widget.checkBox_moveall_velocity.isChecked():
			self._widget.horizontalSlider_v_f2.setSliderPosition(self._widget.horizontalSlider_v_f1.value())
			self._widget.horizontalSlider_v_f3.setSliderPosition(self._widget.horizontalSlider_v_f1.value())
	
	
	def slider_v_f2_changed(self):
		'''
			Handler for slider v_f2
		'''
		self.finger2_spread_vel = self._widget.horizontalSlider_v_f2.value() * self.vel_factor
		if self._widget.checkBox_moveall_velocity.isChecked():
			self._widget.horizontalSlider_v_f1.setSliderPosition(self._widget.horizontalSlider_v_f2.value())
			self._widget.horizontalSlider_v_f3.setSliderPosition(self._widget.horizontalSlider_v_f2.value())
		
	
	def slider_v_f3_changed(self):
		'''
			Handler for slider v_f3
		'''
		self.finger3_spread_vel = self._widget.horizontalSlider_v_f3.value() * self.vel_factor
		if self._widget.checkBox_moveall_velocity.isChecked():
			self._widget.horizontalSlider_v_f2.setSliderPosition(self._widget.horizontalSlider_v_f3.value())
			self._widget.horizontalSlider_v_f1.setSliderPosition(self._widget.horizontalSlider_v_f3.value())
	

		
	def send_position_command(self):
		'''
			Sends a command in position mode
		'''
		
		self.desired_ref.name = [self.joint_state_pointer['SPREAD_1']['joint'], self.joint_state_pointer['F3']['joint'], self.joint_state_pointer['F1']['joint'], self.joint_state_pointer['F2']['joint']]
		self.desired_ref.position = [self.base_spread, self.finger3_spread, self.finger1_spread, self.finger2_spread]
		self.desired_ref.velocity = [0.1, 0.1, 0.1, 0.1]
		self.desired_ref.effort = [0.0, 0.0, 0.0, 0.0]
		self._publisher_command.publish(self.desired_ref)
		
		
	def fingers_check_box_pressed(self, state):
		'''
			Handler call when clicking checkbox to control all fingers
		'''
		if state == 0:
			self.fixed_fingers = 0
		elif state == 2:
			self.fixed_fingers = 1
		
			
		
	@Slot(str)
	# Handles the change of topic's name
	def _on_topic_changed(self):
		# reads the topic's name from the 'QLineEdit' 'topic_line_edit' (in the .ui file)
		#self._topic = self._widget.topic_line_edit.text()
		self._subscriber.unregister()
		try:
			self._subscriber = rospy.Subscriber(self._topic, State, self._receive_state_data)
		except ValueError, e:
			rospy.logerr('BHandGUI: Error connecting topic (%s)'%e)
		
	# Handles the messages from the agvs controller
	def _receive_state_data(self, msg):
		self._bhand_data = msg
		self._topic_timer = time.time()
		
		if not self._topic_connected:
			rospy.loginfo('BHandGUI: connection established with %s'%self._topic)
			self._topic_connected = True
	
	
	def _receive_joints_data(self, msg):
		'''
			Handler for Joint States
		'''
		self._joints_data = msg
		
		self._topic_joint_states_timer = time.time()
		
		for i in range(len(msg.name)):
			for j in self.joint_state_pointer:
				if self.joint_state_pointer[j]['joint'] == msg.name[i]:
					self.joint_state_pointer[j]['values'] = [msg.position[i], msg.velocity[i], msg.effort[i]]
			
		
		if not self._topic_joint_states_connected:
			rospy.loginfo('Bhand: connection established with %s'%self._joint_states_topic)
			self._topic_joint_states_connected = True
		
		
				
		
	def _receive_tact_data(self, msg):
		
		if self._tact_data is None:
			self._tact_data = TactileArray()
			
		self._tact_data = msg
	
			
			
				
	# Handles the messages from the dspic controller
	def _receive_dspic_data(self, msg):
		self._dspic_data = msg
		self._topic_dspic_timer = time.time()
		
		if not self._topic_dspic_connected:
			rospy.loginfo('BHandGUI: connection established with %s'%self._topic_dspic)
			self._topic_dspic_connected = True
	
			
	# Handles the press event of the button dspic reset odom
	def press_reset_dspic_odom(self):   
		
		ret = QMessageBox.question(self._widget, "Dspic Reset Odometry", 'Are you sure you want to reset the odometry?', QMessageBox.Ok, QMessageBox.Cancel)
		
		if ret == QMessageBox.Ok:
			
			# Call the service recalibrate dspic
			try:
				ret = self._service_dspic_reset_odom(0.0, 0.0, 0.0, 0.0)                
			except ValueError, e:
				rospy.logerr('BHandGUI::press_reset_dspic_odom: (%s)'%e)
			except rospy.ServiceException, e:
				rospy.logerr('BHandGUI::press_reset_dspic_odom: (%s)'%e)
				QMessageBox.warning(self._widget, "Warning", "service is not available: press_reset_dspic_odom")
	
	
	def enable_command_vel_changed(self, value):
		'''
			Handles command enabled event from checkbox
		'''
		 
		if value != 0:
			self._widget.checkBox_enable_control_pos.setChecked(True)
			self.enable_commands = True
			#if self._bhand_data.control_mode == 'VELOCITY':
			#   self._timer_commands.start(BHAND_VELOCITY_COMMANDS_FREQ)
				
				 
		else:
			self._widget.checkBox_enable_control_pos.setChecked(False)
			self.enable_commands = False
			#if self._bhand_data.control_mode == 'VELOCITY':
			#   self._timer_commands.stop()
			
			
	
	def enable_command_pos_changed(self, value):
		'''
			Handles command enabled event from checkbox
		'''
		
		if value != 0:
			self._widget.checkBox_enable_control_vel.setChecked(True)
			self.enable_commands = True
		else:
			self._widget.checkBox_enable_control_vel.setChecked(False)
			self.enable_commands = False
	
	
	def radio_button_position_clicked(self, value):
		'''
			Handles the click of this radio button
		'''
		self.set_control_mode('POSITION')
		self._timer_commands.stop()
		self.stop()
		
	
	def radio_button_velocity_clicked(self, value):
		'''
			Handles the click of this radio button
		'''
		self.set_control_mode('VELOCITY')
		self._timer_commands.start(BHAND_VELOCITY_COMMANDS_FREQ)
	

	def set_control_mode(self, mode):   
		'''
			Calls the service to set the control mode of the hand
			@param mode: Desired Bhand mode of operation ('POSITION', 'VELOCITY')
			@type mode: string
		'''         
		try:
			ret = self._service_set_mode(mode)              
		except ValueError, e:
			rospy.logerr('BHandGUI::set_control_mode: (%s)'%e)
		except rospy.ServiceException, e:
			rospy.logerr('BHandGUI::set_control_mode: (%s)'%e)
			QMessageBox.warning(self._widget, "Warning", "Service is not available: set_control_mode")
	
	
	def send_bhand_action(self, action):    
		'''
			Calls the service to set the control mode of the hand
			@param action: Action number (defined in msgs/Service.msg)
			@type action: int
		'''         
		try:
			ret = self._service_bhand_actions(action)               
		except ValueError, e:
			rospy.logerr('BHandGUI::send_bhand_action: (%s)'%e)
		except rospy.ServiceException, e:
			rospy.logerr('BHandGUI::send_bhand_action: (%s)'%e)
			QMessageBox.warning(self._widget, "Warning", "Service is not available: send_bhand_action")
						
	
	def send_velocity_command(self):
		'''
			Sends a command to the bhand
		'''
		self.desired_ref.name = [self.joint_state_pointer['SPREAD_1']['joint'], self.joint_state_pointer['F3']['joint'], self.joint_state_pointer['F1']['joint'], self.joint_state_pointer['F2']['joint']]
		self.desired_ref.position = [0, 0, 0, 0]
		self.desired_ref.velocity = [self.base_spread_vel, self.finger3_spread_vel, self.finger1_spread_vel, self.finger2_spread_vel]
		self.desired_ref.effort = [0.0, 0.0, 0.0, 0.0]
		self._publisher_command.publish(self.desired_ref)
		

	def timeout_command_timer(self):
		'''
			Handles every timeout triggered by the Qtimer for sending commands  
		'''
		if self.enable_commands and self._bhand_data.control_mode == 'VELOCITY':
			self.send_velocity_command()
	
	
	def stop(self):
		'''
			Stops the movement of the joints
			It sets the velocity to 0
		'''
		self._widget.horizontalSlider_v_spread.setValue(0)
		self._widget.horizontalSlider_v_f1.setValue(0)
		self._widget.horizontalSlider_v_f2.setValue(0)
		self._widget.horizontalSlider_v_f3.setValue(0)
	
	
	def shutdown_plugin(self):
		'''
			Shutdowns connections
		'''
		self._timer_commands.stop()
		self._timer.stop()
		self._subscriber.unregister()
		self._joint_subscriber.unregister()
		self._tact_subscriber.unregister()
		self._publisher_command.unregister()
		self._service_bhand_actions.close()
		self._service_set_mode.close()


	def getPosition(self, location_name):
		# now = rospy.Time.now()
		# self.listener.waitForTransform(location_name, "world", now, rospy.Duration(4.0))
		# try:
		# 	(trans,rot) = self.listener.lookupTransform(location_name, 'world', now)
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	rospy.logerr("Could not lookup transform {} to world".format(location_name))
		# 	return None

		# pose_stamped = PoseStamped()
		# pose_stamped.pose.position.x = trans[0]
		# pose_stamped.pose.position.y = trans[1]
		# pose_stamped.pose.position.z = trans[2]

		# quat = pm.transformations.quaternion_from_euler(rot[0], rot[1], rot[2])
		# pose_stamped.pose.orientation.x = quat[0]
		# pose_stamped.pose.orientation.y = quat[1]
		# pose_stamped.pose.orientation.z = quat[2]
		# pose_stamped.pose.orientation.w = quat[3]

		pose_stamped = PoseStamped()
		pose_stamped.pose.position.x = 0
		pose_stamped.pose.position.y = 0
		pose_stamped.pose.position.z = 0

		pose_stamped.pose.orientation.x = 0
		pose_stamped.pose.orientation.y = 0
		pose_stamped.pose.orientation.z = 0
		pose_stamped.pose.orientation.w = 1

		pose_stamped.header.frame_id = location_name

		return pose_stamped


	def publishLocations(self, location_list):
		msg = TactileInfo()
		msg.tactile_info = location_list
		self._publisher_tactile_info.publish(msg)

	def updateLCD(self, tact_array_index, lcd_number, lcd_tag, tact_to_sensor_map, link_name, sensor_name, finger_array):

		lcd_string = "lcdNumber{}{}".format(lcd_number, lcd_tag)
		offset = self.threshold_values[link_name][lcd_number]
		value = max(0.0, round(finger_array[tact_array_index] - offset, 1))

		getattr(self._widget,lcd_string).display(value)
		if 0.0 <= value and value < 4.0:
			color_string = self.green_string
		elif 4.0 <= value and value < 8.0:
			color_string = self.yellow_string
		elif 8.0 <= value and value < 12.0:
			color_string = self.orange_string
		elif 12.0 <= value and value <= 16.0:
			color_string = self.red_string
		else:
			color_string = self.black_string
		getattr(self._widget,lcd_string).setStyleSheet(color_string)

		if self.THRESHOLD <= value:
			return self.getPosition(sensor_name.format(tact_to_sensor_map[tact_array_index]))

		return None

	
	# Method executed periodically
	# Updates the graphical qt components
	def timerEvent(self, e):
		
		#state = self._bhand_data.state
		
		#Initialized?
		if self._bhand_data.hand_initialized:
			self._widget.checkBox_hand_initialized.setChecked(1)
		else:
			self._widget.checkBox_hand_initialized.setChecked(0)
		
		#State      
		if self._bhand_data.state == 100:
			self.state_string = "INIT_STATE"
		elif self._bhand_data.state == 200:
			self.state_string = "STANDBY_STATE"
		elif self._bhand_data.state == 300:
			self.state_string = "READY_STATE"
		elif self._bhand_data.state == 400:
			self.state_string = "EMERGENCY_STATE"
		elif self._bhand_data.state == 500:
			self.state_string = "FAILURE_STATE"
		elif self._bhand_data.state == 600:
			self.state_string = "SHUTDOWN_STATE"
			
		if self.state_string != " ":
			self._widget.lineEdit_state.setText(self.state_string)
		
		self._widget.lineEdit_mode.setText(self._bhand_data.control_mode)
			
		#Frequencies
		self._widget.lineEdit_motor_pos_hz.setText(str(self._bhand_data.desired_freq))
		self._widget.lineEdit_motor_pos_hz_2.setText(str(round(self._bhand_data.real_freq,1)))
		
		#Temperatures
		self._widget.lineEdit.setText(str(self._bhand_data.temp_spread[0]))
		self._widget.lineEdit_3.setText(str(self._bhand_data.temp_spread[1]))
		self._widget.lineEdit_2.setText(str(self._bhand_data.temp_f1[0]))
		self._widget.lineEdit_4.setText(str(self._bhand_data.temp_f1[1]))
		self._widget.lineEdit_5.setText(str(self._bhand_data.temp_f2[0]))
		self._widget.lineEdit_6.setText(str(self._bhand_data.temp_f2[1]))
		self._widget.lineEdit_7.setText(str(self._bhand_data.temp_f3[0]))
		self._widget.lineEdit_8.setText(str(self._bhand_data.temp_f3[1]))
		
		# Desired Control Positions
		self._widget.lineEdit_spread_des_pos.setText(str(round(self.base_spread,3)))
		self._widget.lineEdit_f1_des_pos.setText(str(round(self.finger1_spread,3)))
		self._widget.lineEdit_f2_des_pos.setText(str(round(self.finger2_spread,3)))
		self._widget.lineEdit_f3_des_pos.setText(str(round(self.finger3_spread,3)))
		# Read Control Positions
		self._widget.lineEdit_spread_read_pos.setText(str(round(self.joint_state_pointer['SPREAD_1']['values'][0],3)))
		self._widget.lineEdit_f1_read_pos.setText(str(round(self.joint_state_pointer['F1']['values'][0],3)))
		self._widget.lineEdit_f2_read_pos.setText(str(round(self.joint_state_pointer['F2']['values'][0],3)))
		self._widget.lineEdit_f3_read_pos.setText(str(round(self.joint_state_pointer['F3']['values'][0],3)))
		self._widget.lineEdit_spread_read_pos_2.setText(str(round(self.joint_state_pointer['SPREAD_1']['values'][0],3)))
		self._widget.lineEdit_f1_read_pos_2.setText(str(round(self.joint_state_pointer['F1']['values'][0],3)))
		self._widget.lineEdit_f2_read_pos_2.setText(str(round(self.joint_state_pointer['F2']['values'][0],3)))
		self._widget.lineEdit_f3_read_pos_2.setText(str(round(self.joint_state_pointer['F3']['values'][0],3)))
		# Desired velocities
		self._widget.lineEdit_spread_des_vel.setText(str(round(self.base_spread_vel,3)))
		self._widget.lineEdit_f1_des_vel.setText(str(round(self.finger1_spread_vel,3)))
		self._widget.lineEdit_f2_des_vel.setText(str(round(self.finger2_spread_vel,3)))
		self._widget.lineEdit_f3_des_vel.setText(str(round(self.finger3_spread_vel,3)))
		
		#Effort
		if self._bhand_data.state == 300:
			self._widget.lineEdit_fingertip1.setText(str(round(self.joint_state_pointer['F1_TIP']['values'][2],4)))
			self._widget.lineEdit_fingertip2.setText(str(round(self.joint_state_pointer['F2_TIP']['values'][2],4)))
			self._widget.lineEdit_fingertip3.setText(str(round(self.joint_state_pointer['F3_TIP']['values'][2],4)))
		
		#Tactile sensors
		
		if self._tact_data is not None and self._bhand_data.state == 300:

			if self.threshold_values["num_values"] == 0:
				for i in range(0,8):
					for j in range(0,3):
						index = (7-i)*3 + j
						self.threshold_values["link1"].append(self._tact_data.finger1[index])
						self.threshold_values["link2"].append(self._tact_data.finger2[index])
						self.threshold_values["link3"].append(self._tact_data.finger3[index])

				for i in range(0, 24):
					self.threshold_values["palm"].append(self._tact_data.palm[i])

			self.threshold_values["num_values"] += 1
	
			activated_positions = []

			# Loop over finger positions
			for i in range(0,8):
				for j in range(0,3):
					# Finger 1
					val = self.updateLCD((7-i)*3 + j, i*3 + j, "", tact_to_finger1_map, "link1", "bh_link1_sensor{}_link", self._tact_data.finger1)
					if val is not None:
						activated_positions.append(val)

					# Finger 2
					val = self.updateLCD((7-i)*3 + j, i*3 + j, "_3", tact_to_finger2_map, "link2", "bh_link2_sensor{}_link", self._tact_data.finger2)
					if val is not None:
						activated_positions.append(val)

					# Finger 3
					val = self.updateLCD((7-i)*3 + j, i*3 + j, "_4", tact_to_finger3_map, "link3", "bh_link3_sensor{}_link", self._tact_data.finger3)
					if val is not None:
						activated_positions.append(val)
				
			#Palm
			for i in range(0,24):
				val = self.updateLCD(i, i, "_6", tact_to_palm_map, "palm", "bh_palm_sensor_{}_link", self._tact_data.palm)
				if val is not None:
					activated_positions.append(val)

			if len(activated_positions) > 0:
				self.publishLocations(activated_positions)
		
		# Checks the ROS connection
		t = time.time()
		if self._topic_connected and (t - self._topic_timer >= self._topic_timeout_connection):
			self._topic_connected = False
			rospy.logerr('BHandGUI: error in communication with %s'%self._topic)
			self._widget.qlabel_state_connection.setPixmap(self._pixmap_red)
			
		if self._topic_joint_states_connected and (t - self._topic_joint_states_timer >= self._topic_timeout_connection):
			self._topic_joint_states_connected = False
			rospy.logerr('BHandGUI: error in communication with %s'%self._joint_states_topic)
			self._widget.qlabel_jointstate_connection.setPixmap(self._pixmap_red)
		
		if self._topic_connected:
			self._widget.qlabel_state_connection.setPixmap(self._pixmap_green)
			
		if self._topic_joint_states_connected:
			self._widget.qlabel_jointstate_connection.setPixmap(self._pixmap_green)
