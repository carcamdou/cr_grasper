#  pyHand_api.py
#  
#  ~~~~~~~~~~~~
#  
#
#  pyHand_api based on pyHand_api.py from Barrett Technology Hand Communications
#  
#  ~~~~~~~~~~~~
#  
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


from pcan_python.pcan_library import *


from puck_properties_consts import*
from ctypes import *
import time

BASE_TYPE = 0
TIP_TYPE = 1
SPREAD_TYPE = 2
BASE_LIMIT = 140.0
TIP_LIMIT = 48.0
SPREAD_LIMIT = 180.0

HAND_GROUP = 0x405

# CAN MSG IDs
F1_POSITION = 0x563
F2_POSITION = 0x583
F3_POSITION = 0x5A3
SPREAD_POSITION = 0x5C3
F1_STRAIN	= 0x566
F2_STRAIN	= 0x586
F3_STRAIN	= 0x5A6
STRAIN_ID   = 0x99
F1_MOTOR_TEMP	= 0x566
F2_MOTOR_TEMP	= 0x586
F3_MOTOR_TEMP	= 0x5A6
SPREAD_MOTOR_TEMP	= 0x5C6
MOTOR_TEMP_ID   = 0x89
F1_MOTOR_THERM	= 0x566
F2_MOTOR_THERM	= 0x586
F3_MOTOR_THERM	= 0x5A6
SPREAD_MOTOR_THERM	= 0x5C6
MOTOR_THERM_ID   = 0x94
F1_TACT = 0x569
F2_TACT = 0x589
F3_TACT = 0x5A9
PALM_TACT = 0x5C9
TACT_ID = 0x40


#==========================CAN_STUFF=======================================

# Class pyHand based on library pyHand_api.py
class pyHand:
	
	def __init__(self, port = '/dev/pcan32'):
		'''
		Creates a PCANBasic object

			@param port: CAN port used to communicate with the hand
			@type connection: string
		'''
		
		self.PCAN = PCANBasic(port)
		
		self.motor_positions = {FINGER1: {'encoder': [0, 0], 'position': [0.0, 0.0]}, FINGER2: {'encoder': [0, 0], 'position': [0.0, 0.0]}, FINGER3: {'encoder': [0, 0], 'position': [0.0, 0.0]},
		SPREAD: {'encoder': [0, 0], 'position': [0.0, 0.0]}}
		self.strain = {FINGER1: 0, FINGER2: 0, FINGER3: 0}
		self.temp = {FINGER1:{ 'temp': 0.0, 'therm': 0.0}, FINGER2: { 'temp': 0.0, 'therm': 0.0}, FINGER3: { 'temp': 0.0, 'therm': 0.0},
		 SPREAD: { 'temp': 0.0, 'therm': 0.0}}
		self.tactile_sensor = {FINGER1: {'values': range(0,24), 'data': range(0,5)}, FINGER2: {'values': range(0,24), 'data': range(0,5)}, 
		FINGER3: {'values': range(0,24), 'data': range(0,5)}, SPREAD: {'values': range(0,24), 'data': range(0,5)}}
		
	def check_error(self, connection,result,location_of_error):
		'''
		Checks error on the CAN Bus.

			@param connection: The connection on which the CAN is talking. For example, PCANBasic()
			@type connection: PCANBasic
			@param result: The number corresponding to the error returned.
			@type resulr: int
			@param location_of_error: A description of where the error occurred.
			@type location_of_error: str
		'''
		if result == PCAN_ERROR_OK:
			pass
		else:
			raise Exception("Error Number: " + hex(result) + " while attempting to " + str(location_of_error) + "\n" + connection.GetErrorText(result)[1])

	def enum(self):
		'''
		Finds and returns all of the pucks that are attached to the bus.

			@rtype: Array[]
			@return: An array containing the pucks attached to the bus.
		'''
		pucks = []
		for i in range(32):
			try:
				self.get_property(i,1)
				pucks.append(i)
			except:
				pass
		return pucks

	def can_reset(self):
		'''
		Resets the CAN connection.
		Note that this may cause a loss of data, but may also clear unwanted data. Utilize as needed.
		'''
		reset_result=self.PCAN.Reset(PCAN_USBBUS1)
		try:
			self.check_error(self.PCAN,reset_result,"reset")
		except:
			return False
		time.sleep(0.025)
		return True

	def can_status(self):
		'''
		Returns the status of the CAN connection as specified in PCANBasic's GetStatus method.
		'''
		status_result=self.PCAN.GetStatus(PCAN_USBBUS1)
		time.sleep(0.025)
		return status_result

	def can_init(self):
		'''
		Initializes the CAN connection. Note that this does not initialize the hand itself.
		'''
		# initialize self.PCAN bus
		init_result=self.PCAN.Initialize(PCAN_USBBUS1, PCAN_BAUD_1M)
		self.check_error(self.PCAN,init_result,"initialize")
		time.sleep(0.025)

	def can_uninit(self):
		'''
		Uninitializes the CAN connection. Rarely used.
		'''
		uninit_result=self.PCAN.Uninitialize(PCAN_USBBUS1)
		self.check_error(self.PCAN,uninit_result,"uninitialize")
		
	#============================INIT_STUFF==================================
		
	def initialize(self):
		'''
		Wakes up pucks and initializes the CAN.
		This function must be implemented at the beginning of a program for this library to properly work.

			@rtype: ROLE
			@return: First finger's ROLE property if initialization is successful. Otherwise it returns False.
		'''
		try:
			self.can_init()
			#Reset Can bus
			self.can_reset()
			time.sleep(0.025)
			# wake up pucks by setting the STAT(5) property to READY(2)
			self.set_property(0x400, 5, 2)
			time.sleep(1)
			self.can_reset()
			
			return True
		except:
			return False

	def init_hand(self):
		'''
		Initialize all pucks in the hand.

			@rtype: Boolean
			@return: Succesfully Initialized Hand?
		'''
		try:
			#write 13 to the command property (CMD-29) of each of the pucks.
			self.init_finger(FINGER1)
			self.init_finger(FINGER2)
			self.init_finger(FINGER3)
			time.sleep(3)
			self.init_finger(SPREAD)
			time.sleep(2)
			
			self.can_reset()
			self.get_property(FINGER1, ROLE)
			return True
		except:
			self.can_reset()
			return False

	def init_finger(self, msgID):
		'''
		Sends a command to the puck to wake up the motor attatched to the puck.

			@param msgID: The id of the finger to initialize.
			@type msgID: int
		'''
		self.set_property(msgID, CMD, CMD_HI)

	#============================SET_AND_GET_STUFF=========================

	def read_msg(self):
		'''
		Read a general message from PCAN_USBBUS1
		Typically, msg[1] (where msg is the thing returned), contains the pertinent information.

			@rtype: (TPCANStatus, TPCANMsg, TPCANTimestamp)
			@return: A tuple containing the status, message, and timestamp.
		'''
		return self.PCAN.Read(PCAN_USBBUS1)

	#def write_msg(self, msgID, data, delay=.015):
	
	def write_msg(self, msgID, data, delay=.002):
		'''
		Send a general message to PCAN_USBBUS1. This can be a get, set, or even garbage.

			@param msgID: The puck or group to which the message will be sent.
			@type msgID: int
			@param data: The array containing the data for the TPCANMsg.
			@type data: Array[]
			@param delay: The time delay to wait for the message to be written.
			@type delay: float
			@rtype: TPCANStatus
			@return: Status of the self.PCAN bus.
		'''
		msg = TPCANMsg()
		msg.ID = msgID
		msg.LEN = len(data)
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD
		for j in range(0, len(data)):
			msg.DATA[j] = data[j]
		stat = self.PCAN.Write(PCAN_USBBUS1, msg)
		self.check_error(self.PCAN,stat,"write")
		time.sleep(delay)
		return stat
	
	
	def send_msg(self, msgID, data):
		'''
			Send a general message to PCAN_USBBUS1. This can be a get, set, or even garbage.
			It does not apply any sleep
			
			@param msgID: The puck or group to which the message will be sent.
			@type msgID: int
			@param data: The array containing the data for the TPCANMsg.
			@type data: Array[]
			
			@return the return status after writing in the bus
		'''
		msg = TPCANMsg()
		msg.ID = msgID
		msg.LEN = len(data)
		msg.MSGTYPE = PCAN_MESSAGE_STANDARD
		for j in range(0, len(data)):
			msg.DATA[j] = data[j]
		stat = self.PCAN.Write(PCAN_USBBUS1, msg)
		self.check_error(self.PCAN,stat,"write")
		
		return stat

	def set_property(self, msgID, propID, value):
		'''
		Set property to a given value.

			@param msgID: The puck or group whose property will be set.
			@type msgID: int
			@param propID: The number corresponding to the property to be set.
			@type propID: int
			@param value: The value to which the property will be set.
			@type value: int
		'''
		is32bits = [48, 50, 52, 54, 56, 58, 66, 68, 74, 88, 96, 98]
		if propID in is32bits:
			self.set_32(msgID, propID, value)
		else:
			self.set_16(msgID, propID, value)
			
	def set_32(self, msgID, propID, value):
		'''
		Set property to a given value for a 32 bit property.
		Avoid usage of this method. Use self.set_property instead.

			@param msgID: The puck or group whose property will be set.
			@type msgID: int
			@param propID: The number corresponding to the property to be set.
			@type propID: int
			@param value: The value to which the property will be set.
			@type value: int
		'''
		data = [0x80+propID, 0, value%0x100, int(value/0x100)%0x100, int(value/0x10000)%0x100, int(value/0x1000000)]
		#self.write_msg(msgID, data)
		return self.send_msg(msgID, data)

	def set_16(self, msgID, propID, value):
		'''
		Set property to a given value for a 16 bit property.
		Avoid usage of this method. Use self.set_property instead.
		
			@param msgID: The puck or group whose property will be set.
			@type msgID: int
			@param propID: The number corresponding to the property to be set.
			@type propID: int
			@param value: The value to which the property will be set.
			@type value: int
		'''
		data = [0x80+propID, 0, value%256, int(value/256)]
		#self.write_msg(msgID, data)
		return self.send_msg(msgID, data)


	# TODO: Allow a "GET" from a group.
	def get_property(self, msgID, propID):
		'''
		Get property from pucks in msgID.

			@param msgID: The puck whose property will be read from.
			@type msgID: int
			@param propID: The property be read from.
			@type propID: int
			@rtype: int
			@return: The value held in the property.
		'''
		is32bits = [48, 50, 52, 54, 56, 58, 66, 68, 74, 88, 96, 98]
		if propID in is32bits:
			return self.get_32(msgID, propID)
		else:
			if propID == TACT:
				return self.get_tact(msgID)
			return self.get_16(msgID, propID)


	def get_32(self, msgID, propID):
		'''
		Gets a 32 bit property. Please use get_property instead of this method where applicable.

			@param msgID: The puck whose property will be read from.
			@type msgID: int
			@param propID: The property be read from.
			@type propID: int
			@rtype: int
			@return: The value held in the property.
		'''
		self.write_msg(msgID, [propID])
		#time.sleep(0.005)
		read_result = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN, read_result[0], "read")
		data = read_result[1].DATA
		value = (0x1000000 * data[5]) + (0x0010000 * data[4]) + (0x0000100 * data[3]) + (0x0000001 * data[2])
		return value


	def get_16(self, msgID, propID):
		'''
		Gets a 16 bit property. Please use get_property instead of this method where applicable.
		
			@param msgID: The puck whose property will be read from.
			@type msgID: int
			@param propID: The property be read from.
			@type propID: int
			@rtype: int
			@return: The value held in the property.
		'''
		self.write_msg(msgID, [propID])
		#time.sleep(0.05)
		#time.sleep(0.005)
		read_result = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN, read_result[0], "read")
		data = read_result[1].DATA
		value =(0x0000100 * data[3]) + (0x0000001 * data[2])
		return value

		
	def save_property(self, msgID, propID):
		'''
		Save a property.

			@param msgID: The puck or group to have its property saved.
			@type msgID: int
			@param propID: The property to be saved.
			@type propID: int
		'''
		self.set_property(msgID, SAVE, propID)


	def load_property(self, msgID, propID):
		'''
		Load a property's value for puck's flash memory.

			@param msgID: The puck or group to have its property loaded.
			@type msgID: int
			@param propID: The property to be loaded.
			@type propID: int
		'''
		self.set_property(msgID, LOAD, propID)
		
	def get_prop_quick(self, msgID,propID,speed):
		'''
		Gets a property timed at a certain rate.

			@param msgID: The puck or group to have its property gotten.
			@type msgID: int
			@param propID: The property to be saved.
			@type propID: int
			@param speed: The time delay for the get.
			@type speed: float
		'''
		self.write_msg(msgID, [propID],speed)
		read_result=self.read_msg_resilient(msgID,propID)
		self.check_error(self.PCAN, read_result[0], "read")
		data = read_result[1].DATA
		value = (0x1000000 * data[5]) + (0x0010000 * data[4]) + (0x0000100 * data[3]) + (0x0000001 * data[2])
		return value

	def read_msg_resilient(self, expect_puck,expect_prop,max_recurse=10,counter=0):
		'''
		Reads message given the puckID and the propertyID. 
		It will read as normal, until it gets some expected output from the puck.

			@param expect_puck: The puck to read from.
			@type expect_puck: int
			@param expect_prop: The property read from.
			@type expect_prop: int
			@param max_recurse: The most number of times to repeat the get.
			@type max_recurse: int
			@param counter: Used internally. Do not set.
			@type counter: int
			@rtype: int
			@return: The value held in the property of the given puck.
		'''
		counter+=1
		response=self.PCAN.Read(PCAN_USBBUS1)
		received_prop=response[1].DATA[0]-128
		received_puck=(response[1].ID-1024)>>5
		
		if (received_prop==expect_prop) and (received_puck==expect_puck):
			return response
		else:
			print "a"
			if counter<max_recurse:
				return self.read_msg_resilient(expect_puck,expect_prop,counter=counter)
			else:
				raise Exception("Missed message")

	def get_role(self, msgID):
		'''
		Read from ROLE property and return something that makes sense.
		Returns an array of holding the following data:

		[4-bit Product Identifier,
		Internal Thermistor,
		20 MHz (vs 32 MHz),
		Hall Motor Encoder,
		Enc Motor Encoder,
		Strain Gauge,
		IMU for Force-Torque Sensor,
		Optical Motor Encoder]

			@param msgID: The puck to get the ROLE from.
			@type msgID: int
			@rtype: Array[int,bool,bool,bool,bool,bool,bool,bool]
			@return: An array holding the above values.
		'''
		role = self.get_property(msgID, 1)
		data= [0,0,0,0,0,0,0,0,0]
		data[0] = role%16               # 4-bit Product Identifier
		data[1] = int(role/2**6)%2==1   # Internal Thermistor Bit
		data[2] = int(role/2**7)%2==1   # 20 MHz (vs 32 MHz)
		data[3] = int(role/2**8)%2==1   # Is there a Dig+Ser Motor Encoder?
		data[4] = int(role/2**9)%2==1   # Is there a Hall Motor Encoder?
		data[5] = int(role/2**10)%2==1  # Is there an Enc Motor Encoder?
		data[6] = int(role/2**11)%2==1  # Is there a Strain Gauge?
		data[7] = int(role/2**12)%2==1  # IMU for Force-Torque Sensor
		data[8] = int(role/2**13)%2==1  # Optical Motor Encoder
		return data

	def get_mode(self, msgID):
		'''
		Read from MODE property, and return a tuple with the number corresponding to the mode, along with
		the string name of the mode.

			@param msgID: The puck from which to get the mode.
			@type msgID: int
			@rtype: Tuple(int, str)
			@return: A tuple with the number and name of the mode.
		'''
		m = self.get_property(msgID, MODE)
		if m==MODE_IDLE:
			return (MODE_IDLE, "IDLE")
		elif m==MODE_TORQUE:
			return (MODE_TORQUE, "TORQUE")
		elif m==MODE_PID:
			return (MODE_PID, "PID")
		elif m==MODE_VEL:
			return (MODE_VEL, "VEL")
		elif m==MODE_TRAP:
			return (MODE_TRAP, "TRAP")
		else:
			print "Invalid get_mode() operation: "+str(m)
			return (m, "???")

	def set_mode(self, msgID, value):
		'''
		Set the mode property using either strings or numbers.

			@param msgID: The puck or group to set the mode.
			@type msgID: int
			@param value: The value to which the mode should be set.
			@type value: int
		'''
		modes = {"IDLE":MODE_IDLE, "TORQUE":MODE_TORQUE, "PID":MODE_PID, "VEL":MODE_VEL, "TRAP":MODE_TRAP}
		m = value
		if m in modes:
			m = modes[m]
		self.set_property(msgID, MODE, m)
		
	# I'm sorry that this method is so unbearably long, but it won't compile otherwise because of circular imports TT_TT
	def set_puck_like(self, puckID, virtID):
		'''
		Set the puck to have all the default properties of the indicated puck ID.

			@param puckID: The original puck to change.
			@type puckID: int
			@param virtID: The ID of the puck to load defaults from.
			@type virtID: int
		'''
		self.set_property(puckID, MODE, MODE_IDLE)
		# Set universal puck properties. (taterDefs[])
		# if virtID in range(1,8)
		#     self.set_property(puckID, TIE, 0)
		#     self.set_property(puckID, ACCEL, 100)
		#     self.set_property(puckID, AP, 0)
		#     self.set_property(puckID, CT, 4096)
		#     self.set_property(puckID, OT, 0)
		#     self.set_property(puckID, CTS, 4096)
		#     self.set_property(puckID, DP, 0)
		#     self.set_property(puckID, MV, 100)
		#     self.set_property(puckID, MCV, 100)
		#     self.set_property(puckID, MOV, 100)
		#     self.set_property(puckID, OT, 0)
		#     self.set_property(puckID, HOLD, 0)
		#     self.set_property(puckID, TSTOP, 0)
		#     self.set_property(puckID, OTEMP, 60)
		#     self.set_property(puckID, PTEMP, 0)
		#     self.set_property(puckID, DS, -256)
		#     self.set_property(puckID, KP, 2000)
		#     self.set_property(puckID, KD, 8000)
		#     self.set_property(puckID, KI, 0)
			#wamDefaultMT has yet to be implemented correctly. The interns coding this don't really care about the WAM, so this will be put off until someone does.
		
		#Set Barrett Hand Defaults
		is_280 = self.get_property(puckID, HALLH)==7 #Identifier for 280 version
		if virtID in [FINGER1, FINGER2, FINGER3, SPREAD]:
			self.set_property(puckID, JIDX, virtID-3)
			self.save_property(puckID, JIDX)
			self.set_property(puckID, PIDX, virtID-10)
			self.save_property(puckID, PIDX)
			self.set_property(puckID, TIE, 0)
			self.save_property(puckID, TIE)
			self.set_property(puckID, ACCEL, 200)
			self.save_property(puckID, ACCEL)
			#self.set_property(puckID, AP, 0)
			#self.save_property(puckID, AP)
			self.set_property(puckID, OT, 0)
			self.save_property(puckID, OT)
			self.set_property(puckID, CTS, 4096)
			self.save_property(puckID, CTS)
			self.set_property(puckID, MT, 2200)
			self.save_property(puckID, MT)
			self.set_property(puckID, MCV, 200)
			self.save_property(puckID, MCV)
			self.set_property(puckID, MOV, 200)
			self.save_property(puckID, MOV)
			self.set_property(puckID, OTEMP, 60)
			self.save_property(puckID, OTEMP)
			self.set_property(puckID, PTEMP, 0)
			self.save_property(puckID, PTEMP)
			self.set_property(puckID, POLES, 6)
			self.save_property(puckID, POLES)
			self.set_property(puckID, IKCOR, 102)
			self.save_property(puckID, IKCOR)
			self.set_property(puckID, IOFF, 0)
			self.save_property(puckID, IOFF)
			self.set_property(puckID, IVEL, -75)
			self.save_property(puckID, IVEL)
			self.set_property(puckID, DS, 25600)
			self.save_property(puckID, DS)
			self.set_property(puckID, KI, 0)
			self.save_property(puckID, KI)
			self.set_property(puckID, IPNM, 20000)
			self.save_property(puckID, IPNM)
			self.set_property(puckID, GRPA, 0)
			self.save_property(puckID, GRPA)
			self.set_property(puckID, GRPB, 7)
			self.save_property(puckID, GRPB)
			self.set_property(puckID, GRPC, 5)
			self.save_property(puckID, GRPC)
			self.set_property(puckID, IKI, 204)
			self.save_property(puckID, IKI)
			self.set_property(puckID, IKP, 500)
			self.save_property(puckID, IKP)
			if virtID == SPREAD:
				self.set_property(puckID, CT, 35950)
				self.save_property(puckID, CT)
				self.set_property(puckID, DP, 17975)
				self.save_property(puckID, DP)
				self.set_property(puckID, MV, 50)
				self.save_property(puckID, MV)
				self.set_property(puckID, HSG, 0)
				self.save_property(puckID, HSG)
				self.set_property(puckID, LSG, 0)
				self.save_property(puckID, LSG)
				self.set_property(puckID, HOLD, 1)
				self.save_property(puckID, HOLD)
				self.set_property(puckID, TSTOP, 150)
				self.save_property(puckID, TSTOP)
				self.set_property(puckID, KP, 1000)
				self.save_property(puckID, KP)
				self.set_property(puckID, KD, 10000)
				self.save_property(puckID, KD)
			else:
				self.set_property(puckID, CT, 195000)
				self.save_property(puckID, CT)
				self.set_property(puckID, DP, 45000)
				self.save_property(puckID, DP)
				self.set_property(puckID, MV, 200)
				self.save_property(puckID, MV)
				self.set_property(puckID, HSG, 0)
				self.save_property(puckID, HSG)
				self.set_property(puckID, LSG, 0)
				self.save_property(puckID, LSG)
				self.set_property(puckID, HOLD, 0)
				self.save_property(puckID, HOLD)
				self.set_property(puckID, TSTOP, 50)
				self.save_property(puckID, TSTOP)
				self.set_property(puckID, KP, 500)
				self.save_property(puckID, KP)
				self.set_property(puckID, KD, 2500)
				self.save_property(puckID, KD)
		else:
			print "Invalid Puck Id for Hand"
		
	#============================MOVING_STUFF================================

	def set_hand_targets(self, f1_target, f2_target, f3_target, sp_target):
		'''
		Given fingers and spread target values, move the hand to that position.
		Will mainly be used to load user-defined hand positions.
		Takes Barrett Units as inputs.

			@param f1_target: The position (in encoder ticks) for finger 1 to move to.
			@type f1_target: int
			@param f2_target: The position for finger 2 to move to.
			@type f2_target: int
			@param f3_target: The position for finger 3 to move to.
			@type f3_target: int
			@param sp_target: The position for spread to move to.
			@type sp_target: int
		'''
		self.set_property(FINGER1, DP, f1_target)
		self.set_property(FINGER2, DP, f2_target)
		self.set_property(FINGER3, DP, f3_target)
		tar = self.get_position(SPREAD)
		self.set_property(SPREAD, DP, tar)
		self.set_property(0x405, CMD, CMD_MOVE)
		time.sleep(2)
		self.set_property(SPREAD, DP, sp_target)
		self.set_property(SPREAD, CMD, CMD_MOVE)
		time.sleep(1)

	def move_to(self, puckID, target, autowait=True):
		'''
		Move the motor to a specific position.

			@param puckID: The puck to move.
			@type puckID: int
			@param target: The end position to move to.
			@type target: int
			@param autowait: Does the program wait until the motor is done moving?
			@type autowait: bool
		'''
		self.set_property(puckID, M, target)
		if autowait:
			self.wait_done_moving([puckID])

	def done_moving(self, motors_to_check=ALL_FINGERS):
		'''
		Checks a given list of motors once to see if they have stopped moving, and if so, it returns true

			@param motors_to_check: A list of motors to check.
			@type motors_to_check: Array[*int]
			@rtype: bool
			@return: Whether or not the motors are done moving.
		'''
		for FINGER in motors_to_check:
			if (self.get_mode(FINGER)[1]!="IDLE" and (self.get_mode(FINGER)[1]!="PID" or self.get_property(FINGER,77)==0)):
				return False
		return True

	# TODO: Ensure that TSTOP > 0. Otherwise, this may be an infinite loop.
	def wait_done_moving(self, motors_to_check=ALL_FINGERS):
		'''
		Waits until the given list of motors have all stopped moving.

			@param motors_to_check: A list of motors to wait for.
			@type motors_to_check: Array[*int]
		'''
		while(not self.done_moving(motors_to_check)):
			time.sleep(0.025)

	def detect_breakaway(self, finger):
		'''

			@return: True if the finger has broken away, False if it hasn't
		'''
		tup = self.get_packed_position(finger)
		ratio = tup[0]/tup[1]
		return (ratio>3)

	def open_grasp(self):
		'''
		Opens all fingers to the position encoded by Open Target (OT)
		'''
		self.set_property(0x405, TSTOP, 50)
		self.set_property(SPREAD, TSTOP, 150)

		open_target=self.get_property(FINGER1,OT)
		self.set_property(FINGER1,DP,open_target)
		
		open_target=self.get_property(FINGER2,OT)
		self.set_property(FINGER2,DP,open_target)
		
		open_target=self.get_property(FINGER3,OT)
		self.set_property(FINGER3,DP,open_target)

		spread_stay=self.get_position(SPREAD)
		self.set_property(SPREAD,DP,spread_stay)
		
		self.set_property(0x405,CMD,CMD_MOVE)
		self.wait_done_moving(GRASP)

	def close_grasp(self):
		'''
		Closes all fingers to the position encoded by Close Target (CT).
		'''
		self.set_property(0x405, TSTOP, 50)
		self.set_property(SPREAD, TSTOP, 150)

		close_target=self.get_property(FINGER1,CT)
		self.set_property(FINGER1,DP,close_target)

		close_target=self.get_property(FINGER2,CT)
		self.set_property(FINGER2,DP,close_target)

		close_target=self.get_property(FINGER3,CT)
		self.set_property(FINGER3,DP,close_target)

		spread_stay=self.get_position(SPREAD)
		self.set_property(SPREAD,DP,spread_stay)

		self.set_property(0x405,CMD,CMD_MOVE)
		self.wait_done_moving(GRASP)
		
	def open_spread(self):
		'''
		Open spread to position determined by Open Target (OT).
		'''
		self.set_property(SPREAD, TSTOP, 150)
		self.set_property(SPREAD, CMD, CMD_OPEN)
		self.wait_done_moving([SPREAD])

	def close_spread(self):
		'''
		Close spread to position determined by Close Target (CT).
		'''
		self.set_property(SPREAD, CMD, CMD_CLOSE)
		self.wait_done_moving([SPREAD])

	def open_finger(self, puckID, autowait=True):
		'''
		Open finger and wait for completion.

			@param puckID: Finger to be opened.
			@type puckID: int
			@param autowait: calls wait_done_moving if True. Defaults to True.
			@type autowait: bool
		'''
		if puckID in [FINGER1, FINGER2, FINGER3]:
			self.set_property(puckID, TSTOP, 50)
		if puckID == SPREAD:
			self.set_property(puckID, TSTOP, 150)
		self.set_property(puckID, CMD, CMD_OPEN)
		if autowait:
			self.wait_done_moving([puckID])

	def close_finger(self, puckID, autowait=True):
		'''
		Close finger and wait for completion.

		@param puckID: Finger to be closed.
		@type puckID: int
		@param autowait: calls wait_done_moving if True. Defaults to True.
		@type autowait: bool
		'''
		if puckID in [FINGER1, FINGER2, FINGER3]:
			self.set_property(puckID, TSTOP, 50)
		if puckID == SPREAD:
			self.set_property(puckID, TSTOP, 150)
		self.set_property(puckID, CMD, CMD_CLOSE)
		if autowait:
			self.wait_done_moving([puckID])

	def move_grasp(self, position = -1):
		'''
		Moves all fingers to input argument or default position (50).

			@param position: position of fingers. Defaults to -1. Valid position range is from 0-195,000 encoder counts.
			@type position: int
		'''
		default = self.get_position(SPREAD)
		self.set_property(SPREAD, DP, default)
		if(position != -1):
			self.set_property(FINGER1, DP, position)
			self.set_property(FINGER2, DP, position)
			self.set_property(FINGER3, DP, position)	
		self.move()

	def move(self):
		'''
		Moves all fingers/spread to their default.
		'''
		self.set_property(0x405, CMD, CMD_MOVE)
		self.wait_done_moving(GRASP)

	def open_all(self):
		'''
		Opens every fingers at once. Mainly used in DEMO. WARNING: can be dangerous because it may cause fingers to collide if the hand is in an unknown position.
		'''
		open_target = self.get_property(FINGER1, OT)
		self.set_property(FINGER1, DP, open_target)
		open_target = self.get_property(FINGER2, OT)
		self.set_property(FINGER2, DP, open_target)
		open_target = self.get_property(FINGER3, OT)
		self.set_property(FINGER3, DP, open_target)
		open_target = self.get_property(SPREAD, OT)
		self.set_property(SPREAD, DP, open_target)
		self.move()

	def close_all(self):
		'''
		Closes every finger at once. Mainly for use in DEMO. WARNING: can be dangerous because it may cause fingers to collide if the hand is in an unknown position.
		'''
		close_target = self.get_property(FINGER1, CT)
		self.set_property(FINGER1, DP, close_target)
		close_target = self.get_property(FINGER2, CT)
		self.set_property(FINGER2, DP, close_target)
		close_target = self.get_property(FINGER3, CT)
		self.set_property(FINGER3, DP, close_target)
		close_target = self.get_property(SPREAD, CT)
		self.set_property(SPREAD, DP, close_target)
		self.move()

	#============================INCREMENTALLY_MOVE_STUFF========================
		
	def open_grasp_step(self, step=0):
		'''
		Open grasp by input increment.

			@param step: size of increment in encoder counts. Defaults to 0.
			@type step: int
		'''
		self.open_finger_step(FINGER1, step, False)
		self.open_finger_step(FINGER2, step, False)
		self.open_finger_step(FINGER3, step, False)
		self.wait_done_moving(GRASP)

	def close_grasp_step(self, step=0):
		'''
		Close grasp by input decrement.

			@param step: size of decrement in encoder counts. Defaults to 0.
			@type step: int
		'''
		self.close_finger_step(FINGER1, step, False)
		self.close_finger_step(FINGER2, step, False)
		self.close_finger_step(FINGER3, step, False)
		self.wait_done_moving(GRASP)

	def open_spread_step(self, step=-1):
		'''
		Open spread by input increment.

			@param step: size of increment in encoder counts. Defaults to -1.
			@type step: int
		'''
		if step == -1:
			step = self.get_property(SPREAD, 60)
		self.open_finger_step(SPREAD, step)
		
	def close_spread_step(self, step=-1):
		'''
		Close spread by input decrement.

			@param step: size of decrement in encoder counts. Defaults to -1.
			@type step: int 
		'''
		if step == -1:
			step = self.get_property(SPREAD, 60)
		self.close_finger_step(SPREAD, step)

	def open_finger_step(self, puckID, step=-1, autowait=True):
		'''
		Open finger by input increment.

			@param puckID: Finger to be opened.
			@type puckID: int
			@param step: size of increment in encoder counts. Defaults to -1.
			@type step: int
			@param autowait: calls wait_done_moving if True. Defaults to True.
			@type autowait: bool
		'''
		if step == -1:
			step = self.get_property(puckID, 60)
		self.set_property(puckID, DS, step)
		self.set_property(puckID, CMD, CMD_IO)
		if autowait:
			self.wait_done_moving([puckID])

	def close_finger_step(self, puckID, step=-1, autowait=True):
		'''
		Close finger by input decrement.

			@param puckID: Finger to be closed.
			@type puckID: int
			@param step: size of decrement in encoder counts. Defaults to -1.
			@type step: int
			@param autowait: calls wait_done_moving if True. Defaults to True.
			@type autowait: bool
		'''
		if step == -1:
			step = self.get_property(puckID, 60)
		self.set_property(puckID, DS, step)
		self.set_property(puckID, CMD, CMD_IC)
		if autowait:
			self.wait_done_moving([puckID])

	#===========================NON_TRIVIAL FUNCTIONS=============================
	def get_full_pos_packet(msgID):
			self.write_msg(msgID, [P])
			read_result=self.PCAN.Read(PCAN_USBBUS1)
			return read_result
			
	def get_velocity(self, msgID):
		'''
		Returns velocity values of finger when in motion. Mostly returns garbage. It's used to help tell when finger is stopped or near to it.

			@param msgID: The puck or group to get velocity.
			@type msgID: int
			@rtype: float
			@return: A (garbage) value representing the approximate velocity of the finger.
		'''    	
		packet1=self.get_full_pos_packet(msgID)
		packet2=self.get_full_pos_packet(msgID)
		
		error1=packet1[0]
		error2=packet2[0]

		msg1=packet1[1]
		msg2=packet2[1]

		data1=msg1.DATA 
		data2=msg2.DATA

		time1=packet1[2]
		time2=packet2[2]
		
		stamp1=time1.micros + 1000 * time1.millis + 0xFFFFFFFF * 1000 * time1.millis_overflow
		stamp2=time2.micros + 1000 * time2.millis + 0xFFFFFFFF * 1000 * time2.millis_overflow

		delta=(stamp2-stamp1)/1000000.0
		
		self.check_error(self.PCAN,error1,"reading position for fake get_velocity")
		self.check_error(self.PCAN,error2,"reading position for fake get_velocity")

		val1=(0x0000100 * data1[3]) + (0x0000001 * data1[2])
		val2=(0x0000100 * data2[3]) + (0x0000001 * data2[2])
		
		return (val2-val1)/delta

	def get_temp(self, msgID):
		'''
		Gets temperature value for all pucks in msgID.

			@param msgID: The puck or group to get temp.
			@type msgID: int
			@rtype: int
			@return: The value of the TEMP property.
		'''
		return self.temp[msgID]['temp']
		
		
	def read_temp(self, msgID):
		'''
		Sends a message to get the temperature value for all pucks in msgID.

			@param msgID: The puck or group to get temp.
			@type msgID: int
			@rtype: int
			@return: The value of the TEMP property.
		'''
		return self.send_msg(msgID, [TEMP])

	def get_therm(self, msgID):
		'''
		Gets motor temperature value for all pucks in msgID. 

			@param msgID: The puck or group to get motor temperature.
			@text msgID: int
			@rtype: int
			@return: The value of the THERM property.
		'''
		return self.temp[msgID]['therm']
	
	def read_therm(self, msgID):
		'''
		Gets motor temperature value for all pucks in msgID. 

			@param msgID: The puck or group to get motor temperature.
			@text msgID: int
			@rtype: int
			@return: The value of the THERM property.
		'''
		return self.send_msg(msgID, [THERM])

	def get_top_tact(self, msgID):
		'''
		Unpack the top10 values from TACT.
		Returns a dictionary with 10 items like (sensor number):(tact value).

			@param msgID: The puck or group to get top 10 tactile data.
			@type msgID: int
			@rtype: Dictionary{sensorID:value}
			@return topVals: Dictionary of the top 10 tactile array sensor values. 
		'''
		# Set TACT(106) to top10 mode (1)
		self.set_property(msgID, TACT, TACT_10) 
		self.write_msg(msgID, [TACT]) #GET TACT
		read_result=self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN,read_result[0],"reading top ten tactile values")
		#output is mapped to here.
		output = read_result[1].DATA 
		#parsing this output
		top10 = output[0] * 0x10000 + output[1] * 0x100 + output[2] * 0x1
		topVals = {}
		data = [output[3]/(0x10), output[3]%(0x10), output[4]/(0x10), output[4]%(0x10), output[5]/(0x10), output[5]%(0x10)]
		count=0
		for sensor in range(0, 24):
			if top10%2 == 1:
				#print 'sensor = %d, count = %d'%(sensor, count)
				topVals[sensor] = data[count]
				count+=1
			top10 = top10/2
			# Each bit represents one of the top 10 pressures for purposes of efficiency. top10 has the last bit sliced
		return topVals

	def read_full_tact(self, msgID):
		'''
			Read all tactile sensors
		'''
		return self.set_property(msgID, TACT, TACT_FULL) 

	def get_full_tact(self, msgID):
		'''
		Unpack all tactile sensor values in an array.

			@param msgID: The puck or group to get full tactile array sensor data.
			@type msgID: int
			@rtype: Array[*data]
			@return: An array containing the tactile data from a given puck.
		'''
		# Set TACT(106) to full mode (2)
		'''self.set_property(msgID, TACT, TACT_FULL) 
		#self.write_msg(msgID, [TACT])
		output = [0,0,0,0,0]
		read_result = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN,read_result[0],"reading full tactile data")
		read_result2 = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN,read_result2[0],"reading full tactile data")
		read_result3 = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN,read_result3[0],"reading full tactile data")
		read_result4 = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN,read_result4[0],"reading full tactile data")
		read_result5 = self.PCAN.Read(PCAN_USBBUS1)
		self.check_error(self.PCAN,read_result5[0],"reading full tactile data")
		
		output[0] = read_result[1].DATA
		output[1] = read_result2[1].DATA
		output[2] = read_result3[1].DATA
		output[3] = read_result4[1].DATA
		output[4] = read_result5[1].DATA
		
		#print 'Init: ID1 = %x,  ID2 = %x,  ID3 = %x,  ID4 = %x,  ID5 = %x'%(read_result[1].ID, read_result2[1].ID, read_result3[1].ID, read_result4[1].ID, read_result5[1].ID)
		tactileVals = range(0,24)
		index_ = 0
		for data in output:
			index_ = int(data[0]/16) * 5
			#print 'index = %d, data[0] = %x'%(index_, data[0])
			# Get the bits and then unpack them.
			tactileVals[index_ + 0] = round(((data[0]%0x10)*0x100 + data[1])/256.0,2)
			tactileVals[index_ + 1] = round((data[2]*0x10 + int(data[3]/0x10))/256.0,2)
			tactileVals[index_ + 2] = round(((data[3]%0x10)*0x100 + data[4])/256.0,2)
			tactileVals[index_ + 3] = round((data[5]*0x10 + int(data[6]/0x10))/256.0,2)
			if index_ != 20:
				tactileVals[index_ + 4] = round(((data[6]%0x10)*0x100 + data[7])/256.0,2)
		#print 'Return OK : %s'%(tactileVals)
		return tactileVals'''
		
		return self.tactile_sensor[msgID]['values']


	def get_tact(self, msgID, topOrFull="TOP10"):
		'''
		Obtain and interpret tactile sensor data.

			@param msgID: The puck or group to get full or top 10 tactile array sensor data.
			@type msgID: int
			@param topOrFull: To get full data, enter "FULL". To get the top 10 values, enter "TOP10". Or anything else, really.
			@type topOrFull: str
			@return 
		'''
		if topOrFull == "FULL":
			return self.get_full_tact(msgID)
		else:
			return self.get_top_tact(msgID)

	def set_velocity(self, puckID, velocity):
		'''
		Set the velocity and make the motor move.

			@param puckID: The ID of the puck to set the velocity of.
			@type puckID: int
			@param velocity: The velocity (in cts/ms) of the motor.
			@type velocity: int
		'''
		#First set TSTOP to 0.
		self.set_property(puckID, TSTOP, 0)
		#Set Velocity
		self.set_property(puckID, V, velocity)
		#Set mode to allow the puck to move.
		self.set_property(puckID, MODE, MODE_VEL)


	def get_strain(self, msgID):
		'''
		Gets the fingertip torque sensor value. 

			@param msgID: The puck or group to get fingertip torque sensor data. 
			@type msgID: int
			@rtype: int
			@return: Strain Gauge Reading
		'''
		return self.strain[msgID]
	
	
	def read_strain(self, msgID):
		'''
		Sends the message to get the fingertip torque sensor value. 

			@param msgID: The puck or group to get fingertip torque sensor data. 
			@type msgID: int
			@rtype: int
			@return: CAN status
		'''
		return self.send_msg(msgID, [SG])
		
	

	def onescomp(self, binstr):
		return ''.join('1' if b=='0' else '0' for b in binstr)

	def twoscomp(self, number):
		binstr= bin(number)[2:]
		a= bin(int(self.onescomp(binstr),2)+1)[2:]
		return -1*int(a,2)

	def get_position(self, msgID, depth=0):
		'''
		Get packed position data and return it.

			@param msgID: The puck or group to get position data.
			@type msgID: int
			@rtype: int

			@param depth: number of times get message was retried.
			
			@return: The position of the finger in encoder counts.
		'''
		if depth!=0:
			self.write_msg(msgID, [P],.009)
		read_result=self.PCAN.Read(PCAN_USBBUS1)
		try:
			self.check_error(self.PCAN,read_result[0],"getting position data")
			received_puck=(read_result[1].ID-1024)>>5
			if received_puck!=msgID:
				raise Exception("Did not read expected MSGID")
		except:
			if depth>10:
				raise Exception("Failure to get position data.")
			else:
				return self.get_position(msgID, depth+1)

		output = read_result[1].DATA
		temp=(output[0]-0x80)*0x10000 + output[1] * 0x100 + output[2]

		if (temp & 0b1000000000000000000000): 
			return self.twoscomp(temp)
		else:
			return temp

	def get_packed_position(self, msgID):
		'''
		Get packed position data and return both P and JP.

			@param msgID: The puck or group to get position data.
			@type msgID: int
			@rtype: (int, int)

			@return: The position and joint position of the finger in encoder counts.
					 Position in radians
		'''
		
		return  self.motor_positions[msgID]['position']
	
	
	def read_packed_position(self, msgID):
		'''
		Get packed position data and return both P and JP.

			@param msgID: The puck or group to get position data.
			@type msgID: int
			@rtype: (int, int)

			@return: sends a msg to read the position.
		'''
		
		return self.send_msg(msgID, [P])
		
	
	def process_can_messages(self):
		''' 
			Reads and process all the msgs in the bus
			Depending on the CAN id, it'll use different methods
		'''
		ret = 0
		# Reads a can msg
		msg = self.read_msg()
		
		# No read messages
		if msg[0] != PCAN_ERROR_OK:
			ret = -1
		
		while msg[0] == PCAN_ERROR_OK:
			
			can_id = msg[1].ID
			#print 'process_can_messages: CAN ID = %x'%can_id
			
			if can_id in [F1_POSITION, F2_POSITION, F3_POSITION, SPREAD_POSITION]:
				self.process_packed_position(msg[1])
			elif can_id in [F1_STRAIN, F2_STRAIN, F3_STRAIN] and msg[1].DATA[0] == STRAIN_ID:
				self.process_strain(msg[1])
			elif can_id in [F1_MOTOR_TEMP, F2_MOTOR_TEMP, F3_MOTOR_TEMP, SPREAD_MOTOR_TEMP] and msg[1].DATA[0] == MOTOR_TEMP_ID:
				self.process_motor_temp(msg[1])
			elif can_id in [F1_MOTOR_THERM, F2_MOTOR_THERM, F3_MOTOR_THERM, SPREAD_MOTOR_THERM] and msg[1].DATA[0] == MOTOR_THERM_ID:
				self.process_motor_therm(msg[1])
			elif can_id in [F1_TACT, F2_TACT, F3_TACT, PALM_TACT]:
				self.process_full_tact(msg[1])
				
			msg = self.read_msg()
			
		return ret
	
	def process_packed_position(self, msg):
		'''
			Process the CAN msgs and saves the position depending on the MSG ID
		'''
		data = msg.DATA
		pos = (data[0]-0x80)*0x10000 + data[1]*0x100 + data[2]
		jpos= (data[3]-0x80)*0x10000 + data[4]*0x100 + data[5]
		pos = self.twoscomp(pos) if pos & 0b1000000000000000000000 else pos
		jpos= self.twoscomp(jpos) if jpos & 0b1000000000000000000000 else jpos
		
		if msg.ID == F1_POSITION:
			self.motor_positions[FINGER1]['encoder'][0] = pos
			self.motor_positions[FINGER1]['encoder'][1] = jpos
			self.motor_positions[FINGER1]['position'][0] = self.enc_to_rad(pos, BASE_TYPE)
			self.motor_positions[FINGER1]['position'][1] = self.enc_to_rad(jpos, BASE_TYPE)
			#print 'F1'
			
		elif msg.ID == F2_POSITION:
			self.motor_positions[FINGER2]['encoder'][0] = pos
			self.motor_positions[FINGER2]['encoder'][1] = jpos
			self.motor_positions[FINGER2]['position'][0] = self.enc_to_rad(pos, BASE_TYPE)
			self.motor_positions[FINGER2]['position'][1] = self.enc_to_rad(jpos, BASE_TYPE)
			#print 'F2'
			
		elif msg.ID == F3_POSITION:
			self.motor_positions[FINGER3]['encoder'][0] = pos
			self.motor_positions[FINGER3]['encoder'][1] = jpos
			self.motor_positions[FINGER3]['position'][0] = self.enc_to_rad(pos, BASE_TYPE)
			self.motor_positions[FINGER3]['position'][1] = self.enc_to_rad(jpos, BASE_TYPE)
			#print 'F3 = %f, %f'%(self.motor_positions[FINGER3]['position'][0], self.motor_positions[FINGER3]['position'][1])
			#print 'F3'
			
		elif msg.ID == SPREAD_POSITION:
			self.motor_positions[SPREAD]['encoder'][0] = pos
			self.motor_positions[SPREAD]['encoder'][1] = jpos
			self.motor_positions[SPREAD]['position'][0] = self.enc_to_rad(pos, SPREAD_TYPE)
			self.motor_positions[SPREAD]['position'][1] = self.enc_to_rad(jpos, SPREAD_TYPE)
			#print 'SPREAD = %f, %f'%(self.motor_positions[SPREAD]['position'][0], self.motor_positions[SPREAD]['position'][1])
			
	
	def process_strain(self, msg):
		'''
			Process the msg and extract the strain value depending on the CAN ID
		'''
		data = msg.DATA
		value =(0x0000100 * data[3]) + (0x0000001 * data[2])
		
		if msg.ID == F1_STRAIN:
			self.strain[FINGER1] = value
		if msg.ID == F2_STRAIN:
			self.strain[FINGER2] = value
		if msg.ID == F3_STRAIN:
			self.strain[FINGER3] = value
	
	def process_motor_temp(self, msg):
		'''
			Process the msg and extract the temperature of the motor puck on the CAN ID
		'''
		data = msg.DATA
		value =(0x0000100 * data[3]) + (0x0000001 * data[2])
		
		if msg.ID == F1_MOTOR_TEMP:
			self.temp[FINGER1]['temp'] = value
			
		if msg.ID == F2_MOTOR_TEMP:
			self.temp[FINGER2]['temp'] = value
			
		if msg.ID == F3_MOTOR_TEMP:
			self.temp[FINGER3]['temp'] = value
			
		if msg.ID == SPREAD_MOTOR_TEMP:
			self.temp[SPREAD]['temp'] = value
			#print 'Motor Temp S'
			
	def process_motor_therm(self, msg):
		'''
			Process the msg and extract the temperature of the motor on the CAN ID
		'''
		data = msg.DATA
		value =(0x0000100 * data[3]) + (0x0000001 * data[2])
		
		if msg.ID == F1_MOTOR_THERM:
			self.temp[FINGER1]['therm'] = value
			
		if msg.ID == F2_MOTOR_THERM:
			self.temp[FINGER2]['therm'] = value
			
		if msg.ID == F3_MOTOR_THERM:
			self.temp[FINGER3]['therm'] = value
			
		if msg.ID == SPREAD_MOTOR_THERM:
			self.temp[SPREAD]['therm'] = value
			#print 'Motor Temp S'
	
	def process_full_tact(self, msg):
		'''
			Process and saves all the messages containing the tactile information
		'''
		
		data = msg.DATA
		
		if msg.ID == F1_TACT:
			if data[0] >= 0x00 and data[0] < 0x10:
				self.tactile_sensor[FINGER1]['data'][0] = data			
			if data[0] >= 0x10 and data[0] < 0x20:
				self.tactile_sensor[FINGER1]['data'][1] = data
			if data[0] >= 0x20 and data[0] < 0x30:
				self.tactile_sensor[FINGER1]['data'][2] = data
			if data[0] >= 0x30 and data[0] < 0x40:
				self.tactile_sensor[FINGER1]['data'][3] = data
			if data[0] >= 0x40:
				self.tactile_sensor[FINGER1]['data'][4] = data
				self.tactile_sensor[FINGER1]['values'] = self.process_tactile_data(self.tactile_sensor[FINGER1]['data'])
		if msg.ID == F2_TACT:
			if data[0] >= 0x00 and data[0] < 0x10:
				self.tactile_sensor[FINGER2]['data'][0] = data			
			if data[0] >= 0x10 and data[0] < 0x20:
				self.tactile_sensor[FINGER2]['data'][1] = data
			if data[0] >= 0x20 and data[0] < 0x30:
				self.tactile_sensor[FINGER2]['data'][2] = data
			if data[0] >= 0x30 and data[0] < 0x40:
				self.tactile_sensor[FINGER2]['data'][3] = data
			if data[0] >= 0x40:
				self.tactile_sensor[FINGER2]['data'][4] = data
				self.tactile_sensor[FINGER2]['values'] = self.process_tactile_data(self.tactile_sensor[FINGER2]['data'])
		if msg.ID == F3_TACT:
			if data[0] >= 0x00 and data[0] < 0x10:
				self.tactile_sensor[FINGER3]['data'][0] = data			
			if data[0] >= 0x10 and data[0] < 0x20:
				self.tactile_sensor[FINGER3]['data'][1] = data
			if data[0] >= 0x20 and data[0] < 0x30:
				self.tactile_sensor[FINGER3]['data'][2] = data
			if data[0] >= 0x30 and data[0] < 0x40:
				self.tactile_sensor[FINGER3]['data'][3] = data
			if data[0] >= 0x40:
				self.tactile_sensor[FINGER3]['data'][4] = data
				self.tactile_sensor[FINGER3]['values'] = self.process_tactile_data(self.tactile_sensor[FINGER3]['data'])
		if msg.ID == PALM_TACT:
			if data[0] >= 0x00 and data[0] < 0x10:
				self.tactile_sensor[SPREAD]['data'][0] = data			
			if data[0] >= 0x10 and data[0] < 0x20:
				self.tactile_sensor[SPREAD]['data'][1] = data
			if data[0] >= 0x20 and data[0] < 0x30:
				self.tactile_sensor[SPREAD]['data'][2] = data
			if data[0] >= 0x30 and data[0] < 0x40:
				self.tactile_sensor[SPREAD]['data'][3] = data
			if data[0] >= 0x40:
				self.tactile_sensor[SPREAD]['data'][4] = data
				self.tactile_sensor[SPREAD]['values'] = self.process_tactile_data(self.tactile_sensor[SPREAD]['data'])
				
	def process_tactile_data(self, data_array):
		'''
			Process the array of data and returns a tactile array
		'''
		tactileVals = range(0,24)
		index_ = 0
		for data in data_array:
			index_ = int(data[0]/16) * 5
			#print 'index = %d, data[0] = %x'%(index_, data[0])
			# Get the bits and then unpack them.
			tactileVals[index_ + 0] = round(((data[0]%0x10)*0x100 + data[1])/256.0,2)
			tactileVals[index_ + 1] = round((data[2]*0x10 + int(data[3]/0x10))/256.0,2)
			tactileVals[index_ + 2] = round(((data[3]%0x10)*0x100 + data[4])/256.0,2)
			tactileVals[index_ + 3] = round((data[5]*0x10 + int(data[6]/0x10))/256.0,2)
			if index_ != 20:
				tactileVals[index_ + 4] = round(((data[6]%0x10)*0x100 + data[7])/256.0,2)
		#print 'Return OK : %s'%(tactileVals)
		return tactileVals
	
		
	def new_temp_mail(self, fingers_to_change):
		former_mailbox_c={}
		for finger in fingers_to_change:
			former_mailbox_c[finger]=self.get_property(finger,GRPC)
			self.set_property(finger,GRPC,12)
		return former_mailbox_c

	def revert_temp_mail(self, fingers_to_change,former):
		for finger in fingers_to_change:
			former_mailbox_value=former[finger]
			self.set_property(finger,GRPC,former_mailbox_value)
	#==========================ANGLE_CONVERSIONS=========================

	def enc_to_per(self, enc):
		'''
		Given an angle in encoder counts, return the percentage of the angle that represents.

			@param enc: Encoder counts.
			@type enc: int
			@return: Percentage
			@rtype: float
		'''
		per = enc/1950.0
		return round(per, 2)

	def enc_to_rad(self, enc, type = BASE_TYPE):
		'''
		Given an angle in encoder counts, return the radian measure of the angle that represents.

			@param enc: Encoder counts.
			@type enc: int
			@return: Radians
			@rtype: float
		'''
		motion_limit = BASE_LIMIT
		tics = MAX_ENCODER_TICKS
		
		if type == TIP_TYPE:
			motion_limit = TIP_LIMIT
			tics = MAX_FINGERTIP_TICKS
		elif type == SPREAD_TYPE:
			motion_limit = SPREAD_LIMIT
			tics = MAX_SPREAD_TICKS
				
		PI = 3.141592653589
		rad = enc * (motion_limit*PI/180)/tics
		return round(rad,2)

	def enc_to_deg(self, enc):
		'''
		Given an angle in encoder counts, return the degree measure of the angle that represents.

			@param enc: Encoder counts.
			@type enc: int
			@return: Degrees
			@rtype: float
		'''
		deg = enc * 140/MAX_ENCODER_TICKS
		return round(deg,2)

	def per_to_enc(self, per):
		'''
		Given a percentage of an angle, return it in encoder counts.

			@param per: Percentage
			@type per: float
			@return: Encoder counts
			@rtype: int
		'''
		enc = per * 1950.0
		return int(enc)

	def rad_to_enc(self, rad, type = BASE_TYPE):
		'''
		Given the readian measure of an angle, return it in encoder counts.

			@param rad: Radians
			@type rad: float
			@return: Encoder counts
			@rtype: int
		'''
		motion_limit = BASE_LIMIT
		tics = MAX_ENCODER_TICKS
		
		if type == TIP_TYPE:
			motion_limit = TIP_LIMIT
			tics = MAX_FINGERTIP_TICKS
		elif type == SPREAD_TYPE:
			motion_limit = SPREAD_LIMIT
			tics = MAX_SPREAD_TICKS
			
		PI = 3.141592653589
		enc = rad / ((motion_limit*PI/180)/tics)
		return int(enc)

	def deg_to_enc(self, deg):

		'''
		Given a degree measure of an angle, return it in encoder counts.

			@param deg: Degrees
			@type deg: float
			@return: Encoder counts
			@rtype: int
		'''
		enc = deg * 195000.0/140
		return int(enc)
	
	def clean_read_buffer(self):
		
		result_ = self.read_msg()
			
		while result_[0] == 0:
			result_ = self.read_msg()
		
