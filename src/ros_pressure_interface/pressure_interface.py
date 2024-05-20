### Pressure Interface Class ###

import rospy
from std_msgs.msg import Float32MultiArray

import numpy as np
import serial
import struct

## Global Variables
DEFAULT_CHAMBERS	= 6								# Default number of chambers
PMAX = rospy.get_param('hardware_params/pmax')

# Serial Communication
BAUDRATE = rospy.get_param('serial_params/baudrate')
TIMEOUT = rospy.get_param('serial_params/timeout')
PORT = rospy.get_param('serial_params/port')
SYNCBYTE = rospy.get_param('serial_params/syncbyte')

# Topic Names
topic_name = '/pressures'

class ChamberException(Exception):
	pass

class Pressure_Interface(object):
    
	def __init__(self, n_chambers = DEFAULT_CHAMBERS):
     
		# Arduino Obj
		self.arduino = self.set_communication()
		
		# Parameters of the class
		self.n_chambers = n_chambers

		# Define Pressure Array
		self.pressures = [PMAX]*self.n_chambers
		# Put to 0 every chambers
		self.write_pressure(self.pressures)

		# Define Pub/Sub objects
		self.sub_obj = rospy.Subscriber(topic_name, Float32MultiArray, self.pressure_callback)

	def set_communication(self):
		try:
			arduino = serial.Serial( # set parameters, in fact use your own :-)
				port=PORT,
				baudrate=BAUDRATE,
				timeout=TIMEOUT
			)
			arduino.isOpen() # try to open port, if possible print message
			print ("port is opened!")
		except IOError: # if port is already opened, close it and open it again and print message
			arduino.close()
			arduino.open()
			print ("port was already open, was closed and opened again!")
		return arduino
	
	def write_pressure(self, pressures):
		#########################################################
		# We pass "pressures" to avoid changes					#
		# during the execution of the methods,					#
		# due to the callback or other types of interruptions.	#
		#########################################################

		# Safe Saturation
		for chamber in pressures:
			if chamber > PMAX:
				chamber = PMAX
		
		# Add syncbyte & create packet
		packet = np.array([SYNCBYTE] + pressures, dtype = np.uint8)

		if self.arduino.isOpen():
			for value in packet: # Sending Data
				s = struct.pack('!{0}B'.format(len(packet)), *packet)
				self.arduino.write(s)
	
	def pressure_callback(self, msg):
		# Log
		rospy.loginfo("Writing in the Arduino the commanded pressures...")
  
  		# Extract Data
		try:
			if not self.n_chambers == len(msg.data):
				raise ChamberException
			else:
				self.pressures = msg.data
		except ChamberException:
			rospy.logerr("The length of the message ({}) is not consinstent with the number of chambers ({}).".format(len(msg.data), self.n_chambers))

		# Send to Arduino
		self.write_pressure(self.pressures)

	def __del__(self):
     
		# Set to 0 Pressure Array
		self.pressures = [0.0]*self.n_chambers
		# Put to 0 every chambers
		self.write_pressure(self.pressures)
  
		if self.arduino:
			self.arduino.close()
    
		if self.sub_obj:
			self.sub_obj.unregister()

		print("Object destroyed succesfully! ")
		


