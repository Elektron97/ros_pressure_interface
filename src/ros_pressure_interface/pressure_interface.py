### Pressure Interface Class ###

import rospy
from std_msgs.msg import Float32MultiArray

import numpy as np
import serial
import struct

## Global Variables
DEFAULT_CHAMBERS	= 6		# Default number of chambers
SLEEP_TIME 			= 1 	# Sleep Time for the deconstructor - in seconds
MAX_DIGIT			= 255   # Max value of pressure in digit 
MIN_DIGIT			= 10   	# Min value of pressure in digit 




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
  
		rospy.sleep(SLEEP_TIME) # Sleeps for SLEEP_TIME - in seconds

  
		if self.arduino:
			self.arduino.close()
    
		if self.sub_obj:
			self.sub_obj.unregister()

		print("Object destroyed succesfully! ")
  
  
  
	def bar_to_digit(pressures):
     
	#####################################################################
	# 																	#
	#			Function to convert the pressure from bar to digit:		#
	#																	#
	#	p_digit = (max_digit - min_digit) * (p / max_bar) + min_digit	#
	#																	#
	#				p_digit 	= pressure value in digit				#
	#				p 			= pressure value in bar					#
	# 				max_digit 	= max value of pressure in digit		#
	# 				min_digit	= min value of pressure in digit		#
	#				max_bar		= max value of pressure in bar			#
	#				int() = function to convert from double to int  	#
	#																	#
	#####################################################################
 
		# Initialization
		pressures_digit = [0] * 9

		# Conversion
		pressures_digit[0] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[0] / 1.68) + MIN_DIGIT)
		pressures_digit[1] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[1] / 1.68) + MIN_DIGIT)
		pressures_digit[2] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[2] / 1.68) + MIN_DIGIT)
  
		pressures_digit[3] = int((MAX_DIGIT - 15) * (pressures[3] / 1.08) + 15)
  
		pressures_digit[4] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[4] / 1.68) + MIN_DIGIT)
		pressures_digit[5] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[5] / 1.68) + MIN_DIGIT)

		pressures_digit[6] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[6] / 4.4) + MIN_DIGIT)
		pressures_digit[7] = int((MAX_DIGIT - MIN_DIGIT) * (pressures[7] / 4.4) + MIN_DIGIT)

		pressures_digit[8] = int((4095 - 50) * (pressures[8] / 1.02) + 50)
		


