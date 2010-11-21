#!/usr/bin/env python
'''
Created on November 20, 2010

@author: Dr. Rainer Hessmer
'''

import roslib; roslib.load_manifest('playground')
import rospy
from std_msgs.msg import String
from playground.msg import Encoder

from SerialDataGateway import SerialDataGateway

class Arduino(object):
	'''
	Helper class for communicating over serial port with an Arduino board
	'''

	def _HandleReceivedLine(self,  line):
		rospy.logdebug(line)
		self._Publisher.publish(String(line))

	def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		self._Publisher = rospy.Publisher('serial', String)
		rospy.init_node('arduino')

		self._SerialDataGateway = SerialDataGateway(port, baudrate,  self._HandleReceivedLine)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()

if __name__ == '__main__':
	arduino = Arduino("/dev/ttyUSB0", 115200)
	try:
		arduino.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		arduino.Stop()

