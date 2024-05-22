#!/usr/bin/env python
import rospy
from ros_pressure_interface.pressure_interface import Pressure_Interface

def main():
	rospy.init_node("control_box_interface", anonymous=True)
	Pressure_Interface(6)
	rospy.spin()

if __name__ == '__main__':
	main()
