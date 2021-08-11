#!/usr/bin/env python
import os
import time
import numpy as np
from setup import env
import rospy

ob1=env()
############################################################################################################################################################################
def main():
	
	rospy.init_node('Communication', anonymous=True, disable_signals=True)
	rate = rospy.Rate(50)
	
	while not rospy.is_shutdown():
		
		for i in range(5):

			initial_reading=ob1.reset()
			
			print(initial_reading)
			angles=np.array([-40.0,40.0,0.0,20.0], dtype="float64")
			ob1.step(angles)
			print("step taken")

############################################################################################################################################################################
if __name__=='__main__':
	main()		


