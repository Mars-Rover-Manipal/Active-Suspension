#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from math import radians
import time



def calculate():
    angle = radians(0)
    fl_pub.publish(angle)
    fr_pub.publish(angle)
    bl_pub.publish(angle)
    br_pub.publish(angle)

def listener():

    start_time = time.time()

    while not rospy.is_shutdown():
        end_time = time.time()

        if abs(start_time - end_time) > 3:
            break

        calculate()
        rospy.sleep(0.01)

if __name__ == '__main__':
    try:

        fl_pub = rospy.Publisher('/lsd/fl_joint_position_controller/command', Float64, queue_size=10)
        fr_pub = rospy.Publisher('/lsd/fr_joint_position_controller/command', Float64, queue_size=10)
        bl_pub = rospy.Publisher('/lsd/bl_joint_position_controller/command', Float64, queue_size=10)
        br_pub = rospy.Publisher('/lsd/br_joint_position_controller/command', Float64, queue_size=10)
        rospy.init_node('reset_pose_node', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
