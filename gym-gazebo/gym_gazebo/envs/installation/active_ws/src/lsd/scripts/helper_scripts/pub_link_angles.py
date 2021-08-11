#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetLinkState, SetLinkState
from tf.transformations import quaternion_from_euler
from math import radians


def link_service(fl_motor_angle, fr_motor_angle, bl_motor_angle, br_motor_angle):
    rospy.wait_for_service('/gazebo/get_link_state')
    rospy.wait_for_service('/gazebo/set_link_state')

    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

        ob1 = get_link_state("lsd::FCL_l", "")
        ob2 = get_link_state("lsd::FCL_r", "")
        ob3 = get_link_state("lsd::RCL_l", "")
        ob4 = get_link_state("lsd::RCL_r", "")

        fl_motor = ob1.link_state
        fr_motor = ob2.link_state
        bl_motor = ob3.link_state
        br_motor = ob4.link_state

        fl_motor.reference_frame = ""

        # Front Left Motor

        fl_orientation_list = quaternion_from_euler(radians(90), radians(fl_motor_angle), 0)

        fl_motor.pose.orientation.x = fl_orientation_list[0]
        fl_motor.pose.orientation.y = fl_orientation_list[1]
        fl_motor.pose.orientation.z = fl_orientation_list[2]
        fl_motor.pose.orientation.w = fl_orientation_list[3]

        # Front Right Motor

        fr_orientation_list = quaternion_from_euler(radians(90), radians(fr_motor_angle), 0)

        fr_motor.pose.orientation.x = fr_orientation_list[0]
        fr_motor.pose.orientation.y = fr_orientation_list[1]
        fr_motor.pose.orientation.z = fr_orientation_list[2]
        fr_motor.pose.orientation.w = fr_orientation_list[3]

        # Rear Left Motor

        bl_orientation_list = quaternion_from_euler(radians(90), radians(bl_motor_angle), 0)

        bl_motor.pose.orientation.x = bl_orientation_list[0]
        bl_motor.pose.orientation.y = bl_orientation_list[1]
        bl_motor.pose.orientation.z = bl_orientation_list[2]
        bl_motor.pose.orientation.w = bl_orientation_list[3]

        # Rear Right Motor

        br_orientation_list = quaternion_from_euler(radians(90), radians(br_motor_angle), 0)

        br_motor.pose.orientation.x = br_orientation_list[0]
        br_motor.pose.orientation.y = br_orientation_list[1]
        br_motor.pose.orientation.z = br_orientation_list[2]
        br_motor.pose.orientation.w = br_orientation_list[3]


        set_link_state(fl_motor)
        set_link_state(fr_motor)
        #set_link_state(bl_motor)
        #set_link_state(br_motor)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def calculate():
    link_service(7,7,13,13)



def listener():
    while not rospy.is_shutdown():
        calculate()
        rate.sleep()


if __name__ == '__main__':
    try:

        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(200)

        listener()

    except rospy.ROSInterruptException:
        pass
