#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion
from math import degrees

# Initialization
fl_motor_pitch = 0
fr_motor_pitch = 0
bl_motor_pitch = 0
br_motor_pitch = 0


def callback_links(msg):
    global fl_motor_pitch, fr_motor_pitch, bl_motor_pitch, br_motor_pitch

    links = msg.name
    pose_link = msg.pose

    fl_motor_idx = links.index("lsd::FCL_l")
    fr_motor_idx = links.index("lsd::FCL_r")
    bl_motor_idx = links.index("lsd::RCL_l")
    br_motor_idx = links.index("lsd::RCL_r")

    fl_orientation_list = [pose_link[fl_motor_idx].orientation.x, pose_link[fl_motor_idx].orientation.y,
                        pose_link[fl_motor_idx].orientation.z, pose_link[fl_motor_idx].orientation.w]
    (fl_motor_roll, fl_motor_pitch, fl_motor_yaw) = euler_from_quaternion(fl_orientation_list)
    fl_motor_pitch = int(degrees(fl_motor_pitch))

    fr_orientation_list = [pose_link[fr_motor_idx].orientation.x, pose_link[fr_motor_idx].orientation.y,
                        pose_link[fr_motor_idx].orientation.z, pose_link[fr_motor_idx].orientation.w]
    (fr_motor_roll, fr_motor_pitch, fr_motor_yaw) = euler_from_quaternion(fr_orientation_list)
    fr_motor_pitch = int(degrees(fr_motor_pitch))

    bl_orientation_list = [pose_link[bl_motor_idx].orientation.x, pose_link[bl_motor_idx].orientation.y,
                           pose_link[bl_motor_idx].orientation.z, pose_link[bl_motor_idx].orientation.w]
    (bl_motor_roll, bl_motor_pitch, bl_motor_yaw) = euler_from_quaternion(bl_orientation_list)
    bl_motor_pitch = int(degrees(bl_motor_pitch))

    br_orientation_list = [pose_link[br_motor_idx].orientation.x, pose_link[br_motor_idx].orientation.y,
                           pose_link[br_motor_idx].orientation.z, pose_link[br_motor_idx].orientation.w]
    (br_motor_roll, br_motor_pitch, br_motor_yaw) = euler_from_quaternion(br_orientation_list)
    br_motor_pitch = int(degrees(br_motor_pitch))


def calculate():
    global fl_motor_pitch, fr_motor_pitch, bl_motor_pitch, br_motor_pitch

    print(fl_motor_pitch, fr_motor_pitch, bl_motor_pitch, br_motor_pitch)


def listener():
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback_links)

    while not rospy.is_shutdown():
        calculate()
        rospy.sleep(0.01)


if __name__ == '__main__':
    try:

        rospy.init_node('Communication', anonymous=True, disable_signals=True)
        rate = rospy.Rate(50)

        listener()

    except rospy.ROSInterruptException:
        pass
