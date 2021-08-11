from math import degrees, inf, radians
import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
from std_srvs.srv import Empty
from gym import utils, spaces
from tf.transformations import euler_from_quaternion
from gym_gazebo.envs import gazebo_env
from sensor_msgs.msg import Imu, LaserScan
import rospkg
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import time
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped
from random import randint
import torch
from gazebo_msgs.msg import ModelStates
rospack = rospkg.RosPack()


class LsdEnv(gazebo_env.GazeboEnv):
    def __init__(self):
        gazebo_env.GazeboEnv.__init__(self, "custom_world.launch")

        self.pitch = 0
        self.chassis_rise = 0.0
        self.counter=0
        self.roll = 0
        self.yaw = 0
        self.centroid = 0
        self.y_displacement = 0
        self.x_displacement = 0
        self.ground_clearance = 0
        self.reward = 0
        self.observation_space = spaces.Box(low=-50, high=50, shape=(4,), dtype=np.float32)
        self.orientation_list = []
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
        self.obstacle_distance = 0
        self.obstacle_height = 0
        self.step_height=0
        self.obstacle_offset = 0
        self.chassis_angle = 0
        self.actual_speed = 0
        self.done = False
        self.package_path = rospack.get_path('lsd')

        rospy.Subscriber("/imu", Imu, self.callback_imu)

        rospy.Subscriber("/odom", Odometry, self.callback_pose)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback_chassis_rise)

        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.fl_joint_pub = rospy.Publisher("/lsd/fl_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.fr_joint_pub = rospy.Publisher("/lsd/fr_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.bl_joint_pub = rospy.Publisher("/lsd/bl_joint_position_controller/command",
                                                 Float64, queue_size=10)
        self.br_joint_pub = rospy.Publisher("/lsd/br_joint_position_controller/command",
                                                 Float64, queue_size=10)

        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        rospy.Subscriber("/centroid_point", PoseStamped, self.callback_point)
        self.rate = rospy.Rate(50)

    def forward(self):

        vel_cmd = Twist()
        vel_cmd.linear.x = -0.7
        vel_cmd.linear.y = 0
        vel_cmd.angular.z = 0

        self.velocity_publisher.publish(vel_cmd)

    def teleport(self):

        state_msg = ModelState()
        state_msg.model_name = 'lsd'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0.5
        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 1

        step = ModelState()
        self.step_height = randint(25, 32)
        print("\033[0;32m\nOBSTACLE HEIGHT = %scm\033[0m" % self.step_height)

        step.model_name = 'step1'
        step.pose.position.x = 5
        step.pose.position.y = 0
        step.pose.position.z = (float(self.step_height) / 100.) - 0.16
        step.pose.orientation.x = 0
        step.pose.orientation.y = 0
        step.pose.orientation.z = 0
        step.pose.orientation.w = 1

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            set_state(state_msg)
            set_state(step)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def callback_imu(self, msg):

        self.orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z,
                                 msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        self.pitch = degrees(self.pitch)
        self.roll = degrees(self.roll)
        self.yaw = degrees(self.yaw)
        self.counter+=1

    def callback_chassis_rise(self, msg):
        try:
            self.chassis_rise = msg.pose[msg.name.index('lsd')].position.z
            # print(self.chassis_rise)
    
        except ValueError:
            pass

    def callback_point(self, msg):
        self.centroid = msg

    def callback_pose(self, msg):
        self.actual_speed = msg.twist.twist.linear.x
        self.y_displacement = msg.pose.pose.position.y
        self.x_displacement = msg.pose.pose.position.x

    def descretize_func(self, tu):
        for i in range(len(tu)):
            tu[i] = round(tu[i], 1)

        return tu

    def get_observation(self):

        observation = [self.pitch, self.roll, self.x_displacement, self.step_height]
        return observation

    def step(self, action):

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except rospy.ServiceException:
            print("/gazebo/unpause_physics service call failed")

        self.forward()

        if 2.9< self.x_displacement <3.3:


            action[2]=action[2]*37
            action[3]=action[3]*37

            self.fl_joint_pub.publish(radians(0))
            self.fr_joint_pub.publish(radians(0))
            self.bl_joint_pub.publish(radians(abs(action[2])))
            self.br_joint_pub.publish(radians(abs(action[3])))

            time.sleep(0.5)

            time.sleep(5)

            action[0] = -abs(action[0]*37)
            action[1] = -abs(action[1]*37)

            self.fl_joint_pub.publish(radians(0))
            self.fr_joint_pub.publish(radians(0))
            self.bl_joint_pub.publish(radians(action[0]))
            self.br_joint_pub.publish(radians(action[1]))

            time.sleep(0.5)

            time.sleep(3)

            self.fl_joint_pub.publish(radians(10))
            self.fr_joint_pub.publish(radians(10))
            self.bl_joint_pub.publish(radians(0))
            self.br_joint_pub.publish(radians(0))

            time.sleep(2)

            self.fl_joint_pub.publish(radians(0))
            self.fr_joint_pub.publish(radians(0))
            self.bl_joint_pub.publish(radians(0))
            self.br_joint_pub.publish(radians(0))

            time.sleep(1)


        observation_ = self.get_observation()

        self.get_reward()
        # print(np.array(observation_,dtype=np.float32), self.reward, self.done)
        return np.array(observation_), self.reward, self.done, {}

    def get_reward(self):
        if abs(self.pitch) > 20:
            self.done = True
            self.reward = -100
        if abs(self.yaw)>10:
            self.reward=-100
        if abs(self.x_displacement)>3.6:
            self.reward= 100
            self.done=True
        if(self.counter>430 and self.x_displacement<3.3):
            self.reward=-50
            self.done=True
        if(self.counter>430):
            self.done=True



    def reset(self):

        self.done = False
        self.reward=0
        self.counter=0
        self.teleport()
        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0

        self.velocity_publisher.publish(vel_cmd)

        self.fl_joint_pub.publish(0)
        self.fr_joint_pub.publish(0)
        self.bl_joint_pub.publish(0)
        self.br_joint_pub.publish(0)

        time.sleep(2)

        # unpause simulation to make an observation and reset the values
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            # resp_pause = pause.call()
            self.unpause()
        except rospy.ServiceException:
            print("/gazebo/unpause_physics service call failed")

        initial_reading = self.get_observation()

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except rospy.ServiceException:
            print("/gazebo/pause_physics service call failed")

        return np.array(initial_reading, dtype=np.float32)