#############################################################################################################################################################################################################################################################
#!/usr/bin/env python
from math import degrees, inf, radians
import rospy
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, WrenchStamped
from gazebo_msgs.msg import LinkStates
from std_srvs.srv import Empty
from gym import utils, spaces
from tf.transformations import euler_from_quaternion
from gym_gazebo.envs import gazebo_env
from sensor_msgs.msg import Imu, LaserScan
#############################################################################################################################################################################################################################################################
class LsdEnv(gazebo_env.GazeboEnv):
    def __init__(self):
        gazebo_env.GazeboEnv.__init__(self, "custom_world.launch")
        self.force_fl=0
        self.force_fr=0
        self.force_ml=0
        self.force_mr=0
        self.force_rl=0
        self.force_rr=0
        self.pitch=0
        self.roll=0
        self.yaw=0
        self.reward=0
        self.range_data_0=[]
        self.range_data_15=[]
        self.range_data_345=[]
        self.observation_space=spaces.Box(-inf, inf, shape=(9,1), dtype = np.float32)
        self.orientation_list=[]
        self.action_space= spaces.Box(-60, 60, shape=(4,1), dtype = np.float32)
        self.obstacle_distance=0
        self.chassis_angle = 0
        self.done=False
        self.vel=Twist()
        rospy.Subscriber("/fl_wheel_ft_sensor", WrenchStamped, self.callback_fl)
        rospy.Subscriber("/fr_wheel_ft_sensor", WrenchStamped, self.callback_fr)
        rospy.Subscriber("/ml_wheel_ft_sensor", WrenchStamped, self.callback_ml)
        rospy.Subscriber("/mr_wheel_ft_sensor", WrenchStamped, self.callback_mr)
        rospy.Subscriber("/rl_wheel_ft_sensor", WrenchStamped, self.callback_rl)
        rospy.Subscriber("/rr_wheel_ft_sensor", WrenchStamped, self.callback_rr)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback_link)

        rospy.Subscriber("/imu", Imu, self.callback_imu)

        rospy.Subscriber("/scan", LaserScan, self.callback_scan)

        self.velocity_publisher=rospy.Publisher("/cmd_vel",Twist, queue_size=10, latch=True)

        self.joint_1_publisher=rospy.Publisher("/lsd/joint1_position_controller/command",
                                Float64,queue_size=10)
        self.joint_2_publisher=rospy.Publisher("/lsd/joint2_position_controller/command",
                                Float64,queue_size=10)
        self.joint_3_publisher=rospy.Publisher("/lsd/joint3_position_controller/command",
                                Float64,queue_size=10)
        self.joint_4_publisher=rospy.Publisher("/lsd/joint4_position_controller/command",
                                Float64,queue_size=10)
        self.pause = rospy.ServiceProxy("/gazebo/pause", Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def callback_fl(self, msg):
        self.force_fl=msg.wrench.force
    def callback_fr(self, msg):
        self.force_fr=msg.wrench.force
    def callback_ml(self, msg):
        self.force_ml=msg.wrench.force
    def callback_mr(self, msg):
        self.force_mr=msg.wrench.force
    def callback_rl(self, msg):
        self.force_rl=msg.wrench.force
    def callback_rr(self, msg):
        self.force_rr=msg.wrench.force
    def callback_imu(self, msg):

        self.orientation_list=[msg.orientation.x, msg.orientation.y, msg.orientation.z, 
                                msg.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(self.orientation_list)
        self.pitch=degrees(self.pitch)
        self.roll=degrees(self.roll)
        self.yaw=degrees(self.yaw)	

    def callback_scan(self, msg):
        
        self.range_data_0= msg.ranges[0]
        self.range_data_15= msg.ranges[15]
        self.range_data_345= msg.ranges[345]
        self.obstacle_distance= min(self.range_data_0,self.range_data_15, self.range_data_345)		
    
    def callback_link(self, msg):
        links = msg.name
        pose_link = msg.pose
        ch_link_idx = links.index("lsd::CH")
        ch_orientation_list = [pose_link[ch_link_idx].orientation.x,
                                pose_link[ch_link_idx].orientation.y,
                                pose_link[ch_link_idx].orientation.z,
                                pose_link[ch_link_idx].orientation.w]
        (ch_roll, ch_pitch, ch_yaw) = euler_from_quaternion(ch_orientation_list)
        self.chassis_angle= ch_pitch

    def get_observation(self):
        self.observation_space =np.array([self.force_fl, self.force_fr, self.force_ml, 
                                self.force_mr, self.force_rl, self.force_rr, 
                                self.pitch, self.roll, self.obstacle_distance])
        return self.observation_space

    def step(self, action):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        self.joint_1_publisher.publish(radians(action[0]))
        self.joint_2_publisher.publish(radians(action[1]))
        self.joint_3_publisher.publish(radians(action[2]))
        self.joint_4_publisher.publish(radians(action[3]))
        #publish till the action taken is completed
        self.done=False
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
        observation_= self.observation_space
        #condition to check if the episode is complete
        self.get_reward()
        #compute reward due to the action taken
        state_=(observation_, self.reward, self.done, {})
        return state_

    def get_reward(self):
        threshold = (-0.5236, 0.5236)
        if self.chassis_angle<threshold[0] and self.chassis_angle>threshold[1]:
            self.reward+=1
        else:
            self.reward-=15
            self.done = True
        #force thresholds to be added
        
    def reset(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")
        #unpause simulation to make an observation and reset the values
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
        self.reward=0
        initial_reading=self.get_observation()
        self.vel.linear.x=2
        self.vel.linear.y=0
        self.vel.linear.z=0
        self.vel.angular.x=0
        self.vel.angular.y=0
        
        self.velocity_publisher.publish(self.vel)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        return initial_reading