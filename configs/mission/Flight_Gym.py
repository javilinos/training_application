#!/usr/bin/env python3

import rospy
from std_srvs.srv import *
from std_msgs.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from aerostack_msgs.msg import *
from aerostack_msgs.srv import SetControlModeResponse
from aerostack_msgs.srv import SetControlMode
from aerostack_msgs.srv import SetControlModeRequest
from trajectory_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler, \
                               quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import numpy as np
import math
import gym
from gym import spaces

GROUND_SPEED = 1
POSE = 2

class Environment(gym.Env):
    def __init__(self):
        rospy.init_node('gym')

        self.pose_agent = PoseStamped()       
        self.pose_point = PoseStamped()

        self.speed_reference = TwistStamped()
        self.pose_reference = PoseStamped()

        self.state = PoseStamped()

        self.set_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.speed_reference_pub = rospy.Publisher('/drone1/motion_reference/speed', TwistStamped, queue_size=1)
        self.pose_reference_pub = rospy.Publisher('/drone1/motion_reference/pose', PoseStamped, queue_size=1)
        self.reset_flightmare = rospy.Publisher('/environment/flightmare/reset', std_msgs.msg.Empty, queue_size=1)

        self.agent_pose_sub = rospy.Subscriber('/drone1/self_localization/pose', PoseStamped, self.AgentPoseCallback)
        self.point_pose_sub = rospy.Subscriber('/flightmare/gate_position', PoseArray, self.PointPoseCallback)
        
        self.reward = 0.0
        self.distance_reward = 0.0
        self.initial_distance = 0.0
        self.r = rospy.Rate(100.0)
        self.n = 0

        self.pose_received = False
        
        self.action_space = spaces.Box(np.array((-1.0, -1.0, -1.0)), np.array((1.0, 1.0, 1.0)), dtype=np.float32) # Espacio de acción continuo vector 1x4 con valores -> [-1, 1]
        self.observation_space = spaces.Box(np.array((-1.0, -1.0, -1.0)), np.array((1.0, 1.0, 1.0)), dtype=np.float32)
        rospy.spin()

    def step (self, action): # Esta acción tiene la forma de np.array(float, float, float)
        done = False
        goal_reward = 0.0
        self.speed_reference.twist.linear.x = action[0]
        self.speed_reference.twist.linear.y = action[1]
        self.speed_reference.twist.linear.z = action[2]
        self.PublishSpeedReferences()
        rospy.wait_for_message("/drone1/self_localization/pose", PoseStamped)
        if (round(self.state.pose.position.x) == 0 and round(self.state.pose.position.y) == 0 and round(self.state.pose.position.z) == 0):
            done = True
            goal_reward = 200.0

        if (self.pose_agent.pose.position.z < 0.3):
            done = True
            goal_reward = -100.0

        elif (self.pose_agent.pose.position.z > 4.0):
            done = True
            goal_reward = -100.0

        actual_distance = self.CalcDistance([self.pose_point.pose.position.x, self.pose_point.pose.position.y, self.pose_point.pose.position.z],
             [self.pose_agent.pose.position.x, self.pose_agent.pose.position.y, self.pose_agent.pose.position.z])
        distance_reward = np.clip(actual_distance/self.initial_distance, -1.0, 0.0)
        print (distance_reward)
        reward = distance_reward + goal_reward
        return np.array([self.state.pose.position.x, self.state.pose.position.y, self.state.pose.position.z]).astype(np.float32), reward, done, {}

    def reset(self): 
        x = 0.0
        y = 0.0
        z = 2.0
        qx = 0.0
        qy = 0.0
        qz = 0.0
        qw = 0.0
        """rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_physics()"""
        self.pose_received = False 
        ctrl_c = False
        self.setControlMode(MotionControlMode.SPEED)
        self.ClearSpeedReferences()
        self.PublishSpeedReferences()
        self.setControlMode(MotionControlMode.GROUND_SPEED)

        self.pose_reference.pose.position.x = x
        self.pose_reference.pose.position.y = y
        self.pose_reference.pose.position.z = z


        ctrl_c = False

        while not ctrl_c:
            connections = self.pose_reference_pub.get_num_connections()
                
            if (connections > 0):
                pose = ModelState()
                pose.model_name = 'iris'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                pose.reference_frame = 'map'

                ctrl_c = False
                while not ctrl_c:
                    connections = self.set_pose.get_num_connections()
                    
                    if (connections > 0):
                        self.set_pose.publish(pose)
                        ctrl_c = True
                #position = PoseStamped()
                rospy.wait_for_message("/drone1/self_localization/pose", PoseStamped)
                rospy.wait_for_message("/drone1/self_localization/speed", TwistStamped)
                
                self.pose_reference_pub.publish(self.pose_reference)
                ctrl_c = True

        ctrl_c = False
        
        self.reset_flightmare.publish(std_msgs.msg.Empty())
        rospy.wait_for_message("/drone1/self_localization/pose", PoseStamped)

        return np.array([self.state.pose.position.x, self.state.pose.position.y, self.state.pose.position.z]).astype(np.float32)
    

    def PoseStamped_2_mat(self, p):
        q = p.pose.orientation
        pos = p.pose.position
        T = quaternion_matrix([q.x,q.y,q.z,q.w])
        T[:3,3] = np.array([pos.x,pos.y,pos.z])
        return T

    def Mat_2_posestamped(self, m,f_id="map"):
        q = quaternion_from_matrix(m)
        p = PoseStamped(header = Header(frame_id=f_id), #robot.get_planning_frame()
                        pose=Pose(position=Point(*m[:3,3]), 
                        orientation=Quaternion(*q)))
        return p

    def T_inv(self, T_in):
        R_in = T_in[:3,:3]
        t_in = T_in[:3,[-1]]
        R_out = R_in.T
        t_out = -np.matmul(R_out,t_in)
        return np.vstack((np.hstack((R_out,t_out)),np.array([0, 0, 0, 1])))

    def euler_from_quaternion(self, x, y, z, w):

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    def AgentPoseCallback(self, data):
        self.pose_agent = data

        if (self.pose_received):
            Tw1 = self.PoseStamped_2_mat(self.pose_agent)
            Tw2 = self.PoseStamped_2_mat(self.pose_point)
            T1w = self.T_inv(Tw1)
            T12 = np.matmul(T1w, Tw2)
            
            self.state = self.Mat_2_posestamped(T12, f_id="map")
            self.state.pose.position.x = np.clip(self.state.pose.position.x/10, -1, 1)
            self.state.pose.position.y = np.clip(self.state.pose.position.y/10, -1, 1)
            self.state.pose.position.z = np.clip(self.state.pose.position.z/10, -1, 1)
            #print("distance reward:")
            #print (self.distance_reward)


    def PointPoseCallback(self, data):
        self.pose_received = True
        for c, waypoint in enumerate(data.poses):
            self.pose_point.header.frame_id = "map"
            self.pose_point.pose = waypoint
            self.initial_distance = -self.CalcDistance([waypoint.position.x, waypoint.position.y, waypoint.position.z],
             [self.pose_agent.pose.position.x, self.pose_agent.pose.position.y, self.pose_agent.pose.position.z])

    def CalcDistance(self, p1, p2):
        return math.dist(p1, p2)
    
    def ClearSpeedReferences(self):
        self.speed_reference.twist.linear.x = 0.0
        self.speed_reference.twist.linear.y = 0.0
        self.speed_reference.twist.linear.z = 0.0
        self.speed_reference.twist.angular.x = 0.0
        self.speed_reference.twist.angular.y = 0.0
        self.speed_reference.twist.angular.z = 0.0


    def PublishSpeedReferences(self):
        ctrl_c = False
        while not ctrl_c:
            connections = self.speed_reference_pub.get_num_connections()
            if (connections > 0):
                self.speed_reference_pub.publish(self.speed_reference)
                ctrl_c = True

    def setControlMode(self, control_mode):
        rospy.wait_for_service('/drone1/set_control_mode')
        try:
            setControlModeClientSrv = rospy.ServiceProxy('/drone1/set_control_mode', SetControlMode)
            setControlModeSrv = SetControlModeRequest()
            setControlModeSrv.controlMode.mode = control_mode            
            response = setControlModeClientSrv(setControlModeSrv)
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

    def close(self):
        pass


if __name__ == '__main__':
    Environment()