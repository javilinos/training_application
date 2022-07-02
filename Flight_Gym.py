#!/usr/bin/env python3

from threading import local
from xml.etree.ElementTree import PI
import rospy
from std_srvs.srv import *
from std_srvs.srv import Empty as Emptysrv
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
from pyquaternion import Quaternion as Quat
import random

from stable_baselines3.common.vec_env.base_vec_env import VecEnv, VecEnvIndices, VecEnvObs, VecEnvStepReturn
from stable_baselines3.common.vec_env.util import copy_obs_dict, dict_to_obs, obs_space_info
from collections import OrderedDict
from copy import deepcopy
from typing import Any, Callable, List, Optional, Sequence, Type, Union
import time

GROUND_SPEED = 1
POSE = 2
class Environment(VecEnv):
    def __init__(self, num_envs: int):
        
        self.action_space = spaces.Box(np.array((0.0, -0.1, -0.5, -0.5)), np.array((0.5, 0.1, 0.5, 0.5)), dtype=np.float32) # Espacio de acción continuo vector 1x4 con valores -> [-1, 1]
        self.observation_space = spaces.Box(np.array((-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0)), np.array((1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)), dtype=np.float32)
        self.num_envs = num_envs
        #VecEnv.__init__(self, 1, self.observation_space, self.action_space)
        obs_space = self.observation_space
        self.keys, shapes, dtypes = obs_space_info(obs_space)

        self.buf_obs = OrderedDict([(k, np.zeros((self.num_envs,) + tuple(shapes[k]), dtype=dtypes[k])) for k in self.keys])
        self.buf_dones = np.zeros((self.num_envs,), dtype=bool)
        self.buf_rews = np.zeros((self.num_envs,), dtype=np.float32)
        self.buf_infos = [{} for _ in range(self.num_envs)]
        self.actions = None
        self.env_idx = -1

        self.set_pose = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)

        self.speed_references_buf_pub = []
        self.pose_references_buf_pub = []
        self.reset_flightmare_buf_pub = []
        self.hover_references_buf_pub = []

        self.agents_pose = []
        self.agents_speed = []
        self.points_pose = []
        self.n_steps_executed = []

        for env in range(num_envs):
            self.speed_references_buf_pub.append(rospy.Publisher("/drone" + str(env+1) +"/motion_reference/speed", JointTrajectoryPoint, queue_size=1))
            self.pose_references_buf_pub.append(rospy.Publisher("/drone" + str(env+1) +"/motion_reference/trajectory", JointTrajectoryPoint, queue_size=1))
            self.reset_flightmare_buf_pub.append(rospy.Publisher("/drone" + str(env+1) +"/environment/flightmare/reset", std_msgs.msg.Empty, queue_size=1))
            self.hover_references_buf_pub.append(rospy.Publisher("/drone" + str(env+1) +"/motion_reference/hover", JointTrajectoryPoint, queue_size=1))
            self.agents_pose.append(PoseStamped())
            self.agents_speed.append(TwistStamped())
            self.points_pose.append(PoseStamped())
            self.n_steps_executed.append(0)

        self.pose_reference = JointTrajectoryPoint()
        self.speed_reference = JointTrajectoryPoint()

        self.states = [PoseStamped(),PoseStamped(), PoseStamped(),PoseStamped(), TwistStamped(), TwistStamped(), TwistStamped(), TwistStamped()]
        #self.states = [PoseStamped(),TwistStamped()]
        self.reward = 0.0
        self.initial_distance = 0.0
        self.r = rospy.Rate(100.0)
        self.n = 0

        self.poses_received = [False, False, False, False]

        self.agent_pose_sub = rospy.Subscriber('/drone1/self_localization/pose', PoseStamped, self.AgentPoseCallback)
        self.agent_speed_sub = rospy.Subscriber('/drone1/self_localization/speed', TwistStamped, self.AgentSpeedCallback)
        self.point_pose_sub = rospy.Subscriber('/drone1/flightmare/gate_position', PoseArray, self.PointPoseCallback)

        self.agent_pose_sub2 = rospy.Subscriber('/drone2/self_localization/pose', PoseStamped, self.AgentPoseCallback2)
        self.agent_speed_sub2 = rospy.Subscriber('/drone2/self_localization/speed', TwistStamped, self.AgentSpeedCallback2)
        self.point_pose_sub2 = rospy.Subscriber('/drone2/flightmare/gate_position', PoseArray, self.PointPoseCallback2)
  
        self.agent_pose_sub3 = rospy.Subscriber('/drone3/self_localization/pose', PoseStamped, self.AgentPoseCallback3)
        self.agent_speed_sub3 = rospy.Subscriber('/drone3/self_localization/speed', TwistStamped, self.AgentSpeedCallback3)
        self.point_pose_sub3 = rospy.Subscriber('/drone3/flightmare/gate_position', PoseArray, self.PointPoseCallback3)

        self.agent_pose_sub4 = rospy.Subscriber('/drone4/self_localization/pose', PoseStamped, self.AgentPoseCallback4)
        self.agent_speed_sub4 = rospy.Subscriber('/drone4/self_localization/speed', TwistStamped, self.AgentSpeedCallback4)
        self.point_pose_sub4 = rospy.Subscriber('/drone4/flightmare/gate_position', PoseArray, self.PointPoseCallback4)

        self.step_counter = 0

    def take_step (self, action): # Esta acción tiene la forma de np.array(float, float, float))
        quat = Quat(x=0.0, y=0.0, z=self.agents_pose[self.env_idx].pose.orientation.z, w=self.agents_pose[self.env_idx].pose.orientation.w)

        action_prime = np.array([action[0], action[1], 0.0])

        #action_prime = action[0:3]
        action_prime = np.clip(quat.rotate(action_prime), -0.5, 0.5)

        self.speed_reference.positions = [0.0, 0.0, 0.0, 0.0]
        self.speed_reference.velocities = [action_prime[0], action_prime[1], action[2], action[3]]
        self.speed_reference.accelerations = [0.0, 0.0, 0.0, 0.0]
        self.speed_reference.effort = []

        self.PublishSpeedReferences()
        self.n_steps_executed[self.env_idx] += 1
    
 
    def calculate_reward(self):
        done = False
        goal_reward = 0.0
        distance_reward = 0.0
        
        local_state_pose = self.states[self.env_idx]
        local_state_speed = self.states[self.env_idx + self.num_envs]
        quat = Quat(x=0.0, y=0.0, z=self.agents_pose[self.env_idx].pose.orientation.z, w=self.agents_pose[self.env_idx].pose.orientation.w)
        sp_tmp = np.array([local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z])
        sp_tmp = np.clip(quat.rotate(sp_tmp), -1.0, 1.0)
        
        local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z = sp_tmp[0], sp_tmp[1], sp_tmp[2]
        local_state_angle = self.euler_from_quaternion(0.0,0.0,self.states[self.env_idx].pose.orientation.z,self.states[self.env_idx].pose.orientation.w)
        angle_distance = math.cos(local_state_angle)

        if (round(local_state_pose.pose.position.x, 1) == 0.0 and round(local_state_pose.pose.position.y, 1) == 0.0 and round(local_state_pose.pose.position.z, 1) == 0.0):
            done = True
            goal_reward = 40.0 * np.clip(angle_distance, 0.0, 1.0) * np.clip(local_state_speed.twist.linear.x, 0.0, 1.0)*2
            print("got big reward")
            print (goal_reward)
            self.n_steps_executed[self.env_idx] = 0   

        elif (local_state_pose.pose.position.z == 1.0):
            done = True
            goal_reward = -30.0
            print("got crash reward")
            self.n_steps_executed[self.env_idx] = 0   

        elif (local_state_pose.pose.position.z == -1.0):
            done = True
            goal_reward = -30.0
            print("got height reward")
            self.n_steps_executed[self.env_idx] = 0   

        elif (abs(local_state_pose.pose.position.x) >= 1 or abs(local_state_pose.pose.position.y) >= 1):
            done = True
            goal_reward = -30.0
            print("out_of_bounds reward")
            self.n_steps_executed[self.env_idx] = 0   

        if (self.n_steps_executed[self.env_idx]>=2048):
            done = True
            goal_reward = -20.0
            print("time limit reward")
            self.n_steps_executed[self.env_idx] = 0        
        
        
        #distance_reward = max(abs(local_state_speed.twist.linear.x), abs(local_state_speed.twist.linear.y))			

        actual_distance = -self.CalcDistance(np.array([self.points_pose[self.env_idx].pose.position.x, self.points_pose[self.env_idx].pose.position.y]),
            np.array([self.agents_pose[self.env_idx].pose.position.x, self.agents_pose[self.env_idx].pose.position.y]))

        height_distance = np.clip(-abs(self.states[self.env_idx].pose.position.z), -1.0, 0.0)
        """if (round (height_distance, 1) == 0.0):
            height_distance = 100
            print("good height reward")"""
            
        angle_to_point = math.cos(math.atan2(local_state_pose.pose.position.y, local_state_pose.pose.position.x))

        distance_reward = np.clip(actual_distance/10, -1.0, 0.0)*0.01 + height_distance*0.05 + angle_distance*0.01 + angle_to_point*0.02
        
        reward = distance_reward + goal_reward

        #local_state_angle = np.clip(local_state_angle/np.pi, -1.0, 1.0)
        
        return np.array([local_state_pose.pose.position.x, local_state_pose.pose.position.y, local_state_pose.pose.position.z, local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z, angle_distance, angle_to_point]).astype(np.float32), reward, done, {}

    def reset(self) -> VecEnvObs:
        for self.env_idx in range(self.num_envs):
            rng = random.uniform(0, 2*np.pi)
            q8c = Quat(axis=(0.0, 0.0, 1.0), radians=rng)
            x = 0.0
            y = 0.0
            z = 2.0
            qx = 0.0
            qy = 0.0
            qz = q8c.z
            qw = q8c.w
            """rospy.wait_for_service('/gazebo/pause_physics')
            self.pause_physics()"""
            self.poses_received[self.env_idx] = False 
            ctrl_c = False
            yaw = self.euler_from_quaternion(qx,qy,qz,qw)
            
            self.pose_reference.positions = [x, y, z, yaw]
            self.pose_reference.velocities = [0.0, 0.0, 0.0, 0.0]
            self.pose_reference.accelerations = [0.0, 0.0, 0.0, 0.0]
            self.pose_reference.effort = []
            
            rospy.wait_for_message("/drone"+ str(self.env_idx+1) + "/self_localization/pose", PoseStamped)

            ctrl_c = False

            while not ctrl_c:
                connections = self.hover_references_buf_pub[self.env_idx].get_num_connections()
                    
                if (connections > 1):
                    print("entra aqui2")
                    pose = ModelState()
                    pose.model_name = "iris_" + str(self.env_idx)
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
                            print("entra aqui3")
                            self.set_pose.publish(pose)
                            ctrl_c = True
                    #position = PoseStamped()
                    
                    self.hover_references_buf_pub[self.env_idx].publish(self.pose_reference)
                    ctrl_c = True

            ctrl_c = False
            
            while not ctrl_c:
                connections = self.reset_flightmare_buf_pub[self.env_idx].get_num_connections()
                
                if (connections > 0):            
                    self.reset_flightmare_buf_pub[self.env_idx].publish(std_msgs.msg.Empty())
                    ctrl_c = True
                    
            rospy.wait_for_message("/drone"+ str(self.env_idx+1) + "/self_localization/pose", PoseStamped)
            while (not self.poses_received[self.env_idx]):
                continue
            
            local_state_speed = self.states[self.env_idx + self.num_envs]
            quat = Quat(x=0.0, y=0.0, z=self.agents_pose[self.env_idx].pose.orientation.z, w=self.agents_pose[self.env_idx].pose.orientation.w)
            sp_tmp = np.array([local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z])
            sp_tmp = np.clip(quat.rotate(sp_tmp), -1.0, 1.0)
            local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z = sp_tmp[0], sp_tmp[1], sp_tmp[2]
            
            local_state_angle = self.euler_from_quaternion(0.0,0.0,self.states[self.env_idx].pose.orientation.z,self.states[self.env_idx].pose.orientation.w)
            angle_distance = math.cos(local_state_angle)
            angle_to_point = np.clip(math.atan2(self.states[self.env_idx].pose.position.x, self.states[self.env_idx].pose.position.y)/(np.pi/2), -1.0, 1.0)
            obs = np.array([self.states[self.env_idx].pose.position.x, self.states[self.env_idx].pose.position.y, self.states[self.env_idx].pose.position.z, local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z, angle_distance, angle_to_point]).astype(np.float32)
            self._save_obs(obs)
        
        return self._obs_from_buf()
        #return np.array([self.state_pose.pose.position.x, self.state_pose.pose.position.y, self.state_pose.pose.position.z]).astype(np.float32)

    def reset_single_env(self):
        
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
              unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Emptysrv)
              unpause_physics()
        except rospy.ServiceException as e:
              print ("Service call failed: %s"%e)
    
        rng = random.uniform(0, 2*np.pi)
        rng_h = random.uniform(1.3, 2.7)
        q8c = Quat(axis=(0.0, 0.0, 1.0), radians=rng)
        x = 0.0
        y = 0.0
        z = rng_h
        qx = 0.0
        qy = 0.0
        qz = q8c.z
        qw = q8c.w
        """rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_physics()"""
        self.poses_received[self.env_idx] = False 
        ctrl_c = False

        yaw = self.euler_from_quaternion(qx,qy,qz,qw)
        
        self.pose_reference.positions = [x, y, z, yaw]
        self.pose_reference.velocities = [0.0, 0.0, 0.0, 0.0]
        self.pose_reference.accelerations = [0.0, 0.0, 0.0, 0.0]
        self.pose_reference.effort = []

        rospy.wait_for_message("/drone"+ str(self.env_idx+1) + "/self_localization/pose", PoseStamped)


        ctrl_c = False

        while not ctrl_c:
            connections = self.hover_references_buf_pub[self.env_idx].get_num_connections()
                
            if (connections > 1):
                pose = ModelState()
                pose.model_name = "iris_" + str(self.env_idx)
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
                        
                    rospy.wait_for_message("/drone"+ str(self.env_idx+1) + "/self_localization/pose", PoseStamped)
                    rospy.wait_for_message("/drone"+ str(self.env_idx+1) + "/self_localization/speed", TwistStamped)
                #position = PoseStamped()
                
                self.hover_references_buf_pub[self.env_idx].publish(self.pose_reference)
                ctrl_c = True

        
        ctrl_c = False
        
        while not ctrl_c:
            connections = self.reset_flightmare_buf_pub[self.env_idx].get_num_connections()
            
            if (connections > 0):            
                self.reset_flightmare_buf_pub[self.env_idx].publish(std_msgs.msg.Empty())
                ctrl_c = True
                
        rospy.wait_for_message("/drone"+ str(self.env_idx+1) + "/self_localization/pose", PoseStamped)
        while (not self.poses_received[self.env_idx]):
            continue
        
        local_state_speed = self.states[self.env_idx + self.num_envs]
        quat = Quat(x=0.0, y=0.0, z=self.agents_pose[self.env_idx].pose.orientation.z, w=self.agents_pose[self.env_idx].pose.orientation.w)
        sp_tmp = np.array([local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z])
        sp_tmp = np.clip(quat.rotate(sp_tmp), -1.0, 1.0)
        local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z = sp_tmp[0], sp_tmp[1], sp_tmp[2]

        local_state_angle = self.euler_from_quaternion(0.0,0.0,self.states[self.env_idx].pose.orientation.z,self.states[self.env_idx].pose.orientation.w)
        angle_distance = math.cos(local_state_angle)
        angle_to_point = np.clip(math.atan2(self.states[self.env_idx].pose.position.x, self.states[self.env_idx].pose.position.y)/(np.pi/2), -1.0, 1.0)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
              unpause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Emptysrv)
              unpause_physics()
        except rospy.ServiceException as e:
              print ("Service call failed: %s"%e)      

        return np.array([self.states[self.env_idx].pose.position.x, self.states[self.env_idx].pose.position.y, self.states[self.env_idx].pose.position.z, local_state_speed.twist.linear.x, local_state_speed.twist.linear.y, local_state_speed.twist.linear.z, angle_distance, angle_to_point]).astype(np.float32)

    def step_async(self, actions: np.ndarray) -> None:
        self.actions = actions

    def step_wait(self) -> VecEnvStepReturn:
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
              unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Emptysrv)
              unpause_physics()
        except rospy.ServiceException as e:
              print ("Service call failed: %s"%e)
        
        for self.env_idx in range(self.num_envs):
            self.take_step(self.actions[self.env_idx])
        time.sleep(1/30)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
              unpause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Emptysrv)
              unpause_physics()
        except rospy.ServiceException as e:
              print ("Service call failed: %s"%e)
              
        for self.env_idx in range(self.num_envs):
            obs, self.buf_rews[self.env_idx], self.buf_dones[self.env_idx], self.buf_infos[self.env_idx] = self.calculate_reward()
            if self.buf_dones[self.env_idx]:
                # save final observation where user can get it, then reset
                self.buf_infos[self.env_idx]["terminal_observation"] = obs
                obs = self.reset_single_env()
            self._save_obs(obs)
        

        return (self._obs_from_buf(), np.copy(self.buf_rews), np.copy(self.buf_dones), deepcopy(self.buf_infos))			

    def get_attr(self, attr_name, indices=None):
        """
        Return attribute from vectorized environment.

        :param attr_name: (str) The name of the attribute whose value to return
        :param indices: (list,int) Indices of envs to get attribute from
        :return: (list) List of values of 'attr_name' in all environments
        """
        pass

    def set_attr(self, attr_name, value, indices=None):
        """
        Set attribute inside vectorized environments.

        :param attr_name: (str) The name of attribute to assign new value
        :param value: (obj) Value to assign to `attr_name`
        :param indices: (list,int) Indices of envs to assign value
        :return: (NoneType)
        """
        pass
    
    def env_is_wrapped(self, wrapper_class: Type[gym.Wrapper], indices: VecEnvIndices = None) -> List[bool]:
        return [False,False,False,False]

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        """
        Call instance methods of vectorized environments.

        :param method_name: (str) The name of the environment method to invoke.
        :param indices: (list,int) Indices of envs whose method to call
        :param method_args: (tuple) Any positional arguments to provide in the call
        :param method_kwargs: (dict) Any keyword arguments to provide in the call
        :return: (list) List of items returned by the environment's method call
        """
        pass

    def seed(self, seed: Optional[int] = None) -> List[Union[None, int]]:
        """
        Sets the random seeds for all environments, based on a given seed.
        Each individual environment will still get its own seed, by incrementing the given seed.

        :param seed: (Optional[int]) The random seed. May be None for completely random seeding.
        :return: (List[Union[None, int]]) Returns a list containing the seeds for each individual env.
            Note that all list elements may be None, if the env does not return anything when being seeded.
        """
        pass	
    

    def PoseStamped_2_mat(self, p):
        q = p.pose.orientation
        pos = p.pose.position
        T = quaternion_matrix([0.0,0.0,q.z,q.w])
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
            roll_x = np.arctan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = np.arcsin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = np.arctan2(t3, t4)
        
            return yaw_z # in radians

    def AgentPoseCallback(self, data):
        self.agents_pose[0] = data

        if (self.poses_received[0]):
            Tw1 = self.PoseStamped_2_mat(self.agents_pose[0])
            Tw2 = self.PoseStamped_2_mat(self.points_pose[0])
            T1w = self.T_inv(Tw1)
            T12 = np.matmul(T1w, Tw2)
            
            self.states[0] = self.Mat_2_posestamped(T12, f_id="map")
            self.states[0].pose.position.x = np.clip(self.states[0].pose.position.x/10, -1, 1)
            self.states[0].pose.position.y = np.clip(self.states[0].pose.position.y/10, -1, 1)
            self.states[0].pose.position.z = np.clip(self.states[0].pose.position.z, -1, 1)
            
            agent_quat = Quat(x=0.0, y=0.0, z=self.agents_pose[0].pose.orientation.z, w=self.agents_pose[0].pose.orientation.w)
            point_quat = Quat(x=0.0, y=0.0, z=self.points_pose[0].pose.orientation.z, w=self.points_pose[0].pose.orientation.w)

            rel_quat = agent_quat - point_quat

            self.states[0].pose.orientation.x = rel_quat.x
            self.states[0].pose.orientation.y = rel_quat.y
            self.states[0].pose.orientation.z = rel_quat.z
            self.states[0].pose.orientation.w = rel_quat.w


            """print("state x", self.state_pose.pose.position.x)
            print("state y", self.state_pose.pose.position.y)
            print("state z", self.state_pose.pose.position.z)"""
            #print("distance reward:")
            #print (self.distance_reward)

    def AgentSpeedCallback(self, data):
        self.agents_speed[0] = data
        
        self.states[self.num_envs].twist.linear.x = np.clip(self.agents_speed[0].twist.linear.x, -1, 1)
        self.states[self.num_envs].twist.linear.y = np.clip(self.agents_speed[0].twist.linear.y, -1, 1)
        self.states[self.num_envs].twist.linear.z = np.clip(self.agents_speed[0].twist.linear.z, -1, 1)


    def PointPoseCallback(self, data):
        self.poses_received[0] = True
        for c, waypoint in enumerate(data.poses):
            self.points_pose[0].header.frame_id = "map"
            self.points_pose[0].pose = waypoint
            #self.initial_distance = -self.CalcDistance([waypoint.position.x, waypoint.position.y, waypoint.position.z],
             #[self.pose_agent.pose.position.x, self.pose_agent.pose.position.y, self.pose_agent.pose.position.z])

    def AgentPoseCallback2(self, data):
        self.agents_pose[1] = data

        if (self.poses_received[1]):
            Tw1 = self.PoseStamped_2_mat(self.agents_pose[1])
            Tw2 = self.PoseStamped_2_mat(self.points_pose[1])
            T1w = self.T_inv(Tw1)
            T1_2 = np.matmul(T1w, Tw2)
            
            self.states[1] = self.Mat_2_posestamped(T1_2, f_id="map")
            self.states[1].pose.position.x = np.clip(self.states[1].pose.position.x/10, -1, 1)
            self.states[1].pose.position.y = np.clip(self.states[1].pose.position.y/10, -1, 1)
            self.states[1].pose.position.z = np.clip(self.states[1].pose.position.z, -1, 1)

            agent_quat = Quat(x=0.0, y=0.0, z=self.agents_pose[1].pose.orientation.z, w=self.agents_pose[1].pose.orientation.w)
            point_quat = Quat(x=0.0, y=0.0, z=self.points_pose[1].pose.orientation.z, w=self.points_pose[1].pose.orientation.w)

            rel_quat = agent_quat - point_quat

            self.states[1].pose.orientation.x = rel_quat.x
            self.states[1].pose.orientation.y = rel_quat.y
            self.states[1].pose.orientation.z = rel_quat.z
            self.states[1].pose.orientation.w = rel_quat.w

            """print("state x", self.state_pose.pose.position.x)
            print("state y", self.state_pose.pose.position.y)
            print("state z", self.state_pose.pose.position.z)"""
            #print("distance reward:")
            #print (self.distance_reward)

    def AgentSpeedCallback2(self, data):
        self.agents_speed[1] = data
        self.states[self.num_envs+1].twist.linear.x = np.clip(self.agents_speed[1].twist.linear.x, -1, 1)
        self.states[self.num_envs+1].twist.linear.y = np.clip(self.agents_speed[1].twist.linear.y, -1, 1)
        self.states[self.num_envs+1].twist.linear.z = np.clip(self.agents_speed[1].twist.linear.z, -1, 1)


    def PointPoseCallback2(self, data):
        self.poses_received[1] = True
        for c, waypoint in enumerate(data.poses):
            self.points_pose[1].header.frame_id = "map"
            self.points_pose[1].pose = waypoint
   
    def AgentPoseCallback3(self, data):
        self.agents_pose[2] = data

        if (self.poses_received[2]):
            Tw1 = self.PoseStamped_2_mat(self.agents_pose[2])
            Tw2 = self.PoseStamped_2_mat(self.points_pose[2])
            T1w = self.T_inv(Tw1)
            T1_2 = np.matmul(T1w, Tw2)
            
            self.states[2] = self.Mat_2_posestamped(T1_2, f_id="map")
            self.states[2].pose.position.x = np.clip(self.states[2].pose.position.x/10, -1, 1)
            self.states[2].pose.position.y = np.clip(self.states[2].pose.position.y/10, -1, 1)
            self.states[2].pose.position.z = np.clip(self.states[2].pose.position.z, -1, 1)

            agent_quat = Quat(x=0.0, y=0.0, z=self.agents_pose[2].pose.orientation.z, w=self.agents_pose[2].pose.orientation.w)
            point_quat = Quat(x=0.0, y=0.0, z=self.points_pose[2].pose.orientation.z, w=self.points_pose[2].pose.orientation.w)

            rel_quat = agent_quat - point_quat

            self.states[2].pose.orientation.x = rel_quat.x
            self.states[2].pose.orientation.y = rel_quat.y
            self.states[2].pose.orientation.z = rel_quat.z
            self.states[2].pose.orientation.w = rel_quat.w

            """print("state x", self.state_pose.pose.position.x)
            print("state y", self.state_pose.pose.position.y)
            print("state z", self.state_pose.pose.position.z)"""
            #print("distance reward:")
            #print (self.distance_reward)
   
    def AgentSpeedCallback3(self, data):
        self.agents_speed[2] = data
        self.states[self.num_envs+2].twist.linear.x = np.clip(self.agents_speed[2].twist.linear.x, -1, 1)
        self.states[self.num_envs+2].twist.linear.y = np.clip(self.agents_speed[2].twist.linear.y, -1, 1)
        self.states[self.num_envs+2].twist.linear.z = np.clip(self.agents_speed[2].twist.linear.z, -1, 1)


    def PointPoseCallback3(self, data):
        self.poses_received[2] = True
        for c, waypoint in enumerate(data.poses):
            self.points_pose[2].header.frame_id = "map"
            self.points_pose[2].pose = waypoint
   
    def AgentPoseCallback4(self, data):
        self.agents_pose[3] = data

        if (self.poses_received[3]):
            Tw1 = self.PoseStamped_2_mat(self.agents_pose[3])
            Tw2 = self.PoseStamped_2_mat(self.points_pose[3])
            T1w = self.T_inv(Tw1)
            T1_2 = np.matmul(T1w, Tw2)
            
            self.states[3] = self.Mat_2_posestamped(T1_2, f_id="map")
            self.states[3].pose.position.x = np.clip(self.states[3].pose.position.x/10, -1, 1)
            self.states[3].pose.position.y = np.clip(self.states[3].pose.position.y/10, -1, 1)
            self.states[3].pose.position.z = np.clip(self.states[3].pose.position.z, -1, 1)

            agent_quat = Quat(x=0.0, y=0.0, z=self.agents_pose[3].pose.orientation.z, w=self.agents_pose[3].pose.orientation.w)
            point_quat = Quat(x=0.0, y=0.0, z=self.points_pose[3].pose.orientation.z, w=self.points_pose[3].pose.orientation.w)

            rel_quat = agent_quat - point_quat

            self.states[3].pose.orientation.x = rel_quat.x
            self.states[3].pose.orientation.y = rel_quat.y
            self.states[3].pose.orientation.z = rel_quat.z
            self.states[3].pose.orientation.w = rel_quat.w

            """print("state x", self.state_pose.pose.position.x)
            print("state y", self.state_pose.pose.position.y)
            print("state z", self.state_pose.pose.position.z)"""
            #print("distance reward:")
            #print (self.distance_reward)
   
    def AgentSpeedCallback4(self, data):
        self.agents_speed[3] = data
        self.states[self.num_envs+3].twist.linear.x = np.clip(self.agents_speed[3].twist.linear.x, -1, 1)
        self.states[self.num_envs+3].twist.linear.y = np.clip(self.agents_speed[3].twist.linear.y, -1, 1)
        self.states[self.num_envs+3].twist.linear.z = np.clip(self.agents_speed[3].twist.linear.z, -1, 1)


    def PointPoseCallback4(self, data):
        self.poses_received[3] = True
        for c, waypoint in enumerate(data.poses):
            self.points_pose[3].header.frame_id = "map"
            self.points_pose[3].pose = waypoint

    def CalcDistance(self, p1, p2):
        return np.linalg.norm(p1-p2)


    def PublishSpeedReferences(self):
        ctrl_c = False
        while not ctrl_c:
            connections = self.speed_references_buf_pub[self.env_idx].get_num_connections()
            if (connections > 0):
                self.speed_references_buf_pub[self.env_idx].publish(self.speed_reference)
                ctrl_c = True

    def setControlMode(self, control_mode):
        rospy.wait_for_service("/drone" + str(self.env_idx+1) + "/set_control_mode")
        try:
            setControlModeClientSrv = rospy.ServiceProxy("/drone" + str(self.env_idx+1) + "/set_control_mode", SetControlMode)
            setControlModeSrv = SetControlModeRequest()
            setControlModeSrv.controlMode.mode = control_mode            
            response = setControlModeClientSrv(setControlModeSrv)
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

    def close(self) -> None:
        pass

    def _obs_from_buf(self) -> VecEnvObs:
        return dict_to_obs(self.observation_space, copy_obs_dict(self.buf_obs))	

    def _save_obs(self, obs: VecEnvObs) -> None:
        for key in self.keys:
            if key is None:
                self.buf_obs[key][self.env_idx] = obs
            else:
                self.buf_obs[key][self.env_idx] = obs[key]

    def move_at_speed(self):

        while not rospy.is_shutdown():
        
            action = [1.0, 0.0, 0.0]
            quat = Quat(x=0.0, y=0.0, z=self.agents_pose[0].pose.orientation.z, w=self.agents_pose[0].pose.orientation.w)
            #action_prime = action[:3]
            action_prime = np.clip(quat.rotate(action), -1.0, 1.0)
            print (action_prime)

            ctrl_c = False
            reference = JointTrajectoryPoint()
            reference.positions = [0.0, 0.0, 0.0, 0.0]
            reference.velocities = [action_prime[0],action_prime[1],action_prime[2], -1.0]
            reference.accelerations = [0.0, 0.0, 0.0, 0.0]
            reference.effort = []
            while not ctrl_c:
                connections = self.speed_references_buf_pub[0].get_num_connections()
                        
                if (connections > 0):
                    self.speed_references_buf_pub[0].publish(reference)
                    ctrl_c = True

            ctrl_c = False       
