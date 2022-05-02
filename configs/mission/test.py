#!/usr/bin/env python3

from locale import normalize
from random import normalvariate
import mission_execution_control as mxc
import rospy
import time
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
from std_srvs.srv import *
from std_srvs.srv import Empty as Emptysrv
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from aerostack_msgs.msg import *
from trajectory_msgs.msg import *
from behavior_execution_manager_msgs.srv import ActivateBehavior, ActivateBehaviorRequest, ActivateBehaviorResponse
import ast
import sys
import os
PROJECT = os.getenv('PWD')
print (PROJECT)
sys.path.append(PROJECT)
from Flight_Gym import *

from stable_baselines3 import PPO
import torch as th

'''
This is a simple mission, the drone takes off, follows a path and lands
1.startTask. Starts the task and continues.
2.executeTask. Starts the task and waits until it stops.
'''
hover_pub = rospy.Publisher('/drone1/motion_reference/hover', JointTrajectoryPoint, queue_size=1)

path = String()
ctrl = False

"""def callback(msg):
  global path
  global ctrl
  path = msg
  print(str(ast.literal_eval(path.data)))
  ctrl = True"""

def mission():
  rospy.wait_for_service('/drone1/quadrotor_motion_with_df_control/behavior_quadrotor_df_motion_control/activate_behavior')
  try:
        activate_controller = rospy.ServiceProxy('/drone1/quadrotor_motion_with_df_control/behavior_quadrotor_df_motion_control/activate_behavior', ActivateBehavior)
        activate_controller_req = ActivateBehaviorRequest()
        activate_controller_req.timeout = 10000
        activate_controller(activate_controller_req)
  except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

  rospy.wait_for_service('/drone1/quadrotor_motion_with_df_control/behavior_take_off_with_df/activate_behavior')
  try:
        take_off = rospy.ServiceProxy('/drone1/quadrotor_motion_with_df_control/behavior_take_off_with_df/activate_behavior', ActivateBehavior)
        take_off_req = ActivateBehaviorRequest()
        take_off_req.timeout = 10
        take_off(take_off_req)
  except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

  time.sleep(10)


  """reference = JointTrajectoryPoint()
  reference.positions = [0.0, 0.0, 1.0, 0.0]
  reference.velocities = [0.0, 0.0, 0.0, 0.0]
  reference.accelerations = [0.0, 0.0, 0.0, 0.0]
  reference.effort = []
  ctrl_c = False

  while not ctrl_c:
      connections = hover_pub.get_num_connections()
          
      if (connections > 1):
        hover_pub.publish(reference)
        ctrl_c = True"""
  global path
  global ctrl
  t = Environment(1)

  print("Starting mission...")
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()
  
  time.sleep(5)
  
  t.move_at_speed()
  
  time.sleep(5)
  
  obs = t.reset()

  
  print('Finish mission...')
