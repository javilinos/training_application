#!/usr/bin/env python3

from locale import normalize
from random import normalvariate
import mission_execution_control as mxc
import rospy
import time
from std_srvs.srv import Empty as Emptysrv
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import *
from std_msgs.msg import String
from behavior_execution_manager_msgs.srv import ActivateBehavior, ActivateBehaviorRequest, ActivateBehaviorResponse
import ast
import sys
import os
PROJECT = os.getenv('PWD')
print (PROJECT)
sys.path.append(PROJECT)
from stable_baselines3.common.vec_env.vec_monitor import VecMonitor
from typing import Callable
from Flight_Gym import *

from stable_baselines3 import PPO
import torch as th

'''
This is a simple mission, the drone takes off, follows a path and lands
1.startTask. Starts the task and continues.
2.executeTask. Starts the task and waits until it stops.
'''
N_ENVS = 4

"""def callback(msg):
  global path
  global ctrl
  path = msg
  print(str(ast.literal_eval(path.data)))
  ctrl = True"""

from stable_baselines3.common.callbacks import BaseCallback

def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """
    Linear learning rate schedule.

    :param initial_value: Initial learning rate.
    :return: schedule that computes
      current learning rate depending on remaining progress
    """
    def func(progress_remaining: float) -> float:
        """
        Progress will decrease from 1 (beginning) to 0.

        :param progress_remaining:
        :return: current learning rate
        """
        return progress_remaining * initial_value

    return func

class CustomCallback(BaseCallback):
    """
    A custom callback that derives from ``BaseCallback``.

    :param verbose: (int) Verbosity level 0: not output 1: info 2: debug
    """
    def __init__(self, verbose=0):
        super(CustomCallback, self).__init__(verbose)
        #self.update_pub = rospy.Publisher('/environment/flightmare/net_update', EmptyMsg, queue_size=1)
        # Those variables will be accessible in the callback
        # (they are defined in the base class)
        # The RL model
        # self.model = None  # type: BaseAlgorithm
        # An alias for self.model.get_env(), the environment used for training
        # self.training_env = None  # type: Union[gym.Env, VecEnv, None]
        # Number of time the callback was called
        # self.n_calls = 0  # type: int
        # self.num_timesteps = 0  # type: int
        # local and global variables
        # self.locals = None  # type: Dict[str, Any]
        # self.globals = None  # type: Dict[str, Any]
        # The logger object, used to report things in the terminal
        # self.logger = None  # stable_baselines3.common.logger
        # # Sometimes, for event callback, it is useful
        # # to have access to the parent object
        # self.parent = None  # type: Optional[BaseCallback]

    def _on_training_start(self) -> None:
        """
        This method is called before the first rollout starts.
        """
        print("--------------------")
        print (self.model)
        print("--------------------")
        print (self.training_env)
        print("--------------------")
        print (self.model.get_env().action_space)
        print("--------------------")
        print (self.model.get_env().observation_space)
        print("--------------------")
        print (self.model.get_env().keys)
        print("--------------------")
        print (self.model.get_env().observation_space)
        print("--------------------")
        pass
        

    def _on_rollout_start(self) -> None:
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
          unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', Emptysrv)
          unpause_physics()
        except rospy.ServiceException as e:
          print ("Service call failed: %s"%e)
        

    def _on_step(self) -> bool:
        """
        This method will be called by the model after each call to `env.step()`.

        For child callback (of an `EventCallback`), this will be called
        when the event is triggered.

        :return: (bool) If the callback returns False, training is aborted early.
        """
        
        return True

    def _on_rollout_end(self) -> None:
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
          pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', Emptysrv)
          pause_physics()
        except rospy.ServiceException as e:
          print ("Service call failed: %s"%e)
        #self.update_pub.publish()

    def _on_training_end(self) -> None:
        """
        This event is triggered before exiting the `learn()` method.
        """
        pass

cb = CustomCallback()

def mission():
    for env in range(N_ENVS):
      rospy.wait_for_service("/drone" + str(env+1) + "/quadrotor_motion_with_df_control/behavior_quadrotor_df_motion_control/activate_behavior")
      try:
            activate_controller = rospy.ServiceProxy("/drone" + str(env+1) + "/quadrotor_motion_with_df_control/behavior_quadrotor_df_motion_control/activate_behavior", ActivateBehavior)
            activate_controller_req = ActivateBehaviorRequest()
            activate_controller_req.timeout = 10000
            activate_controller(activate_controller_req)
      except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

      rospy.wait_for_service("/drone" + str(env+1) + "/quadrotor_motion_with_df_control/behavior_take_off_with_df/activate_behavior")
      try:
            take_off = rospy.ServiceProxy("/drone" + str(env+1) + "/quadrotor_motion_with_df_control/behavior_take_off_with_df/activate_behavior", ActivateBehavior)
            take_off_req = ActivateBehaviorRequest()
            take_off_req.timeout = 10
            take_off(take_off_req)
      except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

    time.sleep(10)

    t = Environment(4)
    t = VecMonitor(t, filename="/home/javilinos/PPO_Monitor")
    model = PPO("MlpPolicy", t, tensorboard_log="/home/javilinos/PPO", verbose=1, device=th.device("cpu"), n_steps=512, batch_size=64, gae_lambda=0.95, gamma=0.99, n_epochs=20, ent_coef=0.01, vf_coef=0.5, clip_range=0.2, learning_rate=3e-05, use_sde=True, policy_kwargs=dict(
                        log_std_init=-1,
                        ortho_init=False,
                        activation_fn=th.nn.ReLU,
                        net_arch=[dict(pi=[512, 512], vf=[512, 512])]
                        ))
    print("Starting mission...")
    model.learn(total_timesteps=3000000, callback=cb)
    model.save("saved_models")
    obs = t.reset()
    
    print('Finish mission...')
