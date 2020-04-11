# -*- coding: utf-8 -*-
"""
Created on Sat Oct 14 14:16:10 2017

@author: Kjell
"""
import numpy as np
import gym

import gym_airsim.envs
import gym_airsim


import argparse

#from PIL import Image

import tensorflow as tf
tf.compat.v1.enable_v2_behavior()



from tf_agents.environments import suite_gym



parser = argparse.ArgumentParser()
parser.add_argument('--mode', choices=['train', 'test'], default='train')
parser.add_argument('--env-name', type=str, default='AirSimEnv-v42')
parser.add_argument('--weights', type=str, default=None)
args = parser.parse_args()

environment = suite_gym.load(args.env_name)
print('action_spec:', environment.action_spec())
print('time_step_spec.observation:', environment.time_step_spec().observation)
print('time_step_spec.step_type:', environment.time_step_spec().step_type)
print('time_step_spec.discount:', environment.time_step_spec().discount)
print('time_step_spec.reward:', environment.time_step_spec().reward)

action = np.array(1, dtype=np.int32)
time_step = environment.reset()
print(time_step)
while not time_step.is_last():
  time_step = environment.step(action)
  print(time_step)