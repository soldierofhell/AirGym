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

# Get the environment and extract the number of actions.
env = gym.make(args.env_name)
np.random.seed(123)
env.seed(123)
nb_actions = env.action_space.n


# Next, we build our model. We use the same model that was described by Mnih et al. (2015).
INPUT_SHAPE = (30, 100)
WINDOW_LENGTH = 1
# Next, we build our model. We use the same model that was described by Mnih et al. (2015).
input_shape = (WINDOW_LENGTH,) + INPUT_SHAPE

environment = suite_gym.load(args.env_name)