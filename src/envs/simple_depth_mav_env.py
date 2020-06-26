#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

from __future__ import print_function

import gym
import os
import rospkg
import rospy

from .. import DepthProcessor, WaypointController
from .mav_env import MavEnv

gym.envs.registration.register(
    id='SimpleDepthMavEnv-v0',
    entry_point='simple_depth_mav_env:SimpleDepthMavEnv',
)

rospack = rospkg.RosPack()
package_path = rospack.get_path('drone_automation')
default_launch_file_path = os.path.join(package_path,
                                        "launch/iris_depth_maze.launch")

MAX_STEPS = 100
ACTION_LOOKUP = {0: "FORWARD", 1: "RIGHT", 2: "LEFT"}


class SimpleDepthMavEnv(MavEnv):
    def __init__(self,
                 launch_file=default_launch_file_path,
                 move_dist=0.5,
                 n_features=3):
        super(SimpleDepthMavEnv, self).__init__(launch_file)

        self.observation_space = gym.spaces.Box(low=0,
                                                high=100,
                                                shape=(n_features, ))
        self.action_space = gym.spaces.Discrete(n=len(ACTION_LOOKUP))

        self._depth_processor = DepthProcessor()
        self.n_features = n_features
        self.move_dist = move_dist

    def _execute_action(self, action):
        if action == 0:
            self.controller.goto_relative(x=self.move_dist)
        elif action == 1:
            self.controller.goto_relative(y=-self.move_dist)
        elif action == 2:
            self.controller.goto_relative(y=self.move_dist)

    def _observe(self):
        return self._depth_processor.obstacle_features

    def _compute_reward(self, obs):
        return sum(obs)

    def _update_done(self):
        return self.steps_taken >= MAX_STEPS
