#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

from __future__ import print_function

import gym
import roslaunch
import rospy
from std_srvs.srv import Empty

from .. import DepthProcessor, WaypointController


class MavEnv(gym.Env):
    def __init__(self, launch_file):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()
        rospy.init_node("mav_env")
        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/unpause_physics')
        rospy.wait_for_service('/gazebo/pause_physics')
        self._unpause_sim = rospy.ServiceProxy('/gazebo/unpause_physics',
                                               Empty,
                                               persistent=True)
        self._pause_sim = rospy.ServiceProxy('/gazebo/pause_physics',
                                             Empty,
                                             persistent=True)
        self._reset_sim = rospy.ServiceProxy('/gazebo/reset_world',
                                             Empty,
                                             persistent=True)
        self.controller = WaypointController()
        self.steps_taken = 0
        print("[ENV] Initialised")

    def __del__(self):
        self.close()

    def _execute_action(self, action):
        raise NotImplementedError

    def _observe(self):
        raise NotImplementedError

    def _compute_reward(self, obs):
        raise NotImplementedError

    def _update_done(self):
        raise NotImplementedError

    def pause_sim(self):
        print("[ENV] Pausing sim")
        rospy.wait_for_service('/gazebo/pause_physics')
        self._pause_sim()

    def unpause_sim(self):
        print("[ENV] Unpaussing sim")
        rospy.wait_for_service('/gazebo/unpause_physics')
        self._unpause_sim()

    def reset_sim(self):
        print("[ENV] Resetting sim")
        rospy.wait_for_service('/gazebo/reset_world')
        self._reset_sim()

    def step(self, action):
        print("[ENV] Taking step", action)
        self.unpause_sim()
        self._execute_action(action)
        self.pause_sim()
        self.steps_taken += 1
        obs = self._observe()
        reward = self._compute_reward(obs)
        done = self._update_done()
        info = {}
        return obs, reward, done, info

    def reset(self):
        print("[ENV] Resetting env")
        self.steps_taken = 0
        self.reset_sim()
        self.unpause_sim()
        self.controller.takeoff(1.5)
        self.pause_sim()
        obs = self._observe()
        return obs

    def render(self, mode="human", close=False):
        raise NotImplementedError  # TODO

    def close(self):
        print("[ENV Shutting down")
        self.launch.shutdown()
