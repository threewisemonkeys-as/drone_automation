#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped

from .. import utils
from .controller import DroneController


class WaypointController(DroneController):
    """
    Waypoint controller for drone
    """
    def __init__(self, threshold_dist=0.2, hover_time=1, verbose=True):
        super(WaypointController, self).__init__()
        self.T = threshold_dist
        self.verbose = verbose
        self.reached = True

    def prop_vel_goto(self, x=None, y=None, z=None, K=0.2, hold_duration=1):
        """
        Goto a specified point using a proportional velocity controller
        """
        self.set_target_pos(x, y, z)
        rospy.loginfo("Going to %s", self.target_pos)
        
        self.reached = False
        while not (utils.dist(
                utils.unwrap_pose(self.pose.pose)[0], self.target_pos) < self.T
                   and utils.norm(utils.unwrap_twist(
                       self.vel.twist)[0]) < K * self.T):
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")

            vx = K * (self.target_pos.x - self.pose.pose.position.x)
            vy = K * (self.target_pos.y - self.pose.pose.position.y)
            vz = K * (self.target_pos.z - self.pose.pose.position.z)
            self.set_vel(vx, vy, vz)
            # self.rate.sleep()

        self.set_vel(0, 0, 0)
        self.reached = True
        rospy.loginfo("Reached %s", utils.unwrap_pose(self.pose.pose)[0])
        rospy.sleep(rospy.Duration(hold_duration))

    def goto_relative(self, x=0, y=0, z=0, K=0.2, hold_duration=1):
        """
        Goto a relative position using a proportional velocity controller
        """
        self.target_pos.x += x
        self.target_pos.y += y
        self.target_pos.z += z
        self.prop_vel_goto(K=K, hold_duration=hold_duration)

    def traverse_path(self, path, hold_duration=2, takeoff=False, land=False):
        """
        Traverses path given as list of points
        """
        if takeoff: self.takeoff()

        rospy.loginfo("Traversing path")
        for p in path:
            self.prop_vel_goto(*p, hold_duration=hold_duration)

        rospy.loginfo("Path traversal completed")

        if land: self.land()


if __name__ == "__main__":

    rospy.init_node("waypoint_controller")

    # path = [(0, 0, 2), (0, 2, 2), (2, 2, 2), (2, 0, 2), (0, 0, 2)]
    path = [(6, 0, 2), (6, 4, 2)]

    wc = WaypointController()
    wc.takeoff()
    wc.traverse_path(path)
    wc.land()
