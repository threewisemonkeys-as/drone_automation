#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped

from controller import DroneController
import utils


class WaypointController(DroneController):
    """
    Waypoint controller for drone
    """
    def __init__(self, threshold_dist=0.2, hover_time=1, verbose=True):
        super(WaypointController, self).__init__()
        self.T = threshold_dist
        self.verbose = verbose

    def prop_vel_goto(self, x, y, z, K=0.2, hold_duration=2):
        """
        Goto a specified point using a proportional velocity controller
        """
        rospy.loginfo("Going to (%s, %s, %s)", x, y, z)
        while not (utils.dist(utils.unwrap_pose(self.pose.pose)[0],
                              (x, y, z)) < self.T
                   and utils.norm(utils.unwrap_twist(
                       self.vel.twist)[0]) < K * self.T):
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")

            vx = K * (x - self.pose.pose.position.x)
            vy = K * (y - self.pose.pose.position.y)
            vz = K * (z - self.pose.pose.position.z)
            self.set_vel(vx, vy, vz)
            self.rate.sleep()

        self.set_vel(0, 0, 0)
        rospy.sleep(rospy.Duration(hold_duration))

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
