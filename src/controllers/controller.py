#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State  # pylint: disable=import-error
from mavros_msgs.srv import (CommandBool,  # pylint: disable=import-error
                             SetMode)

from .. import utils


class DroneController(object):
    """
    Simple drone controller class
    """
    def __init__(self, rate=20):

        # Rate has to be greater than 2Hz for px4
        self.rate = rospy.Rate(rate)

        self.current_state = State()
        self.pose = PoseStamped()
        self.setpoint_pos = PoseStamped()
        self.vel = TwistStamped()
        self.setpoint_vel = Twist()

        self.state_sub = rospy.Subscriber("mavros/state",
                                          State,
                                          callback=self.state_cb)
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose",
                                         PoseStamped,
                                         callback=self.pose_cb)
        self.vel_sub = rospy.Subscriber("mavros/local_position/velocity",
                                        TwistStamped,
                                        callback=self.vel_cb)

        self.setpoint_pos_pub = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.setpoint_vel_pub = rospy.Publisher(
            "mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)

        rospy.wait_for_service('mavros/cmd/arming')
        self.arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        rospy.wait_for_service('mavros/set_mode')
        self.set_mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

    def state_cb(self, data):
        """
        Callback for state subscriber. Recieves and stores current state.
        """
        self.current_state = data

    def pose_cb(self, data):
        """
        Callback for pose subscriber. Recieves and stores current pose.
        """
        self.pose = data

    def vel_cb(self, data):
        """
        Callback for pose subscriber. Recieves and stores current pose.
        """
        self.vel = data

    def init_stream(self, rate, n=100):
        """
        Stream setpoint for specified number of times.
        Required before setting mode to OFFBOARD.
        """
        for _ in range(n):
            pose = PoseStamped()
            self.setpoint_pos_pub.publish(pose)
            rate.sleep()

    def arm(self):
        """
        Arms the drone through arming service.
        """
        result = self.arm_service(True)
        if result.success:
            rospy.loginfo("Arming successful")
        else:
            rospy.loginfo("Arming unsuccessful")

    def disarm(self):
        """
        Disarms the drone through arming service
        """
        result = self.arm_service(True)
        if result.success:
            rospy.loginfo("Disarming successful")
        else:
            rospy.loginfo("Disarming unsuccessful")

    def land(self):
        """
        Lands drone and disarms it
        """
        rospy.loginfo("Landing")
        self.set_mode("AUTO.LAND")

    def set_mode(self, mode):
        """
        Sets the mode of the drone through set mode service.
        """
        result = self.set_mode_service(custom_mode=mode)
        if result.mode_sent:
            rospy.logdebug("Setting mode to %s successful", mode)
        else:
            rospy.logdebug("Setting mode to %s unsuccessful", mode)

    def _publish_pos(self):
        self.setpoint_pos_pub.publish(self.setpoint_pos)

    def set_pos(self, x=None, y=None, z=None):
        """
        Sets position of drone by publishing local position to move to,
        to the setpoint_position/local topic.
        """

        if x is not None: self.setpoint_pos.pose.position.x = x
        if y is not None: self.setpoint_pos.pose.position.y = y
        if z is not None: self.setpoint_pos.pose.position.z = z

        self._publish_pos()

    def _publish_vel(self):
        self.setpoint_vel_pub.publish(self.setpoint_vel)

    def set_vel(self, vx=None, vy=None, vz=None, az=None):
        """
        Sets position of drone by publishing local position to move to,
        to the setpoint_position/local topic.
        """
        if vx is not None: self.setpoint_vel.linear.x = vx
        if vy is not None: self.setpoint_vel.linear.y = vy
        if vz is not None: self.setpoint_vel.linear.z = vz
        if az is not None: self.setpoint_vel.angular.z = az

        self._publish_vel()

    def takeoff(self, h=1):
        """
        Prepare drone and takeoff
        """
        # Wait till connection established between MAVROS and px4 autopilot
        rospy.loginfo("Connecting to autopilot")
        while not self.current_state.connected:
            self.rate.sleep()

        # Arms the drone
        self.arm()

        # Need to start streaming setpoints before entering OFFBOARD mode
        self.init_stream(self.rate)

        rospy.loginfo("Taking off")
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        z = h

        while utils.dist(utils.unwrap_pose(self.pose.pose)[0],
                         (x, y, z)) > 0.1:
            # Attempt to enter into OFFBOARD mode
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")

            # Set position to move to
            self.set_pos(x, y, z)
            self.rate.sleep()


if __name__ == "__main__":

    rospy.init_node("move_drone")

    # Initialises controller object with its publishers, subscribers and clients
    drone = DroneController()

    try:
        # Drone takes off and lands
        drone.takeoff(2)
        drone.land()

    except KeyboardInterrupt:
        # Land drone if keyboard interrupt is recieved
        print(" <-- In keyboard interrupt ...")
        try:
            drone.land()
        except:
            print("Drone landed!")
