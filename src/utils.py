#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

import math


def norm(x):
    """
    Returns the norm of a any-dimensional tuple
    """
    return math.sqrt(sum([i**2 for i in x]))


def dist(p1, p2):
    """
    Returns distance between two any-dimensional points given as tuples
    """
    return math.sqrt(sum([(i - j)**2 for i, j in zip(p1, p2)]))


def unwrap_pose(pose):
    """
    Unwraps geometry_msgs/Pose into two tuples of position and orientation 
    """
    p_x = pose.position.x
    p_y = pose.position.y
    p_z = pose.position.z
    o_x = pose.orientation.x
    o_y = pose.orientation.y
    o_z = pose.orientation.z
    o_w = pose.orientation.w

    return (p_x, p_y, p_z), (o_x, o_y, o_z, o_w)


def unwrap_twist(twist):
    """
    Unwraps geometry_msgs/Twist into two tuples of linear and angular velocities 
    """
    l_x = twist.linear.x
    l_y = twist.linear.y
    l_z = twist.linear.z
    a_x = twist.angular.x
    a_y = twist.angular.y
    a_z = twist.angular.z

    return (l_x, l_y, l_z), (a_x, a_y, a_z)
