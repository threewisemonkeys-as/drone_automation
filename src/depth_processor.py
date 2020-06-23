#! /usr/bin/python2
# Atharv Sonwane <atharvs.twm@gmail.com>

from __future__ import print_function

import rospy
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray


class DepthProcessor(object):
    def __init__(self):
        self.obstacle_features = []
        self.pc_sub = rospy.Subscriber("/r200/depth/points",
                                       PointCloud2,
                                       callback=self._cb)
        self.pub = rospy.Publisher("/automation/obstacle_features",
                                   Float32MultiArray,
                                   queue_size=50)

    def _cb(self, cloud):
        self.cloud = cloud
        self.pts_gen = pc2.read_points(cloud,
                                       skip_nans=False,
                                       field_names=("x", "y", "z"))
        self._process()

    def _process(self):
        min_dist = [1000000, 1000000, 1000000
                    ]  #min distance corresponding to [front , left , right]
        for p in self.pts_gen:
            if abs(p[0]) > abs(p[1]):
                if p[0] > 0 and p[0] < min_dist[0]:
                    min_dist[0] = p[0]
            else:
                if p[1] > 0:
                    if p[1] < min_dist[1]:
                        min_dist[1] = p[1]
                else:
                    if -1 * p[1] < min_dist[2]:
                        min_dist[2] = -1 * p[1]
        self.obstacle_features = min_dist


if __name__ == "__main__":

    rospy.init_node("depth_processor")
    dp = DepthProcessor()
    rospy.spin()
