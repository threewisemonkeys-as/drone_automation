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
        print(self.cloud.height)
        print(self.cloud.width)
        print(self.cloud.point_step)
        print(self.cloud.row_step)
        print(self.cloud.fields)
        for p in self.pts_gen:
            print(p[0])
            print(p[0])
            print(p[0])
            break


if __name__ == "__main__":

    rospy.init_node("depth_processor")
    dp = DepthProcessor()
    rospy.spin()
