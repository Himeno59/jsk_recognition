#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class CalcBallCentroidNode:
    def __init__(self):
        rospy.init_node('calc_ball_centroid_node', anonymous=True)
        self.bridge = CvBridge()

        # subscriber
        self.point_sub = rospy.Subscriber('/centroid_publisher/output/point', PointStamped, self.calc_cb)

        # publisher
        self.point_pub = rospy.Publisher('/ball_detection/ball_centroid_point', PointStamped, queue_size=1)
        
    def calc_cb(self, msg):
        # 見えている点群(半球)の重心
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        D = (x**2 + y**2 + z**2)**0.5
       
        # 半球の重心から球の中心までの距離
        k = 0.5 * 0.1225
        
        msg.point.x = x + k * x/D
        msg.point.y = y + k * y/D
        msg.point.z = z + k * z/D
        try:
            self.point_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
        

if __name__ == '__main__':
    try:
        calc_ball_centroid_node = CalcBallCentroidNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
