#!/usr/bin/env python3
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2022-01-08 17:10:49
LastEditors: Jianheng Liu
LastEditTime: 2022-01-08 17:27:55
Description: Description
'''
import rospy
from yolo_bridge.yolo_bridge import Ros2Yolo

if __name__ == "__main__":
    yoloBridge = Ros2Yolo()
    rospy.spin()
