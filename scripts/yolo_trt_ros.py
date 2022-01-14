#!/usr/bin/env python3
# coding=utf-8
'''
Author: Jianheng Liu
Date: 2022-01-08 17:10:49
LastEditors: Jianheng Liu
LastEditTime: 2022-01-13 20:38:54
Description: Description
'''
"""
An example that uses TensorRT's Python api to make inferences.
"""




import ctypes
import random
import sys
import threading
import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from yolo_ros.msg import *
from yolov5_tensorrt import YoLov5TRT
class Detector:

    def __init__(self):
        # # Choose to use a config and initialize the detecto
        # Config file

        self._publish_rate = rospy.get_param('~publish_rate', 50)
        self._visualization = rospy.get_param('~visualization', False)

        self._last_msg = None
        self._msg_lock = threading.Lock()

        self.yolo_pub = rospy.Publisher(
            "/untracked_info", yolo_ros.msg.DetectionMessages, queue_size=10)
        if self._visualization:
            self.image_pub = rospy.Publisher(
                "/yolo_result_image", Image, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "~image_topic", Image, self._image_callback, queue_size=10)

        self.categories = ["person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
                           "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
                           "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                           "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
                           "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
                           "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
                           "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
                           "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
                           "hair drier", "toothbrush"]

        # load custom plugin and engine
        PLUGIN_LIBRARY = sys.path[0] + \
            "/../Thirdparty/yolov5_tensorrtx/build/libmyplugins.so"
        engine_file_path = sys.path[0] + \
            "/../Thirdparty/yolov5_tensorrtx/build/yolov5n.engine"

        ctypes.CDLL(PLUGIN_LIBRARY)

        # a YoLov5TRT instance
        self.yolov5_wrapper = YoLov5TRT(engine_file_path)

        print('batch size is', self.yolov5_wrapper.batch_size)

        # warm_up
        for i in range(5):
            self.yolov5_wrapper.infer(np.zeros([640, 640, 3], dtype=np.uint8))

    def run(self):
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            if self._msg_lock.acquire(False):
                msg = self._last_msg
                self._last_msg = None
                self._msg_lock.release()
            else:
                rate.sleep()
                continue

            if msg is not None:
                start = time.time()
                image_np = np.ndarray(shape=(msg.height, msg.width, 3),
                                      dtype=np.uint8, buffer=msg.data)
                result_boxes, result_scores, result_classid = self.yolov5_wrapper.infer(
                    image_np)

                results = yolo_ros.msg.DetectionMessages()
                results.header = msg.header

                for j in range(len(result_boxes)):
                    result = yolo_ros.msg.DetectionMessage()
                    result.x1 = int(result_boxes[j][0])
                    result.y1 = int(result_boxes[j][1])
                    result.x2 = int(result_boxes[j][2])
                    result.y2 = int(result_boxes[j][3])
                    result.class_pred = int(result_classid[j])
                    result.score = result_scores[j]
                    result.label = self.categories[int(result_classid[j])]
                    results.data.append(result)
                    results.detection_num += 1

                self.yolo_pub.publish(results)
                use_time = time.time()-start
                # print('time->{:.2f}ms'.format(use_time * 1000))

                # Visualize results
                if self._visualization:
                    image_out = msg
                    for j in range(len(result_boxes)):
                        box = result_boxes[j]
                        self.plot_one_box(
                            box,
                            image_np,
                            label="{}:{:.2f}".format(
                                self.categories[int(result_classid[j])
                                                ], result_scores[j]
                            ),
                        )
                    image_out.data = image_np.tobytes()

                    self.image_pub.publish(image_out)
            rate.sleep()

        # destroy the instance
        self.yolov5_wrapper.destroy()

    def _image_callback(self, msg):
        rospy.logdebug("Get an image")
        if self._msg_lock.acquire(False):
            self._last_msg = msg
            self._msg_lock.release()

    def service_handler(self, request):
        return self._image_callback(request.image)

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        """
        description: Plots one bounding box on image img,
                    this function comes from YoLov5 project.
        param: 
            x:      a box likes [x1,y1,x2,y2]
            img:    a opencv image object
            color:  color to draw rectangle, such as (0,255,0)
            label:  str
            line_thickness: int
        return:
            no return

        """
        tl = (
            line_thickness or round(
                0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
        )  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(
                label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(
                img,
                label,
                (c1[0], c1[1] - 2),
                0,
                tl / 3,
                [225, 255, 255],
                thickness=tf,
                lineType=cv2.LINE_AA,
            )


if __name__ == '__main__':
    rospy.init_node('detector')

    obj = Detector()
    obj.run()
