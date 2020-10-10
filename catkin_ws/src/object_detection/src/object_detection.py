#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  8 22:01:43 2020

@author: 203
"""
import rospy
from std_msgs.msg import String, Float64, Header
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as Image_msg
from sensor_msgs.msg import CompressedImage


import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import mxnet as mx
import gluoncv as gcv

import time

import threading

os.environ["MXNET_CUDNN_AUTOTUNE_DEFAULT"] = "0"


class ObjectDetection():
    def __init__(self):
        self.cv_image = []

        self.is_aim = False
        self.is_lock = False
        self.non_track = 0
        self.score_threshold = 0.5
        self.lock_threshold = 0.5
        self.track_threshold = 0.5
        self.move_xy = (0, 0)

        self.detect_device = 'gpu'    # 'cpu' & 'gpu'
        self.detect_mode = 'standard'  # 'standard' & 'fast'
        self.track_mode = 'standard'  # 'standard' & 'fast'
        self.class_mode = 'human'       # 'human' & 'cat' & 'dog'

        if self.detect_device == 'cpu':
            self.ctx = mx.cpu(0)
        elif self.detect_device == 'gpu':
            self.ctx = mx.gpu(0)

        if self.detect_mode == 'standard':
            self.detector = gcv.model_zoo.get_model(
                'yolo3_darknet53_coco', pretrained=True)
        elif self.detect_mode == 'fast':
            self.detector = gcv.model_zoo.get_model(
                'yolo3_mobilenet1.0_coco', pretrained=True)
        self.detector.collect_params().reset_ctx(self.ctx)
        self.detector.hybridize()

        # ret, first_frame = cap.read()
        self.width = rospy.get_param("object_detection/image_width", 640)
        self.height = rospy.get_param("object_detection/image_height", 480)
        self.rescale = self.height / 320

        self.center_x = int(self.width/2)
        self.center_y = int(self.height/2)
        self.maskSize = int(min(self.center_x, self.center_y)/4)

        self.lock_p1 = (0, 0)
        self.lock_p2 = (0, 0)
        self.center_p1 = (self.center_x-self.maskSize,
                          self.center_y-self.maskSize)
        self.center_p2 = (self.center_x+self.maskSize,
                          self.center_y+self.maskSize)

        self.ros_setup()

        self.process_thread = threading.Thread(target=self.detection_process) 
        self.process_thread.start()

        rospy.spin()

        self.process_thread.join()

        

    def ros_setup(self):
        self.glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

        rospy.init_node('object_detection', anonymous=True)

        self.image_pubulisher_raw = rospy.Publisher('mask_image/image_raw',Image_msg,queue_size=1)
        self.image_pubulisher_compressed = rospy.Publisher('mask_image/image_raw/compressed',CompressedImage,queue_size=1)


        self.image_topic_name = rospy.get_param(
            "object_detection/image_topic", "/usb_cam/image_raw/compressed")

        if("compressed" in self.image_topic_name):
            rospy.Subscriber(self.image_topic_name,
                             CompressedImage, self.image_callback)
        else:
            rospy.Subscriber(self.image_topic_name,
                             Image_msg, self.image_callback)

        self.bridge = CvBridge()

    def publish_image(self, imgdata, height, width, time=None):
        image_raw=Image_msg()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'map'
        image_raw.height = height
        image_raw.width = width
        image_raw.encoding='rgb8'
        image_raw.data=np.array(imgdata).tostring()
        image_raw.header=header
        image_raw.step=1241*3
        self.image_pubulisher_raw.publish(image_raw)

        image_compressed = CompressedImage()
        image_compressed.header.stamp =rospy.Time.now()
        image_compressed.format = "jpeg"
        imgdata = cv2.cvtColor(imgdata, cv2.COLOR_BGR2RGB)

        image_compressed.data = np.array(cv2.imencode('.jpg', imgdata)[1]).tostring()
        self.image_pubulisher_compressed.publish(image_compressed)


    def image_callback(self, data):
        if("compressed" in self.image_topic_name):
            frame = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(
                data, desired_encoding='passthrough'), cv2.COLOR_BGR2RGB)
        else:
            frame = self.bridge.imgmsg_to_cv2(
                data, desired_encoding='passthrough')
        self.cv_image  = cv2.flip(frame, 1)
        # img = Image.fromarray(cv_image)
        


    def detection_process(self):
        time.sleep(3)
        while True:
            if(self.cv_image == []):
                continue
            frame = self.cv_image

            aim_p1 = (self.center_p1[0]+self.move_xy[0], self.center_p1[1]+self.move_xy[1])
            aim_p2 = (self.center_p2[0]+self.move_xy[0], self.center_p2[1]+self.move_xy[1])

            yolo_frame = mx.nd.array(cv2.cvtColor(
                frame, cv2.COLOR_BGR2RGB)).astype('uint8')
            rgb_nd, yolo_frame = gcv.data.transforms.presets.yolo.transform_test(
                yolo_frame, short=320)
            rgb_nd = rgb_nd.as_in_context(self.ctx)
            class_IDs, scores, bounding_boxs = self.detector(rgb_nd)
            if isinstance(bounding_boxs, mx.nd.NDArray):
                bounding_boxs = bounding_boxs[0].asnumpy()
                bounding_boxs *= self.rescale
            if isinstance(class_IDs, mx.nd.NDArray):
                class_IDs = class_IDs[0].asnumpy()
            if isinstance(scores, mx.nd.NDArray):
                scores = scores[0].asnumpy()

            # Show Bounding Box
            if self.class_mode == 'human':
                class_id = 0.
            elif self.class_mode == 'cat':
                class_id = 15.
            elif self.class_mode == 'dog':
                class_id = 16.
            detect_p1 = []
            detect_p2 = []
            for i in range(100):
                if class_IDs[i] == class_id and scores[i] > self.score_threshold:
                    p1 = (int(bounding_boxs[i, 0]), int(bounding_boxs[i, 1]))
                    p2 = (int(bounding_boxs[i, 2]), int(bounding_boxs[i, 3]))
                    detect_p1.append(p1)
                    detect_p2.append(p2)
                    cv2.rectangle(frame, p1, p2, (255, 0, 0), 2)
                    # print(p1)
            # print(self.is_lock)

            # track
            if self.is_lock:
                tracked = -1
                tracked_ratio = -1
                if len(detect_p1) != 0:
                    non_track = 0
                    for i in range(len(detect_p1)):
                        track_ratio, area_dif = self.interaction_ratio(
                            detect_p1[i], detect_p2[i], self.lock_p1, self.lock_p2)
                        if track_ratio > self.track_threshold:
                            if tracked == -1:
                                tracked = i
                                tracked_diff = area_dif
                            else:
                                if area_dif < tracked_diff:
                                    tracked = i
                                    tracked_diff = area_dif
                    if tracked == -1:
                        non_track += 1
                    else:
                        lock_p1 = detect_p1[tracked]
                        lock_p2 = detect_p2[tracked]
                        cv2.rectangle(frame, lock_p1, lock_p2, (0, 0, 255), 2)
                else:
                    non_track += 1
                if non_track > 3:
                    is_lock = False

            # lock
            if self.is_aim:
                if is_lock:
                    is_lock = False
                    lock_p1 = (0, 0)
                    lock_p2 = (0, 0)
                else:
                    locked = -1
                    locked_ratio = -1
                    for i in range(len(detect_p1)):
                        lock_ratio, _ = self.interaction_ratio(
                            detect_p1[i], detect_p2[i], aim_p1, aim_p2)
                        if lock_ratio > self.lock_threshold:
                            if locked == -1:
                                locked = i
                                locked_ratio = lock_ratio
                            else:
                                if lock_ratio > locked_ratio:
                                    locked = i
                                    locked_ratio = lock_ratio
                    # print(locked)
                    if locked == -1:
                        pass
                    else:
                        lock_p1 = detect_p1[locked]
                        lock_p2 = detect_p2[locked]
                        cv2.rectangle(frame, lock_p1, lock_p2, (0, 0, 255), 2)

            frame = self.lock_mask(frame, self.move_xy, self.is_aim)
            # cv2.imshow('frame', frame)

            # print((rospy.Time.now().nsecs-data.header.stamp.nsecs )*10**(-9))

            # self.publish_image(frame, frame.shape[0], frame.shape[1], data.header.stamp)
            self.publish_image(frame, frame.shape[0], frame.shape[1])




    def interaction_ratio(self, detect_1, detect_2, lock_1, lock_2):
        cx1, cy1 = detect_1
        cx2, cy2 = detect_2
        gx1, gy1 = lock_1
        gx2, gy2 = lock_2
        x1 = max(cx1, gx1)
        y1 = max(cy1, gy1)
        x2 = min(cx2, gx2)
        y2 = min(cy2, gy2)
        w = max(0, x2 - x1)
        h = max(0, y2 - y1)
        iou = h*w
        detect_area = (cx2-cx1)*(cy2-cy1)
        lock_area = (gx2-gx1)*(gy2-gy1)
        lock_ratio = iou/lock_area
        area_dif = abs(lock_area-detect_area)
        return lock_ratio, area_dif

    def shot_mask(self, img, color):

        # center_x = int(img.shape[1]/2)
        # center_y = int(img.shape[0]/2)
        # maskSize = int(min(center_x,center_y)/4)
        img = cv2.line(img, (self.center_x-self.maskSize, self.center_y),
                       (self.center_x-int(self.maskSize/2), self.center_y),
                       color, 2)
        # right
        img = cv2.line(img, (self.center_x+self.maskSize, self.center_y),
                       (self.center_x+int(self.maskSize/2), self.center_y),
                       color, 2)
        # up
        img = cv2.line(img, (self.center_x, self.center_y-self.maskSize),
                       (self.center_x, self.center_y-int(self.maskSize/2)),
                       color, 2)
        # button
        img = cv2.line(img, (self.center_x, self.center_y+self.maskSize),
                       (self.center_x, self.center_y+int(self.maskSize/2)),
                       color, 2)
        # center circle
        img = cv2.ellipse(img, (self.center_x, self.center_y),
                          (int(self.maskSize/2), int(self.maskSize/2)),
                          0, 0, 180,
                          color, 2)
        # arrow
        points = np.array([[self.center_x, self.center_y],
                           [self.center_x+int(self.maskSize/5),
                            self.center_y-int(self.maskSize/3)],
                           [self.center_x-int(self.maskSize/5), self.center_y-int(self.maskSize/3)]], np.int32)
        points = points.reshape((-1, 1, 2))
        cv2.polylines(img, [points], True, color, 2)
        return img

    def aim_mask(self, img, move_xy, color):
        # center_x = int(img.shape[1]/2)
        # center_y = int(img.shape[0]/2)
        # maskSize = int(min(center_x,center_y)/4)
        blk = np.zeros(img.shape, np.uint8)

        if move_xy == (0, 0):
            mask = cv2.rectangle(blk, (self.center_x-self.maskSize, self.center_y-self.maskSize),
                                 (self.center_x+self.maskSize,
                                  self.center_y+self.maskSize),
                                 color, -1)
        else:
            mask = cv2.rectangle(blk, (self.center_x-self.maskSize+move_xy[0], self.center_y-self.maskSize+move_xy[1]),
                                 (self.center_x+self.maskSize +
                                  move_xy[0], self.center_y+self.maskSize+move_xy[1]),
                                 color, -1)
        combine = cv2.addWeighted(img, 1, mask, 0.25, 1.0)
        return combine

    def lock_mask(self, img, move_xy, is_aim):
        if is_aim:
            color = (50, 50, 255)
        else:
            color = (50, 255, 50)

        img = self.shot_mask(img, color)
        img = self.aim_mask(img, move_xy, color)
        return img


# while cap.isOpened():



#     if cv2.waitKey(1) == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()

if __name__ == '__main__':  
    try:
        ob = ObjectDetection()
    except rospy.ROSInterruptException:
        pass
