#!/usr/bin/env python3

# USAGE
# python detect_mask_video.py

# import the necessary packages
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
import numpy as np
import argparse
import imutils
import time
import cv2
import os

import rospy
from std_msgs.msg import String, Float64, Header, Bool
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image as Image_msg
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


def detect_and_predict_mask(frame, faceNet, maskNet):
    # grab the dimensions of the frame and then construct a blob
    # from it
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300),
                                 (104.0, 177.0, 123.0))

    # pass the blob through the network and obtain the face detections
    faceNet.setInput(blob)
    detections = faceNet.forward()

    # initialize our list of faces, their corresponding locations,
    # and the list of predictions from our face mask network
    faces = []
    locs = []
    preds = []

    # loop over the detections
    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the confidence is
        # greater than the minimum confidence
        if confidence > 0.5:
            # compute the (x, y)-coordinates of the bounding box for
            # the object
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # ensure the bounding boxes fall within the dimensions of
            # the frame
            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))

            # extract the face ROI, convert it from BGR to RGB channel
            # ordering, resize it to 224x224, and preprocess it
            face = frame[startY:endY, startX:endX]
            face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
            face = cv2.resize(face, (224, 224))
            face = img_to_array(face)
            face = preprocess_input(face)

            # add the face and bounding boxes to their respective
            # lists
            faces.append(face)
            locs.append((startX, startY, endX, endY))

    # only make a predictions if at least one face was detected
    if len(faces) > 0:
        # for faster inference we'll make batch predictions on *all*
        # faces at the same time rather than one-by-one predictions
        # in the above `for` loop
        faces = np.array(faces, dtype="float32")
        preds = maskNet.predict(faces, batch_size=32)

    # return a 2-tuple of the face locations and their corresponding
    # locations
    return (locs, preds)




# load our serialized face detector model from disk
config_path = rospy.get_param('detect_mask_py/config_path')
print("[INFO] loading face detector model...")
prototxtPath = os.path.sep.join([config_path,'face_detector', "deploy.prototxt"])
weightsPath = os.path.sep.join([config_path,'face_detector',
	"res10_300x300_ssd_iter_140000.caffemodel"])
print(prototxtPath)
faceNet = cv2.dnn.readNet(prototxtPath, weightsPath)

print("[INFO] loading face mask detector model...")
maskNet = load_model(config_path+'/mask_detector.model')


# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting ROS video stream...")

cv_image = np.zeros(shape=[512, 512, 3], dtype=np.uint8)


def image_callback(data):
    global cv_image
    if("compressed" in image_topic_name):
        frame = bridge.compressed_imgmsg_to_cv2(
            data, desired_encoding='passthrough')
    else:
        frame = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    cv_image = cv2.flip(frame, 1)


glz_name = rospy.get_param('GLZ_NAME', 'GLZ01')

rospy.init_node('mask_detection', anonymous=True)

image_pubulisher_raw = rospy.Publisher(
    'mask_detection_image/image_raw', Image_msg, queue_size=1)
image_pubulisher_compressed = rospy.Publisher(
    'mask_detection_image/image_raw/compressed', CompressedImage, queue_size=1)


lock_pos_pubulisher = rospy.Publisher(
    'object_detection/lock_pos', Point, queue_size=1)
image_topic_name = rospy.get_param(
    "mask_detection/image_topic", "/usb_cam1/image_raw/compressed")

if("compressed" in image_topic_name):
    rospy.Subscriber(image_topic_name, CompressedImage, image_callback)
else:
    rospy.Subscriber(image_topic_name, Image_msg, image_callback)

bridge = CvBridge()

time.sleep(2.0)


def publish_image(imgdata, height, width, time=None):
    imgdata = cv2.flip(imgdata, 1)
    image_raw = Image_msg()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_raw.height = height
    image_raw.width = width
    image_raw.encoding = 'rgb8'
    image_raw.data = np.array(imgdata).tostring()
    image_raw.header = header
    image_raw.step = 1241*3
    image_pubulisher_raw.publish(image_raw)
    

    image_compressed = CompressedImage()
    image_compressed.header.stamp = rospy.Time.now()
    image_compressed.format = "jpeg"
    # imgdata = cv2.cvtColor(imgdata, cv2.COLOR_BGR2RGB)

    image_compressed.data = np.array(
        cv2.imencode('.jpg', imgdata)[1]).tostring()
    image_pubulisher_compressed.publish(image_compressed)


# loop over the frames from the video stream
while True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = cv_image
    frame = imutils.resize(frame, width=400)

    # detect faces in the frame and determine if they are wearing a
    # face mask or not
    (locs, preds) = detect_and_predict_mask(frame, faceNet, maskNet)

    # loop over the detected face locations and their corresponding
    # locations
    for (box, pred) in zip(locs, preds):
        # unpack the bounding box and predictions
        (startX, startY, endX, endY) = box
        (mask, withoutMask) = pred

        # determine the class label and color we'll use to draw
        # the bounding box and text
        label = "Mask" if mask > withoutMask else "No Mask"
        color = (0, 255, 0) if label == "Mask" else (0, 0, 255)

        # include the probability in the label
        label = "{}: {:.2f}%".format(label, max(mask, withoutMask) * 100)

        # display the label and bounding box rectangle on the output
        # frame
        cv2.putText(frame, label, (startX, startY - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
        cv2.rectangle(frame, (startX, startY), (endX, endY), color, 2)

    publish_image(frame, frame.shape[0], frame.shape[1])
