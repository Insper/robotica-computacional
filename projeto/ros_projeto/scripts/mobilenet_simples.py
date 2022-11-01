#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import division, print_function

# import the necessary packages
import numpy as np
import argparse
import cv2

import rospkg
import os


rospack = rospkg.RosPack()
path = rospack.get_path('ros_projeto')
scripts = os.path.join(path,  "scripts")

proto = os.path.join(scripts,"MobileNetSSD_deploy.prototxt.txt")
model = os.path.join(scripts, "MobileNetSSD_deploy.caffemodel")
confianca = 0.2


# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
	"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
	"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
	"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
# print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe(proto, model)

# load the input image and construct an input blob for the image
# by resizing to a fixed 300x300 pixels and then normalizing it
# (note: normalization is done via the authors of the MobileNet SSD
# implementation)


def detect(frame):
    image = frame.copy()
    (h, w) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)

    # pass the blob through the network and obtain the detections and
    # predictions
    # print("[INFO] computing object detections...")
    net.setInput(blob)
    detections = net.forward()

    results = []

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with the
        # prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence


        if confidence > confianca:
            # extract the index of the class label from the `detections`,
            # then compute the (x, y)-coordinates of the bounding box for
            # the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # display the prediction
            label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
            #print("[INFO] {}".format(label))
            cv2.rectangle(image, (startX, startY), (endX, endY),
                COLORS[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(image, label, (startX, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

            results.append((CLASSES[idx], confidence*100, (startX, startY),(endX, endY) ))

    # show the output image
    return image, results






import cv2

if __name__ == "__main__":

    #cap = cv2.VideoCapture('hall_box_battery_1024.mp4')
    cap = cv2.VideoCapture(0)

    print("Known classes")
    print(CLASSES)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        result_frame, result_tuples = detect(frame)


        # Display the resulting frame
        cv2.imshow('frame',result_frame)

        # Prints the structures results:
        # Format:
        # ("CLASS", confidence, (x1, y1, x2, y3))
        for t in result_tuples:
            # print(t) # Descomente para imprimir
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
