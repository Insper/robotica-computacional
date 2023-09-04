#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import os
import biblioteca_cow

class YOLOModule():
    def __init__(self, path):
        self.INPUT_WIDTH = 640
        self.INPUT_HEIGHT = 640
        self.SCORE_THRESHOLD = 0.2
        self.NMS_THRESHOLD = 0.4
        self.CONFIDENCE_THRESHOLD = 0.4
        self.colors = [(255, 255, 0), (0, 255, 0), (0, 255, 255), (255, 0, 0)]

        self.class_list = []
        with open("config/classes.txt", "r") as f:
            self.class_list = [cname.strip() for cname in f.readlines()]
        
        self.model = self.build_model(path)
    
    def build_model(self, path):
        model = cv2.dnn.readNet(path)
        model.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        model.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        
        return model

    def format_yolov5(self,frame):
        row, col, _ = frame.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = frame
        return result

    def detect(self, image):
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (self.INPUT_WIDTH, self.INPUT_HEIGHT), swapRB=True, crop=False)
        self.model.setInput(blob)
        preds = self.model.forward()

        return preds
    
    def wrap_detection(self,input_image, output_data):
        class_ids = []
        confidences = []
        boxes = []

        rows = output_data.shape[0]

        image_width, image_height, _ = input_image.shape

        x_factor = image_width / self.INPUT_WIDTH
        y_factor =  image_height / self.INPUT_HEIGHT

        for r in range(rows):
            row = output_data[r]
            confidence = row[4]
            if confidence >= 0.4:

                classes_scores = row[5:]
                _, _, _, max_indx = cv2.minMaxLoc(classes_scores)
                class_id = max_indx[1]
                if (classes_scores[class_id] > .25):

                    confidences.append(confidence)

                    class_ids.append(class_id)

                    x, y, w, h = row[0].item(), row[1].item(), row[2].item(), row[3].item() 
                    left = int((x - 0.5 * w) * x_factor)
                    top = int((y - 0.5 * h) * y_factor)
                    width = int(w * x_factor)
                    height = int(h * y_factor)
                    box = np.array([left, top, width, height])
                    boxes.append(box)

        boxes = [box.tolist() for box in boxes]
        confidences = [float(conf) for conf in confidences]

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.25, 0.45) 

        result_class_ids = []
        result_confidences = []
        result_boxes = []

        for i in indexes:
            i = i[0]
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes
    
    def draw_detections(self, image, class_ids, confidences, boxes):
        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = self.colors[int(classid) % len(self.colors)]
            cv2.rectangle(image, box, color, 2)
            cv2.rectangle(image, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            cv2.putText(image, self.class_list[classid], (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        
        return image

def main():
    import time
    bgr = cv2.imread("img/cow_wolf03.png")

    start = time.perf_counter()
    Neural = YOLOModule("config/yolov5s.onnx")
    inputImage = Neural.format_yolov5(bgr)
    outs = Neural.detect(inputImage)

    class_ids, confidences, boxes = Neural.wrap_detection(inputImage, outs[0])
    result_yolo = Neural.draw_detections(bgr.copy(), class_ids, confidences, boxes)
    print("YOLO: ", time.perf_counter() - start)


    start = time.perf_counter()
    model = biblioteca_cow.load_mobilenet()
    result_mob, out = biblioteca_cow.detect(bgr, model)
    print("MobileNet: ", time.perf_counter() - start)

    cv2.imshow("Result_YOLO", result_yolo)
    cv2.imshow("Result_MobileNet", result_mob)
    cv2.waitKey(0)



    

if __name__ == "__main__":
    main()