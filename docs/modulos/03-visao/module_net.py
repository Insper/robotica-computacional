#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import os

class YOLODector():
    # REF: https://github.com/doleron/yolov5-opencv-cpp-python/blob/main/config_files/yolov5s.onnx
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
            #i = i[0]
            result_confidences.append(confidences[i])
            result_class_ids.append(class_ids[i])
            result_boxes.append(boxes[i])

        return result_class_ids, result_confidences, result_boxes
    
    def draw_detections(self, image, class_ids, confidences, boxes):
        for (classid, confidence, box) in zip(class_ids, confidences, boxes):
            color = self.colors[int(classid) % len(self.colors)]
            cv2.rectangle(image, box, color, 2)
            cv2.rectangle(image, (box[0], box[1] - 20), (box[0] + box[2], box[1]), color, -1)
            cv2.putText(image, f'{self.class_list[classid]} - {confidence:.2f}', (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,0))
        
        return image

class MobileNetDetector():
    def __init__(self):
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        self.CONFIDENCE = 0.7
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))
        self.args_prototxt = "./mobilenet_detection/MobileNetSSD_deploy.prototxt.txt"
        self.args_model = "./mobilenet_detection/MobileNetSSD_deploy.caffemodel"
        self.net = self.load_mobilenet()

    def load_mobilenet(self):
        net = cv2.dnn.readNetFromCaffe(self.args_prototxt, self.args_model)
        return net

    def detect(self, frame):
        image = frame.copy()
        (h, w) = image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()
        results = []
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.CONFIDENCE:
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
                cv2.rectangle(image, (startX, startY), (endX, endY), self.COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)
                results.append((self.CLASSES[idx], confidence*100, (startX, startY), (endX, endY)))
        return image, results

    def separar_caixa_entre_animais(self, img, resultados):
        img = img.copy()
        animais = {'vaca': [], 'lobo': []}
        for i in range(len(resultados)):
            if resultados[i][0] == 'cow':
                cv2.rectangle(img, (resultados[i][2][0], resultados[i][2][1]), (resultados[i][3][0], resultados[i][3][1]), [255,0,0], 2)
                animais['vaca'].append([resultados[i][2][0], resultados[i][2][1], resultados[i][3][0], resultados[i][3][1]])
            else:
                animais['lobo'].append([resultados[i][2][0], resultados[i][2][1], resultados[i][3][0], resultados[i][3][1]])

        retang_lobo = []
        if len(animais['lobo']) == 2:
            if animais['lobo'][0][0] > animais['lobo'][1][0]: #pegando o menor xmin
                retang_lobo.append(animais['lobo'][1][0])
            else:
                retang_lobo.append(animais['lobo'][0][0])

            if animais['lobo'][0][1] > animais['lobo'][1][1]: #pegando o menor ymin
                retang_lobo.append(animais['lobo'][1][1])
            else:
                retang_lobo.append(animais['lobo'][0][1])

            if animais['lobo'][0][2] > animais['lobo'][1][2]: #pegando o maior xmax
                retang_lobo.append(animais['lobo'][0][2])
            else:
                retang_lobo.append(animais['lobo'][1][2])

            if animais['lobo'][0][3] > animais['lobo'][1][3]: #pegando o maior ymax
                retang_lobo.append(animais['lobo'][0][3])
            else:
                retang_lobo.append(animais['lobo'][1][3])
        
            animais['lobo'] = retang_lobo
            cv2.rectangle(img, (retang_lobo[0], retang_lobo[1]),(retang_lobo[2], retang_lobo[3]), [0,0,255], 2)
        else:
            cv2.rectangle(img, (animais['lobo'][0][0], animais['lobo'][0][1]), (animais['lobo'][0][2], animais['lobo'][0][3]), [0,0,255], 2)

        return img, animais

    def calcula_iou(self, boxA, boxB):
        """Não mude ou renomeie esta função
            Calcula o valor do "Intersection over Union" para saber se as caixa se encontram
        """

        xA = min(boxA[0], boxB[0])
        yA = min(boxA[1], boxB[1])
        xB = max(boxA[2], boxB[2])
        yB = max(boxA[3], boxB[3])

        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = interArea / float(boxAArea + boxBArea - interArea)

        return iou

    def checar_perigo(self, image, animais):
        """Não mude ou renomeie esta função
            Recebe as coordenadas das caixas, se a caixa de uma vaca tem intersecção com as do lobo, ela esta em perigo.
            Se estiver em perigo, deve escrever na imagem com a cor vermlha, se não, escreva com a cor azul.
            *Importante*: nesta função, não faça cópia da imagem de entrada!!
            
            Repita para cada vaca na imagem.
        """

        for i in animais['vaca']:
            iou = self.calcula_iou(animais['lobo'][0], i)
            if iou > 0.5:
                image = cv2.putText(image,"Esta em perigo",(i[0], i[1]),cv2.FONT_HERSHEY_SIMPLEX,1.5,[0,0,255],3)
            else:
                image = cv2.putText(image,'Esta sem perigo', (i[0], i[1]), cv2.FONT_HERSHEY_SIMPLEX, 1.5, [255, 0, 0], 3)
            
        return image

def main():
    import time
    bgr = cv2.imread("img/cow_wolf03.png")

    start = time.perf_counter()
    YOLO = YOLODector("config/yolov5s.onnx")
    inputImage = YOLO.format_yolov5(bgr)
    outs = YOLO.detect(inputImage)

    class_ids, confidences, boxes = YOLO.wrap_detection(inputImage, outs[0])
    result_yolo = YOLO.draw_detections(bgr.copy(), class_ids, confidences, boxes)
    print("YOLO: ", time.perf_counter() - start)


    start = time.perf_counter()
    MOBILE = MobileNetDetector()
    result_mob, out = MOBILE.detect(bgr)
    print("MobileNet: ", time.perf_counter() - start)

    cv2.imshow("Result_YOLO", result_yolo)
    cv2.imshow("Result_MobileNet", result_mob)
    cv2.waitKey(0)



    

if __name__ == "__main__":
    main()