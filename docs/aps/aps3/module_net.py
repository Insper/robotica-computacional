#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import os


class MobileNetDetector():
    """Classe para detecção de objetos com o modelo MobileNetSSD.
    """
    def __init__(self,
                 CONFIDENCE = 0.7,
                 args_prototxt = "./config/MobileNetSSD_deploy.prototxt.txt",
                 args_model = "./config/MobileNetSSD_deploy.caffemodel"
                 ):
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        self.CONFIDENCE = CONFIDENCE
        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        self.args_prototxt = args_prototxt
        self.args_model = args_model
        self.net = self.load_mobilenet()

    def load_mobilenet(self):
        """Carrega o modelo MobileNetSSD.
        Certifique-se de que os arquivos .prototxt.txt e .caffemodel diretório correto.

        Returns:
            net: modelo carregado
        """
        net = cv2.dnn.readNetFromCaffe(self.args_prototxt, self.args_model)
        return net

    def detect(self, frame: np.ndarray):
        """Detecta objetos na imagem de entrada.
        Filtra as detecções com uma confiança menor que self.CONFIDENCE.

        Args:
            frame (np.ndarray): Imagem de entrada

        Returns:
            image: Imagem de saída com as detecções desenhadas
            results: Lista com as detecções no formato [classe, confiança, (xmin, ymin), (xmax, ymax)]
        """
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


def rodar_frame():
    import time
    bgr = cv2.imread("img/teste1.png")

    start = time.perf_counter()
    MOBILE = MobileNetDetector()
    bgr, out = MOBILE.detect(bgr)
    print(out)
    print("MobileNet: ", time.perf_counter() - start)

    cv2.imshow("Result_MobileNet", bgr)
    cv2.waitKey(0)


def rodar_video():
    MOBILE = MobileNetDetector()

    cap = cv2.VideoCapture('img/dogs.wmv') # Confira se o video esta na pasta img

    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret == True:
            frame, out = MOBILE.detect(frame)
            print(len(out))

            cv2.imshow('Frame',frame)

            if cv2.waitKey(1) & 0xFF == ord('q'): # !!! Pressione q para sair
                break
        else:
            break
            
def main():
    # Selecione se deseja rodar seu codigo com uma imagem ou um video:

    # rodar_frame()
    rodar_video()

if __name__ == "__main__":
    main()