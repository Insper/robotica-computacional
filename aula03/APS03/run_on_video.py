#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import biblioteca

print("Baixe o arquivo a seguir para funcionar: ")
print("https://github.com/Insper/robot202/raw/master/projeto/centro_massa/video.mp4")

cap = cv2.VideoCapture('yellow.mp4')

while(True):
    # Capture frame-by-frame
    ret, img = cap.read()
    # frame = cv2.imread("frame0000.jpg")
    # ret = True
    
    if ret == False:
        print("Codigo de retorno FALSO - problema para capturar o frame")
        break
    else:
        mask = img.copy()

        # Imagem original
        cv2.imshow('img',img)
        # Mascara
        cv2.imshow('mask',mask)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

