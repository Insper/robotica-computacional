#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import biblioteca_cow

cap = cv2.VideoCapture('cow_wolf/cow_wolf.mp4')

# Classes da MobileNet
CLASSES = [None]

while(True):
    # Capture frame-by-frame
    ret, img = cap.read()
    
    if ret == False:
        print("Codigo de retorno FALSO - problema para capturar o frame")
        break
    else:
        ## Desenvolva o Codigo Aqui
        
        cv2.imshow('img',img)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
