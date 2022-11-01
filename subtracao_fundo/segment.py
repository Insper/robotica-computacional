import cv2
import cv2.cv as cv
import numpy as np
from matplotlib import pyplot as plt
import time

cap = cv2.VideoCapture('hall_robots.mp4')
#cap = cv2.VideoCapture(0)
#cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1


def auto_canny(image, sigma=0.33):
    # compute the median of the single channel pixel intensities
    v = np.median(image)

    # apply automatic Canny edge detection using the computed median
    global lower
    lower = int(max(0, (1.0 - sigma) * v))
    global upper
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)

    # return the edged image
    return edged



while(True):

    ret, frame = cap.read()
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    rgb =  cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = frame


    cv2.line(bordas_color,(0,0),(511,511),(255,0,0),5)

    # cv2.rectangle(img, pt1, pt2, color[, thickness[, lineType[, shift]]])
    cv2.rectangle(bordas_color,(384,0),(510,128),(0,255,0),3)

    # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(bordas_color,'Ninjutsu ;)',(0,50), font, 2,(255,255,255),2,cv2.CV_AA)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Display the resulting frame
    cv2.imshow('frame',bordas_color)
    print("No circles were found")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break    
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
