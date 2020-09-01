#!/usr/bin/python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
from matplotlib import pyplot as plt
import auxiliar as aux
import time

cor = input("ESCOLHA UMA COR (m OU c PARA MAGENTA OU CIANO):\n>>>> ")

# If you want to open a video, just change v2.VideoCapture(0) from 0 to the filename, just like below
#cap = cv2.VideoCapture('hall_box_battery.mp4')

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

lower = 0
upper = 1

print("Press q to QUIT")

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert the frame to rgb
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    if cor == 'm':
        hsv1, hsv2 = aux.ranges('#FF00FF')
    elif cor == 'c':
        hsv1, hsv2 = aux.ranges('#00FFFF')

    mask = cv2.inRange(img_hsv, hsv1, hsv2)

    selecao = cv2.bitwise_and(img_rgb, img_rgb, mask=mask)
    segmentado_cor = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((10, 10)))
    selecao = cv2.bitwise_and(img_rgb, img_rgb, mask=segmentado_cor)

    # cv2.putText(img, text, org, fontFace, fontScale, color[, thickness[, lineType[, bottomLeftOrigin]]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(selecao,'Press q to quit',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Display the resulting frame
    cv2.imshow('Detector de circulos',selecao)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#  When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
