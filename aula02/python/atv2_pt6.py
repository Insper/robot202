#!/usr/bin/python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
from matplotlib import pyplot as plt
import auxiliar as aux
import time

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

    MIN_MATCH_COUNT = 10

    cena_bgr = cv2.imread("../Insper_Logo.png") # Imagem do cenario
    original_bgr = frame

    # Versões RGB das imagens, para plot
    original_rgb = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2RGB)
    cena_rgb = cv2.cvtColor(cena_bgr, cv2.COLOR_BGR2RGB)

    # Versões grayscale para feature matching
    img_original = cv2.cvtColor(original_bgr, cv2.COLOR_BGR2GRAY)
    img_cena = cv2.cvtColor(cena_bgr, cv2.COLOR_BGR2GRAY)

    # Cria o detector BRISK
    brisk = cv2.BRISK_create()

    # Encontra os pontos únicos (keypoints) nas duas imagems
    kp1, des1 = brisk.detectAndCompute(img_original ,None)
    kp2, des2 = brisk.detectAndCompute(img_cena,None)

    # Configura o algoritmo de casamento de features que vê *como* o objeto que deve ser encontrado aparece na imagem
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)


    # Tenta fazer a melhor comparacao usando o algoritmo
    matches = bf.knnMatch(des1,des2,k=2)

    # store all the good matches as per Lowe's ratio test.
    good = []
    for m,n in matches:
        if m.distance < 0.7*n.distance:
            good.append(m)


    if len(good)>MIN_MATCH_COUNT:
        # Separa os bons matches na origem e no destino
        print("Matches found")
    else:
        print("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))


    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,'Press q to quit',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)

    #More drawing functions @ http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html

    # Display the resulting frame
    cv2.imshow('Detector de logo', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#  When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
