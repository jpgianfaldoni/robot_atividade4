#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
# from geometry_msgs.msg import Twist, Vector3, Pose
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge, CvBridgeError
# import smach
# import smach_ros



def identifica_cor(frame, low = [36, 0, 0], high = [70, 255, 255]):
    def auto_canny(image, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(image)

        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        # return the edged image
        return edged

    def cross(img_rgb, point, color, width,length):
        cv2.line(img_rgb, (point[0] - length/2, point[1]),  (point[0] + length/2, point[1]), color ,width, length)
        cv2.line(img_rgb, (point[0], point[1] - length/2), (point[0], point[1] + length/2),color ,width, length) 



    frame = cv2.GaussianBlur(frame, (5,5), cv2.BORDER_DEFAULT)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cor_menor = np.array(low)
    cor_maior = np.array(high)
    segmentado_cor = cv2.inRange(frame_hsv, cor_menor, cor_maior)
    frame_masked = cv2.morphologyEx(segmentado_cor, cv2.MORPH_CLOSE,np.ones((10, 10)))
    contornosMask, arvore = cv2.findContours(frame_masked.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    centro = (frame.shape[1]//2, frame.shape[0]//2)

    areas = [cv2.contourArea(x) for x in contornosMask]
    maior_contorno = None
    maior_contorno_area = 0
    for i,e in enumerate(areas):
        if e > maior_contorno_area:
            maior_contorno = contornosMask[i]
            maior_contorno_area = e

    if not maior_contorno is None :
        cv2.drawContours(frame, [maior_contorno], -1, [0, 0, 255], 5)
        maior_contorno = np.reshape(maior_contorno, (maior_contorno.shape[0], 2))
        media = maior_contorno.mean(axis=0)
        media = media.astype(np.int32)
        cv2.circle(frame, (media[0], media[1]), 5, [0, 255, 0])
        cross(frame, centro, [255,0,0], 1, 17)
        #print("Fazendo contornos")
    else:
        media = (0, 0)

    # Representa a area e o centro do maior contorno no frame
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    cv2.putText(frame,"{:d} {:d}".format(*media),(20,100), 1, 4,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(frame,"{:0.1f}".format(maior_contorno_area),(20,50), 1, 4,(255,255,255),2,cv2.LINE_AA)

   # cv2.imshow('video', frame)
    cv2.imshow('frame', frame)
    cv2.waitKey(1)

    return media, centro, maior_contorno_area
