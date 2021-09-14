#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, random, math
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal

from pid import *
from movingAverage import *
from imageProcessing import *

### 전역변수 선언
global imageProcessing
global cap

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

def init():
    global imageProcessing
    global cap
    
    signal.signal(signal.SIGINT, signal_handler)

    imageProcessing = ImageProcessing()
    cap = cv2.VideoCapture('track1.avi')

def start():
    global imageProcessing
    global cap

    _, frame = cap.read()
    
    while not frame.size == (imageProcessing.getWidth()*imageProcessing.getHeight()*3):
        _, frame = cap.read()
        continue
        
    while cap.isOpened():
        _, frame = cap.read()
        lpos, rpos = imageProcessing.process_image(frame)

        # 키보드에서 q키를 누른 경우 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()


if __name__ == '__main__':
    init()
    start()
