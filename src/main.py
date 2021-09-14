#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal

from pid import *
from movingAverage import *
from imageProcessing import *

### 전역변수 선언
global pub
global imageProcessing
global bridge

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

# 카메라로부터 Image형 토픽이 오면 전처리 시킬 수 있는 이미지로 변환 작업
def img_callback(data):
    global imageProcessing
    global bridge

    image = bridge.imgmsg_to_cv2(data, "bgr8")
    
    while not image.size == (640*480*3):
        continue
    
    imageProcessing.setImage(image)

# 각도와 속도를 모터 노드로 publish
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

def init():
    global pub
    global imageProcessing
    global bridge

    signal.signal(signal.SIGINT, signal_handler)

    imageProcessing = ImageProcessing()

    bridge = CvBridge()
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.sleep(2)

def start():
    global imageProcessing

    # speed 15 : 0.4, 0.0005, 0.25 -> stable
    # speed 25 : 0.3, 0.0, 0.3 -> almost stable
    pid_c = PID(0.5, 0.0, 0.0)			# 곡선(curve)에서의 PID
    pid_s = PID(0.25, 0.0005, 0.3)		# 직선(straight)에서의 PID

    while True:
        lpos, rpos = imageProcessing.process_image(imageProcessing.calibrate_image())

        center = (lpos + rpos) / 2
        error = -(imageProcessing.getWidth()/2 - center - 10)		# 직진 주행을 할 때에 중앙선보다 왼쪽에서 주행하여 임의로 +10

        if lpos == 0.0 or rpos > 630.0:			# 곡선(한쪽 차선을 검출 못하거나 오른쪽 차선이 너무 오른쪽에 있는 경우 곡선으로 가정)
            angle = pid_c.pid_control(error)		# angle = error/2
            drive(angle, 22)
        else:						# 직선
            angle = pid_s.pid_control(error)
            drive(angle, 28)

        # 키보드에서 q키를 누른 경우 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()


if __name__ == '__main__':
    init()
    start()
