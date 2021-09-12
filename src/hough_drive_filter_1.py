#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg, time
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge, CvBridgeError
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal
import pid
import movingAverage

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40
temp = 0
pre_lpos = 100
pre_rpos = 470

def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0: #수직선이면
            slope = 0 #기울기 = 0
        else:
            slope = float(y2-y1) / float(x2-x1) #기울기 = y의 변화량 / x의 변화량 기울기 절대값이 10 이하인 것만 추출
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold: #기울기 절대값이 10 이하인 것만 추출
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90): #화면의 왼쪽(x좌표가 0<x<230)에 있는 선분 중에서 기울기가 음수인 것들만 모음
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90): #화면의 오른쪽(x좌표가 410<x<640)에 있는 선분 중에서 기울기가 양수인 것들만 모음
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines, 기울기와 y절편의 평균값
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0: #구해진 선분이 없으면
        return 0, 0 

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2 #모든 선분의 x 좌표 합
        y_sum += y1 + y2 #모든 선분의 y 좌표 합
        m_sum += float(y2 - y1) / float(x2 - x1) #모든 선분의 기울기 합

    x_avg = x_sum / (size * 2)   #모든 선분의 x좌표 평균값
    y_avg = y_sum / (size * 2)   #모든 선분의 y좌표 평균값
    m = m_sum / size    #모든 선분의 기울기 평균값
    b = y_avg - m * x_avg  #b = y - mx, y절편

    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap
    global pre_lpos, pre_rpos

    m, b = get_line_params(lines)

    if m == 0 and b == 0: #선분을 못찾았을 때
        if left:
            pos = pre_lpos
        if right:
            pos = pre_rpos
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)  #640x480 이미지의 맨 아래(y값 480)의 x1과 이미지 중간(y값 240)dml x2를 구해 (x1, 480)과 (x2, 320) -> (x2, 240)아닌가? 하튼 두 점을 잇는 파란색 선을 그린다.
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    if left:
        pre_lpos = pos
    if right:
        pre_rpos = pos

    return img, int(pos)

calibrated = True
if calibrated:
    # 자이카 카메라의 Calibration 보정값
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    # 위에서 구한 보정 행렬값을 적용하여 이미지를 반듯하게 수정하는 함수(undistort() 호출해서 이미지 수정)
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))
    
# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap
    global pre_lpos, pre_rpos

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # roi
    roi = gray[Offset : Offset+Gap, 0 : Width]

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(roi,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    all_lines = cv2.HoughLinesP(edge_img,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return pre_lpos, pre_rpos, frame

    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)

    return lpos, rpos, frame

def getSpeed(error):
    global Speed, temp

    Speed = 27
    error = -error

    # corner
    if abs(error) >= 18:
        Speed = 25
        pgain, igain, dgain = 0.5, 0.000, 0.000
    else:
        pgain, igain, dgain = 0.48, 0.00, 0.0
        
    usepid = pid.PID(pgain, igain, dgain)
    angle = usepid.pid_control(error)

    if abs(error) < 20:
        angle = angle/3

    return angle, Speed, pgain, igain, dgain

def start():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    while not rospy.is_shutdown():
        while not image.size == (640*480*3):
            continue
            
        image = calibrate_image(image)
        lpos, rpos, image = process_image(image)

        center = (lpos + rpos) / 2
        error = Width/2 - center

        pid_angle, Speed, pgain, igain, dgain = getSpeed(error)
        
        #pid먼저하고 filter
        filtererror.add_sample(pid_angle)
        #use_m = movingAverage.MovingAverage(10)
        filter_angle = filtererror.get_mm()

	#print(center, error, angle)
        drive(filter_angle, Speed)
        print('filter_angle : ', filter_angle)

        # show image
        text = 'angle : ' + str(angle)
        cv2.putText(image, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('calibration', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    start()