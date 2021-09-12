#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from yolo_steering import YOLO

import sys
import os
import signal


# PID 객체 생성
class PID():

    def __init__(self, kp, ki, kd):

        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):

        self.d_error = cte - self.p_error	# (현재 시간 error) - (이전 시간 error)
        self.p_error = cte
        if abs(self.i_error + cte) < 2000:	# 시간이 지날수록 계속 누적되면 i_error가 p나 d에 비해 너무 커지기에 상한선 설정
            self.i_error += cte			# ki = 0.0005인 경우, 0.0005 x 2000 = 1. 적분 부분이 1도 이상 영향 주지 않게끔 설정

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 305		# ROI 설정 시 쓰이는 값. Offset ~ Offset+Gap 까지
Gap = 40


calibrated = True
if calibrated:
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
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))


# 카메라로부터 Image형 토픽이 오면 전처리 시킬 수 있는 이미지로 변환 작업
def img_callback(data):
    global image    
    image = bridge.imgmsg_to_cv2(data, "bgr8")


# 각도와 속도를 모터 노드로 publish
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


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    # 왼쪽 차선
    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    # 오른쪽 차선
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    # 두 차선의 중앙 지점
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)    
    # 화면의 중앙
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img


# 검출한 모든 선들을 왼쪽 차선과 오른쪽 차선으로 분리
def divide_left_right(lines):

    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines


# 선들의 x, y좌표들의 평균으로 기울기와 y절편 구하는 함수
def get_line_params(lines):

    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    # line 하나당 x1, x2로 x좌표가 2개씩이므로 size*2로 나눈다.
    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b


# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0		# 왼쪽 차선이 검출되지 않으면 lpos = 0
        if right:
            pos = Width		# 오른쪽 차선이 검출되지 않으면 rpos = 640
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)


# 이미지 전처리, 차선과 차선의 좌표 검출 및 반환
def process_image(frame):
    global Width
    global Offset, Gap

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
        return 0, 640
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

    # show image
    cv2.imshow('calibration', frame)

    return lpos, rpos
    
def callback(self, msg):
    global box_data
    box_data = msg
    yolo = YOLO()
    yolo.image_detect(box_data)
    
def start():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    # speed 15 : 0.4, 0.0005, 0.25 -> stable
    # speed 25 : 0.3, 0.0, 0.3 -> almost stable
    pid_c = PID(0.5, 0.0, 0.0)			# 곡선(curve)에서의 PID
    pid_s = PID(0.25, 0.0005, 0.3)		# 직선(straight)에서의 PID

    while True:

        while not image.size == (640*480*3):
            continue

        lpos, rpos = process_image(calibrate_image(image))

        center = (lpos + rpos) / 2
        error = -(Width/2 - center - 10)		# 직진 주행을 할 때에 중앙선보다 왼쪽에서 주행하여 임의로 +10


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

    start()



