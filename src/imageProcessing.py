#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes

import sys
import os
import signal

class ImageProcessing:
    def __init__(self):
        self.image = np.empty(shape=[0])
        self.pub = None
        self.Width = 640
        self.Height = 480
        self.Offset = 305		# ROI 설정 시 쓰이는 값. Offset ~ Offset+Gap 까지
        self.Gap = 40
        self.mtx = np.array([
                [422.037858, 0.0, 245.895397], 
                [0.0, 435.589734, 163.625535], 
                [0.0, 0.0, 1.0]
                ])
        self.dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.Width, self.Height), 1, (self.Width, self.Height))

    def setImage(self, image):
        self.image = image

    def getWidth(self):
        return self.Width

    def getHeight(self):
        return self.Height

    def calibrate_image(self):     
        tf_image = cv2.undistort(self.image, self.mtx, self.dist, None, self.cal_mtx)
        x, y, w, h = self.cal_roi
        tf_image = tf_image[y:y+h, x:x+w]

        return cv2.resize(tf_image, (self.Width, self.Height))

    # draw lines
    def draw_lines(self, img, lines):
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1 + self.Offset), (x2, y2 + self.Offset), color, 2)
        return img

    # draw rectangle
    def draw_rectangle(self, img, lpos, rpos, offset=0):
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
    def divide_left_right(self, lines):
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

            if (slope < 0) and (x2 < self.Width/2 - 90):
                left_lines.append([Line.tolist()])
            elif (slope > 0) and (x1 > self.Width/2 + 90):
                right_lines.append([Line.tolist()])

        return left_lines, right_lines

    # 선들의 x, y좌표들의 평균으로 기울기와 y절편 구하는 함수
    def get_line_params(self, lines):
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
    def get_line_pos(self, img, lines, left=False, right=False):
        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            if left:
                pos = 0		# 왼쪽 차선이 검출되지 않으면 lpos = 0
            if right:
                pos = self.Width		# 오른쪽 차선이 검출되지 않으면 rpos = 640
        else:
            y = self.Gap / 2
            pos = (y - b) / m

            b += self.Offset
            x1 = (self.Height - b) / float(m)
            x2 = ((self.Height/2) - b) / float(m)

            cv2.line(img, (int(x1), self.Height), (int(x2), (self.Height/2)), (255, 0,0), 3)

        return img, int(pos)

    # 이미지 전처리, 차선과 차선의 좌표 검출 및 반환
    def process_image(self, frame):
        # gray
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        
        # roi
        roi = gray[self.Offset : self.Offset+self.Gap, 0 : self.Width]

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
        left_lines, right_lines = self.divide_left_right(all_lines)

        # get center of lines
        frame, lpos = self.get_line_pos(frame, left_lines, left=True)
        frame, rpos = self.get_line_pos(frame, right_lines, right=True)

        # draw lines
        frame = self.draw_lines(frame, left_lines)
        frame = self.draw_lines(frame, right_lines)
        frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                    
        # draw rectangle
        frame = self.draw_rectangle(frame, lpos, rpos, offset=self.Offset)

        # show image
        cv2.imshow('final', frame)

        return lpos, rpos
