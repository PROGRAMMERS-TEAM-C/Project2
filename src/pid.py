#!/user/bin/env python
# -*- coding: utf-8 -*-

class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte - self.p_error	# (현재 시간 error) - (이전 시간 error)
        self.p_error = cte
        if abs(self.i_error + cte) < 2000:	# 시간이 지날수록 계속 누적되면 i_error가 p나 d에 비해 너무 커지기에 상한선 설정
            self.i_error += cte			# ki = 0.0005인 경우, 0.0005 x 2000 = 1. 적분 부분이 1도 이상 영향 주지 않게끔 설정

        return self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error
