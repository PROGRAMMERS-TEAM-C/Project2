#!/usr/bin/env python
# -*- coding: utf-8 -*-

class PID():
	def __init__(self,kp,ki,kd):
		self.kp = kp
		self.ki = ki
        self.kd = kd
		self.p_error = 0.0
		self.i_error = 0.0
		self.d_error = 0.0
		
	#P term = Kp x (Error), I term = Ki x (누적 Error), D term = Kd x (Error 변화량)
	def pid_control(self,cte):
		self.d_error = cte - self.p_error #cte = 화면의 중앙점과 좌우차선의 중점과의 차이, D값은 현재 cte 값과 이전 p값의 차이 값을 적용
		self.p_error = cte #P값은 CTE값을 그대로 적용
		self.i_error += cte #I값은 CTE값을 계속 더해서 누적한 값을 적용
		
		return self.kp*self.p_error + self.ki*self.i_error + self.kd*self.d_error
