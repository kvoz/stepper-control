# -*- coding: utf-8 -*- 

"""
Model is fully based on application note AVR446: Linear speed control of stepper motor (Atmel).
"""

import numpy as np

class stepper:
	 
	def __init__(self, name):
		self.name = name	# name of driver
		
		self.f_tim = None 	# frequency of timer-controller of driver
		self.spr = None		# steps per loop

		self.speed_profile_ready = False
		self.angular_accel = 0.0
		self.angular_decel = 0.0
		self.angular_speed = 0.0
		self.startStep = 0
		self.stopStep = 0
		
		self.accelSteps = 0
		self.decelSteps = 0
		self.maxSpeedLimSteps = 0
		
		self.discrete_counter = []
		self.discrete_angular_speed = []
		self.discrete_angular_pos = []

		self.discrete_linear_speed = []
		self.discrete_linear_pos = []
		
		self.discrete_time = []
		
		self.alpha = 0.0 				# angle per one step in rad

		self.angular_speed_start = 0.0 	# start angular speed
		self.angular_speed_max = 0.0 	# rad/sec (1 rad/sec = 9.55 rpm)
		self.linear_speed_max = 0.0 	# in m/s

		self.c0_accel = 0 				# counter of timer-driver for aceleration on start
		self.c0_decel = 0 				# counter of timer-driver for deceleration on end
		self.cMax = 0 					# counter of timer-driver for max speed

		self.pulley_diameter = 0.0 		# stored in meters
		self.pulleyRadius = 0.0 		# stored in meters
		self.pulleyCircleLenght = 0.0 	# stored in meters
		self.stepsPerCm = 0.0
		
	def configurate(self, f_tim, spr, pulley_diameter, target_linear_speed):
		self.f_tim = f_tim
		self.spr = spr
		self.pulley_diameter = pulley_diameter
		self.linear_speed_max = target_linear_speed

		self.pulley_radius = pulley_diameter / 2.0
		self.pulley_circle_lenght = pulley_diameter * np.pi
		self.steps_per_cm = (spr / self.pulley_circle_lenght) / 100.0
		self.angular_speed_max = target_linear_speed / self.pulley_radius

	def __calc_angular_speed(self, counter_timer):
		return (self.alpha * self.f_tim / counter_timer)

	def __calc_linear_speed(self, angular_speed):
		return (angular_speed * self.pulley_radius)

	def __calc_counter(self, step, accelType):
		counter = 0.0

		if accelType == "accel":
			counter = self.c0_accel * (np.sqrt(step + 1) - np.sqrt(step))
		elif accelType == "decel":
			counter = self.c0_decel * (np.sqrt(step + 1) - np.sqrt(step))
		
		print(f"[{step}]c = {counter} and int = {int(counter)}")

		return counter

	def __calc_pos(self, step):
		return ( (step / self.spr) * self.pulley_diameter * np.pi * 100.0)

	def __calc_angle(self, step):
		return (self.alpha * step)

	def generate_speed_profile(self, angular_accel, angular_decel, linear_speed, startPoint, stopPoint):
		self.angular_accel = angular_accel
		self.angular_decel = angular_decel
		self.angular_speed = linear_speed / self.pulley_radius
		
		self.startStep = int(startPoint * self.steps_per_cm)
		self.stopStep = int(stopPoint * self.steps_per_cm)

		self.alpha = 2.0 * np.pi / self.spr

		self.c0_accel = int(self.f_tim * np.sqrt(2.0 * self.alpha / self.angular_accel))
		self.c0_decel = int(self.f_tim * np.sqrt(2.0 * self.alpha / self.angular_decel))
		self.cMax = int(self.alpha * self.f_tim / self.angular_speed)

		self.angular_speed0 = self.alpha * self.f_tim / self.c0_accel

		self.accelSteps = int((self.stopStep - self.startStep + 1) * self.angular_decel / (self.angular_accel + self.angular_decel))
		self.maxSpeedLimSteps = int(self.angular_speed ** 2 / (2.0 * self.alpha * self.angular_accel))
		
		if self.maxSpeedLimSteps < self.accelSteps:
			self.accelSteps = self.maxSpeedLimSteps
			
		self.decelSteps = int(self.accelSteps * self.angular_accel / self.angular_decel)

		self.speed_profile_ready = True

	def print_speed_profile(self):
		print(f"{self.name} speed profile report:")        
		print(f"Total steps = {stopStep - startStep}")
		print(f"Steps to accel = {self.accelSteps}")
		print(f"Steps to decel = {self.decelSteps}")
		print(f"Steps to freerun = {self.stopStep -  self.accelSteps - self.decelSteps}")
		print(f"Steps before decel = {self.stopStep -  self.accelSteps}")
		print(f"Start angular speed {self.angular_speed0} vs target speed {self.angular_speed}")
		print("=========================================================")

	def generate_counter_table(self):
		self.discrete_counter = []

		self.discrete_counter.append(0)

		for step in range(self.startStep + 1, self.stopStep + 1):
			if step < self.accelSteps:
				self.discrete_counter.append(self.__calc_counter(step, "accel"))
			elif step >= (self.stopStep - self.startStep - self.decelSteps):
				self.discrete_counter.append(self.__calc_counter(self.stopStep - step, "decel"))
			else:
				self.discrete_counter.append(self.cMax)

	def generate_angular_speed_table(self):
		self.discrete_angular_speed = []
		self.discrete_angular_speed.append(0)
		for ct in self.discrete_counter[1:]:
			self.discrete_angular_speed.append(self.__calc_angular_speed(ct))
	
	def generate_angular_position_table(self):
		self.discrete_angular_pos = []
		self.discrete_angular_pos.append(0)
		for i in range(self.startStep, self.stopStep):
			self.discrete_angular_pos.append(self.__calc_angle(i))

	def generate_linear_speed_table(self):
		self.discrete_linear_speed = []
		for angular_speed in self.discrete_angular_speed:
			self.discrete_linear_speed.append(self.__calc_linear_speed(angular_speed))

	def generate_linear_position_table(self):
		self.discrete_linear_pos = []
		self.discrete_linear_pos.append(0)
		for i in range(self.startStep, self.stopStep):
			self.discrete_linear_pos.append(self.__calc_pos(i))
	
	def generate_time_table(self):
		prevT = 0
		self.discrete_time = []
		for ct in self.discrete_counter:
			self.discrete_time.append(prevT + ct / self.f_tim)
			prevT = self.discrete_time[-1]

	def calculate(self):
		if self.speed_profile_ready:
			self.generate_counter_table()
			self.generate_time_table()
			self.generate_angular_speed_table()
			self.generate_angular_position_table()
			self.generate_linear_speed_table()
			self.generate_linear_position_table()
		else:
			print("No speed profile generated")

	def plot(self, distance_limit_1 = 75, distance_limit_2 = 125):
		plt.rcParams['xtick.minor.visible'] = True
		plt.rcParams['ytick.minor.visible'] = True
		plt.rcParams['axes.grid'] = True
		plt.rcParams['axes.grid.which'] = 'both'

		fig, axs = plt.subplots(1, 2, sharex=True)

		axs[0].set_title('Angle and angular speed vs time')
		axs[0].plot(self.discrete_time, self.discrete_angular_speed, color='b')
		axs[0].set(xlabel='time, s', ylabel='omega, rad/s')
		axs[0].grid(which='major', lw=1.5)
		axs[0].tick_params(axis='y', labelcolor='b')

		axs_2 = axs[0].twinx()
		axs_2.plot(self.discrete_time, np.asarray(self.discrete_angular_pos)*180.0/3.1415926, color='r')
		axs_2.set(xlabel='time, s', ylabel='alpha, o')
		axs_2.grid(which='major', lw=1.5)
		axs_2.tick_params(axis='y', labelcolor='r')

		axs[1].set_title('Position and speed vs time')
		axs[1].plot(self.discrete_time, self.discrete_linear_speed, color='b')
		axs[1].set(xlabel='time, s', ylabel='V, m/s')
		axs[1].grid(which='major', lw=1.5)
		axs[1].tick_params(axis='y', labelcolor='b')

		axs_3 = axs[1].twinx()
		axs_3.plot(self.discrete_time, self.discrete_linear_pos, color='r')
		axs_3.axhline(y = distance_limit_1, color = 'g', linestyle = '--')
		axs_3.axhline(y = distance_limit_2, color = 'g', linestyle = '--')
		axs_3.set(xlabel='time, s', ylabel='S, cm')
		axs_3.grid(which='major', lw=1.5)
		axs_3.tick_params(axis='y', labelcolor='r')

		# axs[].set_title('Timer counter vs time')
		# axs[].step(self.discrete_time, self.discrete_counter)
		# axs[].set(xlabel='time, s', ylabel='counts')
		# axs[].grid(which='major', lw=1.5)

		fig.tight_layout()
		plt.show()

	
if __name__ == "__main__":
	import matplotlib.pyplot as plt

	F_TIM = 72000  				# Hz
	SPR = 200					# Steps
	PULLEY_DIAMETER = 22.92e-3 	# in m
	TARGET_DISTANCE = 200  		# in cm

	ANGULAR_ACCEL = 40.0   		# in rad/s^2
	ANGULAR_DECEL = 40.0   		# in rad/s^2
	LINEAR_SPEED = 0.7 			# in m/s

	stepper_test = stepper("test")
	stepper_test.configurate(F_TIM, SPR, PULLEY_DIAMETER, LINEAR_SPEED)

	stepper_test.generate_speed_profile(ANGULAR_ACCEL, ANGULAR_DECEL, LINEAR_SPEED, 0, TARGET_DISTANCE)
	stepper_test.calculate()
	stepper_test.plot()