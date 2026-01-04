#!/usr/bin/env python3

'''
*****************************************
 PiFire PID Controller
*****************************************

 Description: This object will be used to calculate PID for maintaining
 temperature in the grill.  This controller adds basic clamping to prevent windup.
 When the output is saturated (either below 0 or above 1) then we do not increase the integral output.
 https://info.erdosmiller.com/blog/pid-anti-windup-techniques

 This controller was originally developed by GitHub user DBorello as part of his excellent
 PiSmoker project: https://github.com/DBorello/PiSmoker and modified by GitHub user markalston.

 PID controller based on proportional band in standard PID form https://en.wikipedia.org/wiki/PID_controller#Ideal_versus_standard_PID_form
   u   = Kp (e(t)+ 1/Ti INT + Td de/dt) = controller output
  PB   = Proportional Band
  Kp   = Proportional Gain = 1/PB
  Ti   = Integration Time constant
  Td   = Derivative Time Constant
  de   = Change in Error
  dt   = Change in Time
  INT  = Historic cumulative value of errors
  e(t) = Current Error = Set Point - Current Temp

  
  Configuration Defaults: 
  "config": {
      "PB": 100.0,
      "Td": 45.0,
      "Ti": 180.0
   }

*****************************************
'''

'''
Imported Libraries
'''
import time
import logging
from common import create_logger
from controller.base import ControllerBase 
log_level = logging.DEBUG
eventLogger = create_logger('events', filename='./logs/events.log', messageformat='%(asctime)s [%(levelname)s] %(message)s', level=log_level)

'''
Class Definition
'''
class Controller(ControllerBase):
	def __init__(self, config, units, cycle_data):
		super().__init__(config, units, cycle_data)
		# self.function_list.append('set_gains') 
		# self.function_list.append('get_k')
		self._calculate_gains(config['Cycle_Ratio'])

		self.p = 0.0
		self.i = 0.0
		self.d = 0.0
		self.u = 0

		self.last_update = time.time()
		self.error = 0.0
		self.error_last = 0.0
		self.set_point = 0

		self.derv = 0.0
		self.inter = 0.0

		self.set_target(0.0)

	def _calculate_gains(self, cr):
		self.cr = cr

	def update(self, current):
		# Output = constant cycle ratio
		self.u = self.cr

		return self.u

	def set_target(self, set_point):
		self.set_point = set_point
		self.error = 0.0
		self.inter = 0.0
		self.derv = 0.0
		self.last_update = time.time()

	def set_gains(self, pb, ti, td):
		# self._calculate_gains(pb,ti,td)
		self.donothing = 1

	def set_config(self,config):
		super().set_config(config)
		self._calculate_gains(config['Cycle_Ratio'])
		self.error = 0.0
		self.inter = 0.0
		self.derv = 0.0

	def get_k(self):
		return self.p, self.i, self.d
	
