#!/usr/bin/env python3

'''
*****************************************
 PiFire PID Controller
*****************************************

 Description: This object will be used to calculate PID for maintaining
 temperature in the grill.  This controller is based on the velocity form of the PID equation instead of the positional form
 When the output is saturated (either below 0 or above 1) then we do not increase the integral output.
 https://info.erdosmiller.com/blog/pid-anti-windup-techniques

 This controller was originally developed by GitHub user DBorello as part of his excellent
 PiSmoker project: https://github.com/DBorello/PiSmoker and modified by GitHub user markalston.

 PID controller based on velocity PID form (type c) https://apmonitor.com/pdc/index.php/Main/ProportionalIntegralDerivative
   u    = u(k-1) - KP(PV - PV(k-1)) + KIe(k)deltaT - KD ((PV - 2PV(k-1) + PV(k-2))/deltaT) = controller output
   k    = current measurement index
   KP   = Proportional Gain
   KI   = Integral Gain
   KD   = Derivative Gain
   PV   = Process Variable (Chamber Temp)
   e    = error

For this controller it is recommended to perform step-response testing in order to approximate a FOPDT (First order plus dead time)
model.  This provides Kp, tau, and theta (see velocity PID link above for equations)
   Kp     = controller gain
   tau    = process time constant
   theta  = process time delay
   lambda = max(1.5 * theta, 0.5 * tau)

IMC-style PID tuning for FOPDT utilized the above parameters to estimate Kc, Ti, and Td:
    Kc = tau / (Kp*(lam + theta)) = controller gain
    Ti = tau + theta/2 = integral reset time
    Td = (tau*theta) / (2*tau + theta) = derivative time constant
	
Then, to get the final gains:
   KP = Kc
   KI = Kc / Ti
   KD = Kc * Td

  
  Configuration Defaults: 
  "config": {
      "Kc": 0.0015,
      "Ti": 0.00000375,
      "Td": 0.0
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
		self.function_list.append('initialize')
		self.function_list.append('set_gains')
		self.function_list.append('get_k')

		# pb, ti, td
		self._calculate_gains(config.get('KP'), config.get('Ti'), config.get('Td'))

		self.u_init = config.get('u_init')

		self.p = 0.0
		self.i = 0.0
		self.d = 0.0
		self.u = 0
		self.u_1 = None
		self.pv_1 = None
		self.pv_2 = None
		self.initialized = False
		
		self.last_update = time.time()
		self.error = 0.0
		self.error_last = 0.0
		self.set_point = 0

		self.derv = 0.0
		self.inter = 0.0

		self.set_target(0.0)

	def _calculate_gains(self, kc, ti, td):
		if kc == 0:
			self.kp = 0
		else:
			self.kp = kc
		if ti == 0:
			self.ki = 0
		else:
			self.ki = ti
		self.kd = td


	def update(self, current):
		# dt
		dt = time.time() - self.last_update

		# Calculate needed deltas and error
		d_pv = current - self.pv_1
		dd_pv = (current - (2.0 * self.pv_1) + self.pv_2)
		error = self.set_point - current
		
        # Calculate P, I, and D portions of equation
		self.p = self.kp * d_pv
		self.i = self.ki * error * dt
		self.d = self.kd * (dd_pv / dt)

		# PID
		self.u = self.u_1 - self.p + self.i - self.d
		
        # Update k-1 and k-2 variables for next loop
		self.u_1 = self.u
		self.pv_2 = self.pv_1
		self.pv_1 = current

		# Clamping anti-windup method. 
		# Stops integration when the sum of the block components exceeds the output limits 
		# and the integrator output and block input have the same sign. 
		# Resumes integration when either the sum of the block components exceeds the output limits 
		# and the integrator output and block input have opposite sign or the sum no longer exceeds the output limits.
		# 
		# Implemented via reversing the addition to self.inter above if we are clamping.		
		if not ((abs(self.u) >= 1) and (self.i * self.u > 0)):
			clamping_log = "false"
			eventLogger.debug('Not clamping integrator.')
		else:
			clamping_log = "true"
			eventLogger.debug('Clamping Integrator.')
			self.inter -= error * dt	
		eventLogger.debug(f'PID Update... error: {str(error)}, p: {str(self.p)}, i: {str(self.i)}, d: {str(self.d)}, pid: {self.u}, clamp: {str(clamping_log)}' )	

		# Update for next cycle
		self.error_last = error
		self.last_update = time.time()

		return self.u
	
	def initialize(self, u_init, pv_init, last_mode):
		if self.u_init == 0 or last_mode != 'Startup':
			self.u_1 = u_init
		else:
			self.u_1 = self.u_init
		self.pv_1 = pv_init
		self.pv_2 = pv_init
		eventLogger.debug(f'Velocity PID Initialized @ u_init = {self.u_1} & pv_init = {pv_init}, last mode = {last_mode}')

	def set_target(self, set_point):
		self.set_point = set_point
		self.error = 0.0
		self.inter = 0.0
		self.derv = 0.0
		self.last_update = time.time()

	def set_gains(self, pb, ti, td):
		self._calculate_gains(pb,ti,td)

	def set_config(self,config):
		super().set_config(config)
		self._calculate_gains(config.get('KC'), config.get('Ti'), config.get('Td'))
		self.error = 0.0
		self.inter = 0.0
		self.derv = 0.0


	def get_k(self):
		return self.kp, self.ki, self.kd
