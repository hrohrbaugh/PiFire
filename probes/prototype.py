'''
*****************************************
PiFire Probes Prototype Module 
*****************************************

Description: 
  This module simulates an ADC/RTD Device and returns temperature data.
	
	Ex Device Definition: 
	
	device_info = {
			'device' : 'your_device_name',	# Unique name for the device
			'module' : 'prototype',  			# Must be populated for this module to load properly
			'ports' : ['ADC0', 'ADC1', 'ADC2', 'ADC3'], # This should be defined by the user with the number of ports desired
			'config' : {
				'ADC0_rd': '10000',
            	'ADC1_rd': '10000',
            	'ADC2_rd': '10000',
            	'ADC3_rd': '10000',
            	'i2c_bus_addr': '0x48',
            	'voltage_ref': '3.28',
				'transient' : False
			} 
		}

	Note: This prototype module will calculate temperature based on the probe profiles, just like for the ADS1115 probes.  
	To get proper output, primary ports should use the "PT-1000-Grill-Probe-OEM" probe profile.  All other ports should use the 
	"Thermoworks-Pro-Series-HeaterMeter" probe profile. 

'''

'''
*****************************************
 Imported Libraries
*****************************************
'''

import random
from probes.base import ProbeInterface

'''
*****************************************
 Class Definitions 
*****************************************
'''

class ProtoDevice():
	''' Create a test devices that returns values for testing '''
	def __init__(self, port_map, primary_port, units, transient=False):
		self.transient = transient
		self.port_value = {}
		self.primary_port = primary_port
		self.units = units 
		''' Set initial voltages 
			Primary Probe Voltages based on the PT-1000-Grill-Probe-OEM profile 
			Other Probe Voltages based on the Thermoworks-Pro-Series-HeaterMeter profile		
		'''
		for port in port_map:
			if port == self.primary_port:
				self.port_value[port] = 316
			else: 
				self.port_value[port] = 3000

		self.maxPrimaryVoltage = 550
		self.minPrimaryVoltage = 300
		self.maxFoodVoltage = 1300
		self.minFoodVoltage = 3200
		self.primaryChangeFactor = 2
		self.otherChangeFactor = 10
		self.status = {
			#'battery_percentage' : random.randint(0,100),
			#'battery_voltage' : 3.3,
			#'battery_charging' : False,
			#'connected' : True,
			'error' : None
		}

	def read_voltage(self, port):
		seed = random.randint(0,9)
		if port == self.primary_port:
			if seed > 7 and self.port_value[port] < self.maxPrimaryVoltage:
				self.port_value[port] += self.primaryChangeFactor
			elif seed < 1 and self.port_value[port] > self.minPrimaryVoltage:
				self.port_value[port] -= self.primaryChangeFactor
		else:
			if self.transient:
				''' If transient, then return None-type approximately 80% of the time. '''
				if random.randint(0,100) < 80:
					return None 

			if seed > 7 and self.port_value[port] > self.maxFoodVoltage:
				self.port_value[port] -= self.otherChangeFactor
			elif seed < 1 and self.port_value[port] < self.minFoodVoltage:
				self.port_value[port] += self.otherChangeFactor

		return self.port_value[port]

	def get_status(self):
		return self.status

class ReadProbes(ProbeInterface):

	def __init__(self, probe_info, device_info, units):
		super().__init__(probe_info, device_info, units)

	def _init_device(self):
		self.time_delay = 0
		self.device = ProtoDevice(self.port_map, self.primary_port, self.units, transient=self.transient)
