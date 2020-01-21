#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Robotnik Automation SLL
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotnik Automation SSL nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

import time, threading
import sys

from robotnik_msgs.msg import State, RobotnikMotorsStatus, inputs_outputs,MotorStatus, BatteryStatus, BatteryDockingStatusStamped, BatteryDockingStatus
from std_msgs.msg import Bool, Float32
from std_msgs.msg import Bool, Float32
from robotnik_msgs.srv import enable_disable, set_digital_output

DEFAULT_FREQ = 100.0
MAX_FREQ = 500.0


# Class Template of Robotnik component for Pyhton
class RobotnikBaseHwSim:

	def __init__(self, args):

		self.node_name = rospy.get_name() #.replace('/','')
		self.desired_freq = args['desired_freq']
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ


		self._motors = args['motors']

		print self._motors

		#exit()
		#self._joint_names = args['joint_name']
		#self._joint_can_ids = args['joint_can_id']

		if self._motors == None or len(self._motors) == 0:
			rospy.logerr('%s::init: no motors defined'%(self.node_name))
			exit()

		self._battery_voltage = args['battery_voltage']
		self._battery_amperes = args['battery_amperes']
		self._battery_alarm_voltage = args['battery_alarm_voltage']
		self._num_inputs_per_driver = args['num_inputs_per_driver']
		self._num_analog_inputs_per_driver = args['num_analog_inputs_per_driver']
		self._num_analog_outputs_per_driver = args['num_analog_outputs_per_driver']
		self._num_outputs_per_driver = args['num_outputs_per_driver']
		self._power_consumption = args['power_consumption']
		self._k_analog_inputs_multipliers = args['k_analog_inputs_multipliers']
		self._voltage_analog_input_number = args['voltage_analog_input_number']
		self._current_analog_input_number = args['current_analog_input_number']
		self._charge_digital_output_number = args['charge_digital_output_number']

		self.real_freq = 0.0

		# Saves the state of the component
		self.state = State.INIT_STATE
		# Saves the previous state
		self.previous_state = State.INIT_STATE
		# flag to control the initialization of the component
		self.initialized = False
		# flag to control the initialization of ROS stuff
		self.ros_initialized = False
		# flag to control that the control loop is running
		self.running = False
		# Variable used to control the loop frequency
		self.time_sleep = 1.0 / self.desired_freq
		# State msg to publish
		self.msg_state = State()
		# Timer to publish state
		self.publish_state_timer = 1


		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)

		self._io = inputs_outputs()

		for i in range(len(self._motors)*self._num_inputs_per_driver):
			self._io.digital_inputs.append(False)
		for i in range(len(self._motors)*self._num_outputs_per_driver):
			self._io.digital_outputs.append(False)
		for i in range(len(self._motors)*self._num_analog_inputs_per_driver):
			self._io.analog_inputs.append(False)
		for i in range(len(self._motors)*self._num_analog_outputs_per_driver):
			self._io.analog_outputs.append(False)

		if len(self._k_analog_inputs_multipliers) != len(self._io.analog_inputs):
			rospy.logwarn('%s::__init__: len of param k_analog_inputs_multipliers is different from the len of analog inputs',self.node_name)
			self._k_analog_inputs_multipliers = [1] * len(self._io.analog_inputs)

		if self._voltage_analog_input_number <= 0 or self._voltage_analog_input_number > len(self._io.analog_inputs):
			rospy.logerr('%s::__init__: voltage_analog_input_number (%d) out of range [1, %d].',self.node_name, self._voltage_analog_input_number, len(self._io.analog_inputs))
			sys.exit()

		if self._current_analog_input_number <= 0 or self._current_analog_input_number > len(self._io.analog_inputs):
			rospy.logerr('%s::__init__: current_analog_input_number (%d) out of range [1, %d].',self.node_name, self._current_analog_input_number, len(self._io.analog_inputs))
			sys.exit()

		if self._charge_digital_output_number <= 0 or self._charge_digital_output_number > len(self._io.digital_outputs):
			rospy.logerr('%s::__init__: charge_digital_output_number (%d) out of range [1, %d].',self.node_name, self._charge_digital_output_number, len(self._io.digital_outputs))
			sys.exit()

		self._motor_status = RobotnikMotorsStatus()
		for i in self._motors:
			print 'motor %s'%i
			self._motor_status.name.append(self._motors[i]['joint'])
			self._motor_status.can_id.append(self._motors[i]['can_id'])
			ms = MotorStatus()
			ms.state = "READY"
			ms.status = "OPERATION_ENABLED"
			ms.communicationstatus = "OPERATIONAL"
			ms.statusword = "1110110001100000"
			ms.driveflags = "10000000000000000000000000000000000010000110000000000000000000000000"
			ms.activestatusword = ['SW_READY_TO_SWITCH_ON', 'SW_SWITCHED_ON', 'SW_OP_ENABLED', 'SW_VOLTAGE_ENABLED', 'SW_QUICK_STOP',
		  	'UNKNOWN', 'SW_TARGET_REACHED']
			ms.activedriveflags = ['BRIDGE_ENABLED', 'NONSINUSOIDAL_COMMUTATION', 'ZERO_VELOCITY', 'AT_COMMAND']

			self._motor_status.motor_status.append(ms)


		self._battery_voltage = self._battery_voltage
		self._battery_alarm = False
		self._emergency_stop = False


	def setup(self):
		'''
			Initializes de hand
			@return: True if OK, False otherwise
		'''
		self.initialized = True

		return 0


	def rosSetup(self):
		'''
			Creates and inits ROS components
		'''
		if self.ros_initialized:
			return 0

		# Publishers
		self._state_publisher = rospy.Publisher('~state', State, queue_size=10)
		self._io_publisher = rospy.Publisher('~io', inputs_outputs, queue_size=10)
		self._motor_status_publisher = rospy.Publisher('~status', RobotnikMotorsStatus, queue_size=10)
		self._voltage_publisher = rospy.Publisher('~voltage', Float32, queue_size=10)
		self._emergency_stop_publisher = rospy.Publisher('~emergency_stop', Bool, queue_size=10)
		self._toogle_robot_operation_service_server = rospy.Service('robotnik_base_control/enable', enable_disable, self.toogleRobotOperationserviceCb)
		# Subscribers
		# topic_name, msg type, callback, queue_size
		# self.topic_sub = rospy.Subscriber('topic_name', Int32, self.topicCb, queue_size = 10)
		# Service Servers
		self.set_digitalservice_server = rospy.Service('~set_digital_output', set_digital_output, self.setDigitalOutputServiceCb)
		# Service Clients
		# self.service_client = rospy.ServiceProxy('service_name', ServiceMsg)
		# ret = self.service_client.call(ServiceMsg)

		self.ros_initialized = True

		self.publishROSstate()

		return 0


	def shutdown(self):
		'''
			Shutdowns device
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.initialized:
			return -1
		rospy.loginfo('%s::shutdown'%self.node_name)

		# Cancels current timers
		self.t_publish_state.cancel()

		self._state_publisher.unregister()

		self.initialized = False

		return 0


	def rosShutdown(self):
		'''
			Shutdows all ROS components
			@return: 0 if it's performed successfully, -1 if there's any problem or the component is running
		'''
		if self.running or not self.ros_initialized:
			return -1

		# Performs ROS topics & services shutdown
		self._state_publisher.unregister()

		self.ros_initialized = False

		return 0


	def stop(self):
		'''
			Creates and inits ROS components
		'''
		self.running = False

		return 0


	def start(self):
		'''
			Runs ROS configuration and the main control loop
			@return: 0 if OK
		'''
		self.rosSetup()

		if self.running:
			return 0

		self.running = True

		self.controlLoop()

		return 0


	def controlLoop(self):
		'''
			Main loop of the component
			Manages actions by state
		'''

		while self.running and not rospy.is_shutdown():
			t1 = time.time()

			if self.state == State.INIT_STATE:
				self.initState()

			elif self.state == State.STANDBY_STATE:
				self.standbyState()

			elif self.state == State.READY_STATE:
				self.readyState()

			elif self.state == State.EMERGENCY_STATE:
				self.emergencyState()

			elif self.state == State.FAILURE_STATE:
				self.failureState()

			elif self.state == State.SHUTDOWN_STATE:
				self.shutdownState()

			self.allState()

			t2 = time.time()
			tdiff = (t2 - t1)


			t_sleep = self.time_sleep - tdiff

			if t_sleep > 0.0:
				try:
					rospy.sleep(t_sleep)
				except rospy.exceptions.ROSInterruptException:
					rospy.loginfo('%s::controlLoop: ROS interrupt exception'%self.node_name)
					self.running = False

			t3= time.time()
			self.real_freq = 1.0/(t3 - t1)

		self.running = False
		# Performs component shutdown
		self.shutdownState()
		# Performs ROS shutdown
		self.rosShutdown()
		rospy.loginfo('%s::controlLoop: exit control loop'%self.node_name)

		return 0


	def rosPublish(self):
		'''
			Publish topics at standard frequency
		'''
		self._io.analog_inputs[self._voltage_analog_input_number - 1] = self._battery_voltage
		if self._io.digital_outputs[self._charge_digital_output_number - 1] is True: # charging
			self._io.analog_inputs[self._current_analog_input_number - 1] = -self._power_consumption
		else:
			self._io.analog_inputs[self._current_analog_input_number - 1] = self._power_consumption
		self._io_publisher.publish(self._io)
		self._motor_status_publisher.publish(self._motor_status)
		self._voltage_publisher.publish(self._battery_voltage)
		self._emergency_stop_publisher.publish(self._emergency_stop)

		return 0


	def initState(self):
		'''
			Actions performed in init state
		'''

		if not self.initialized:
			self.setup()

		else:
			self.switchToState(State.STANDBY_STATE)


		return


	def standbyState(self):
		'''
			Actions performed in standby state
		'''
		self.switchToState(State.READY_STATE)

		return


	def readyState(self):
		'''
			Actions performed in ready state
		'''


		return


	def shutdownState(self):
		'''
			Actions performed in shutdown state
		'''
		if self.shutdown() == 0:
			self.switchToState(State.INIT_STATE)

		return


	def emergencyState(self):
		'''
			Actions performed in emergency state
		'''

		return


	def failureState(self):
		'''
			Actions performed in failure state
		'''


		return


	def switchToState(self, new_state):
		'''
			Performs the change of state
		'''
		if self.state != new_state:
			self.previous_state = self.state
			self.state = new_state
			rospy.loginfo('%s::switchToState: %s'%(self.node_name, self.stateToString(self.state)))

		return


	def allState(self):
		'''
			Actions performed in all states
		'''
		self.rosPublish()

		return


	def stateToString(self, state):
		'''
			@param state: state to set
			@type state: State
			@returns the equivalent string of the state
		'''
		if state == State.INIT_STATE:
			return 'INIT_STATE'

		elif state == State.STANDBY_STATE:
			return 'STANDBY_STATE'

		elif state == State.READY_STATE:
			return 'READY_STATE'

		elif state == State.EMERGENCY_STATE:
			return 'EMERGENCY_STATE'

		elif state == State.FAILURE_STATE:
			return 'FAILURE_STATE'

		elif state == State.SHUTDOWN_STATE:
			return 'SHUTDOWN_STATE'
		else:
			return 'UNKNOWN_STATE'


	def publishROSstate(self):
		'''
			Publish the State of the component at the desired frequency
		'''
		self.msg_state.state = self.state
		self.msg_state.state_description = self.stateToString(self.state)
		self.msg_state.desired_freq = self.desired_freq
		self.msg_state.real_freq = self.real_freq
		self._state_publisher.publish(self.msg_state)

		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()


	def batteryDischarge(self):
		'''
			Method to simulate the battery discharge
		'''

		return


	def toogleRobotOperationserviceCb(self, req):
		'''
			ROS service server to toogle robot operation
			@param req: Required action
			@type req: robotnik_msgs/enable_disable
		'''
		# DEMO
		rospy.loginfo('%s::toogleRobotOperationserviceCb: toogling robot to %s', self.node_name, str(req.value))

		return True

	def setDigitalOutputServiceCb(self, req):
		'''
			ROS service server to set the hw digital outputs
			@param req: Required action
			@type req: robotnik_msgs/set_digital_output
		'''
		if (req.output <= 0 or req.output > len(self._io.digital_outputs)):
			rospy.loginfo('%s::setDigitalOutputServiceCb: output %d out of range', self.node_name, req.output)
			return False

		rospy.loginfo('%s::setDigitalOutputServiceCb: setting output %d to %d', self.node_name, req.output, req.value)
		self._io.digital_outputs[req.output - 1] = req.value

		return True

	"""
	def topicCb(self, msg):
		'''
			Callback for inelfe_video_manager state
			@param msg: received message
			@type msg: std_msgs/Int32
		'''
		# DEMO
		rospy.loginfo('RobotnikBaseHwSim:topicCb')



	"""

def main():

	rospy.init_node("robotnik_base_hw")


	_name = rospy.get_name().replace('/','')

	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
	  'motors': None,
	  'battery_voltage': 24,
	  'battery_amperes': 15,
	  'battery_alarm_voltage': 22.5,
	  'num_inputs_per_driver': 5,
	  'num_outputs_per_driver': 3,
	  'num_analog_inputs_per_driver': 1,
	  'num_analog_outputs_per_driver': 0,
	  'power_consumption': 2,
	  'k_analog_inputs_multipliers': [],
	  'voltage_analog_input_number': 1,
	  'current_analog_input_number': 2,
	  'charge_digital_output_number': 1,
	}

	args = {}

	for name in arg_defaults:
		try:
			if rospy.search_param(name):
				args[name] = rospy.get_param('~%s'%(name)) # Adding the name of the node, because the para has the namespace of the node
			else:
				args[name] = arg_defaults[name]
			#print name
		except rospy.ROSException, e:
			rospy.logerr('%s: %s'%(e, _name))


	rc_node = RobotnikBaseHwSim(args)

	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()


if __name__ == "__main__":
	main()
