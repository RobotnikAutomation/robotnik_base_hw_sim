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
import copy

from robotnik_msgs.msg import State

from gazebo_msgs.srv import GetModelState, GetLinkProperties, SetLinkProperties, GetLinkPropertiesResponse, SetModelState, SetLinkPropertiesRequest, SetLinkState
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.msg import LinkState, LinkStates
from robotnik_base_hw_sim.srv import Pick, Place
from robotnik_base_hw_sim.msg import PickState, PickStates
from geometry_msgs.msg import Pose

DEFAULT_FREQ = 100.0
MAX_FREQ = 500.0


class GazeboLinkState:
	'''
		Class to save the Gazebo Link State plus the state of reception
	'''
	
	def __init__(self):
		
		self._state = LinkState()
		# Saves the properties of the link directly from the response
		self._properties = GetLinkPropertiesResponse()
		self._is_received = False
		self._last_time_received = rospy.Time(0)
		self._timeout_msg = 5.0 # seconds without receiving 
		
	def update(self, link_state, time):
		'''
			Updates model state and time
			@param link_state as an array of gazebo_msgs/LinkState
			@param time as ROS time stamp
		'''
		self._last_time_received = time
		self._state = link_state
		
	def update_properties(self, link_properties):
		'''
			Updates model state and time
			@param link_state as an array of gazebo_msgs/GetLinkPropertiesResponse
		'''
		self._properties = link_properties	
		
	def is_received(self):
		'''
			@returns True if the model state is being received
		'''
		
		if (rospy.Time.now() - self._last_time_received).to_sec() > self._timeout_msg:
			self._is_received = False
		else:
			self._is_received = True
		
		return self._is_received
		
	def get(self):
		'''
			@returns the model state as gazebo_msgs/LinkState
		'''
		return self._state
		
	def get_properties(self):
		'''
			@returns the current state of the properties as gazebo_msgs/GetLinkPropertiesResponse
		'''
		return self._properties
		


class GazeboModelState:
	'''
		Class to save the Gazebo Model State plus the state of reception
	'''
	
	def __init__(self):	
		self._state = ModelState()
		self._is_received = False
		self._last_time_received = rospy.Time(0)
		self._timeout_msg = 5.0 # seconds without receiving 
		
	def update(self, model_state, time):
		'''
			Updates model state and time
			@param model_state as gazebo_msgs/ModelState
			@param time as ROS time stamp
		'''
		self._last_time_received = time
		self._state = model_state
		
	def is_received(self):
		'''
			@returns True if the model state is being received
		'''
		
		if (rospy.Time.now() - self._last_time_received).to_sec() > self._timeout_msg:
			self._is_received = False
		else:
			self._is_received = True
		
		return self._is_received
		
	def get(self):
		'''
			@returns the model state as gazebo_msgs/ModelState
		'''
		return self._state


# Saves the information related to a pick between links
class GazeboPickAndPlace:
	
	def __init__(self, robot_link, object_link, transform_pose):
		'''
			@param robot_link as string like model::link. Sets the link where the object will be linked
			@param object_link as string like model::link. Sets the link of the object that will be linked
			@param transform_pose as geometry_msgs/Pose, to save the transform between links
		'''
		
		self._transform_pose = transform_pose
		self._robot_link = robot_link
		self._object_link = object_link
		self._robot_model = robot_link.split('::')[0]
		self._object_model = object_link.split('::')[0]
		self._place = False
	
	
	def applyTransformToPose(self, pose):
		'''
			@param pose as geometry_msgs/Pose 
			@returns the pose applying the transform pose
		'''
		# TODO: apply the transform correctly
		#pose.position.x += self._transform_pose.position.x
		#pose.position.y += self._transform_pose.position.y
		# only z for now
		pose.position.z += self._transform_pose.position.z
		
		#pose.orientation.x += self._transform_pose.orientation.x
		#pose.orientation.y += self._transform_pose.orientation.y
		#pose.orientation.z += self._transform_pose.orientation.z
		
	def getRobotModel(self):
		
		return self._robot_model
		
	def getRobotLink(self):
		
		return self._robot_link
		
	def getObjectModel(self):
		
		return self._object_model
		
	def getObjectLink(self):
		
		return self._object_link
		
	def place(self):
		self._place = True
	
	def isPlaceRequired(self):

		return self._place
		
		
	
# Class Template of Robotnik component for Pyhton
class ElevatorFakePickup:
	
	def __init__(self, args):
		
		self.node_name = rospy.get_name() #.replace('/','')
		self.desired_freq = args['desired_freq'] 
		# Checks value of freq
		if self.desired_freq <= 0.0 or self.desired_freq > MAX_FREQ:
			rospy.loginfo('%s::init: Desired freq (%f) is not possible. Setting desired_freq to %f'%(self.node_name,self.desired_freq, DEFAULT_FREQ))
			self.desired_freq = DEFAULT_FREQ
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
		
		# TODO: do it from ros params
		# saves the state of all the gazebo models related to robots
		self._gazebo_robots = {}
		# saves the state of all the gazebo models related to pickable objects 
		self._gazebo_objects = {}
		
		self._gazebo_robots['rb2_a'] = {'model': GazeboModelState(), 'links': {}}
		self._gazebo_objects['rb2cart'] = {'model': GazeboModelState(), 'links': {}}
		
		# save the current links/picking between robot & objects (GazeboPickAndPlace)
		self._current_picks = {}
		
			
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
		self._pick_state_publisher = rospy.Publisher('~pick_states', PickStates, queue_size=10)
		self._gazebo_robot_models_publisher = rospy.Publisher('~gazebo_robot_models', ModelStates, queue_size=10)
		self._gazebo_object_models_publisher = rospy.Publisher('~gazebo_objects_models', ModelStates, queue_size=10)
		self._gazebo_robot_links_publisher = rospy.Publisher('~gazebo_robot_links', LinkStates, queue_size=10)
		self._gazebo_object_links_publisher = rospy.Publisher('~gazebo_objects_links', LinkStates, queue_size=10)
		# Subscribers
		# topic_name, msg type, callback, queue_size
		self.topic_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.modelStatesCb, queue_size = 10)
		self.topic_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.linkStatesCb, queue_size = 10)
		# Service Servers
		self.pick_service_server = rospy.Service('~pick', Pick, self.pickServiceCb)
		self.place_service_server = rospy.Service('~place', Place, self.placeServiceCb)

		# Service Clients
		self.get_link_properties_service_client = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
		self.set_link_properties_service_client = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
		self.set_link_state_service_client = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
		#self.set_model_state_service_client = rospy.ServiceProxy('/gazebo/set_link_properties', SetLinkProperties)
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
		robot_models_msg = ModelStates()
		
		for robot_id in self._gazebo_robots:
			if self._gazebo_robots[robot_id]['model'].is_received():
				state = self._gazebo_robots[robot_id]['model'].get()
				
				robot_models_msg.name.append(state.model_name)
				robot_models_msg.pose.append(state.pose)
				robot_models_msg.twist.append(state.twist)
				
		self._gazebo_robot_models_publisher.publish(robot_models_msg)		
		
		picking_models_msg = ModelStates()
		
		for picking_id in self._gazebo_objects:
			if self._gazebo_objects[picking_id]['model'].is_received():
				state = self._gazebo_objects[picking_id]['model'].get()
				
				picking_models_msg.name.append(state.model_name)
				picking_models_msg.pose.append(state.pose)
				picking_models_msg.twist.append(state.twist)
				
		self._gazebo_object_models_publisher.publish(picking_models_msg)		
		
		
		robot_links_msg = LinkStates()
		
		for robot_id in self._gazebo_robots:
			for link_id in self._gazebo_robots[robot_id]['links']:
				if self._gazebo_robots[robot_id]['links'][link_id].is_received():
					state = self._gazebo_robots[robot_id]['links'][link_id].get()
					
					robot_links_msg.name.append(state.link_name)
					robot_links_msg.pose.append(state.pose)
					robot_links_msg.twist.append(state.twist)
				
		self._gazebo_robot_links_publisher.publish(robot_links_msg)		
		
		picking_links_msg = LinkStates()
		
		for picking_id in self._gazebo_objects:
			for link_id in self._gazebo_objects[picking_id]['links']:
				if self._gazebo_objects[picking_id]['links'][link_id].is_received():
					state = self._gazebo_objects[picking_id]['links'][link_id].get()
					
					picking_links_msg.name.append(state.link_name)
					picking_links_msg.pose.append(state.pose)
					picking_links_msg.twist.append(state.twist)
				
		self._gazebo_object_links_publisher.publish(picking_links_msg)		
		#print self._gazebo_robots
		#print self._gazebo_objects
		
					
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
		if len(self._current_picks) > 0:
			self.switchToState(State.READY_STATE)
		
		return
	
	
	def readyState(self):
		'''
			Actions performed in ready state
		'''
		if len(self._current_picks) == 0:
			self.switchToState(State.STANDBY_STATE)
			return
			
		picks_to_remove = []

		# for every pick we have to move the object to same pose than the robot + offset
		for pick in self._current_picks:
			
			# place the object
			if self._current_picks[pick].isPlaceRequired():
				object_link = self._current_picks[pick].getObjectLink()
				object_model = self._current_picks[pick].getObjectModel()
				robot_model = self._current_picks[pick].getRobotModel()
				robot_link = self._current_picks[pick].getRobotLink()

				# Placing the object on the ground
				# Get the pose of the robot link
				robot_link_state = copy.deepcopy(self._gazebo_robots[robot_model]['links'][robot_link].get())
				# Set the position.z to zero 
				robot_link_state.pose.position.z = 0.0
				# change the link name
				robot_link_state.link_name = object_link
				
				if not self.setGazeboLinkState(robot_link_state):
					rospy.logerr_throttle(5, "%s::readyState: Error placing the pick %s"%(self.node_name, pick))

				#
				# Get link properties of the object
				properties = self._gazebo_objects[object_model]['links'][object_link].get_properties()

				# set link properties of the object
				# 	bring gravity bck(TODO: from all the links of the object)
				set_link_properties_srv = SetLinkPropertiesRequest()
				set_link_properties_srv.link_name = object_link
				set_link_properties_srv.com = properties.com
				set_link_properties_srv.gravity_mode = True
				set_link_properties_srv.mass = properties.mass
				set_link_properties_srv.ixx = properties.ixx
				set_link_properties_srv.ixy = properties.ixy
				set_link_properties_srv.ixz = properties.ixz
				set_link_properties_srv.iyy = properties.iyy
				set_link_properties_srv.iyz = properties.iyz
				set_link_properties_srv.izz = properties.izz
				
				if not self.setGazeboLinkProperties(set_link_properties_srv):
					rospy.logerr_throttle(5, "Error setting gazebo link properties of %s"%object_link)
				else:
					# remove the pick
					picks_to_remove.append(pick)

			else:	
				robot_model = self._current_picks[pick].getRobotModel()
				robot_link = self._current_picks[pick].getRobotLink()
				object_link = self._current_picks[pick].getObjectLink()
				
				# Get the pose of the robot link
				robot_link_state = copy.deepcopy(self._gazebo_robots[robot_model]['links'][robot_link].get())
				# Set the pose of the object link like the robot one
				self._current_picks[pick].applyTransformToPose(robot_link_state.pose)
				# change the link name
				robot_link_state.link_name = object_link
				
				if not self.setGazeboLinkState(robot_link_state):
					rospy.logerr_throttle(5, "%s::readyState: Error updating the pick %s"%(self.node_name, pick))
		
		# Remove picks if requested
		for rm_pick in picks_to_remove:
			self._current_picks.pop(pick, None)
		
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

		pick_states_msg = PickStates()
		current_picks = copy.deepcopy(self._current_picks)
		for pick_id in current_picks:
			pick_state = PickState()
			pick_state.id = pick_id
			pick_state.object_link = current_picks[pick_id].getObjectLink()
			pick_state.robot_link = current_picks[pick_id].getRobotLink()
			pick_states_msg.picks.append(pick_state)
		self._pick_state_publisher.publish(pick_states_msg)

		self.t_publish_state = threading.Timer(self.publish_state_timer, self.publishROSstate)
		self.t_publish_state.start()
	
	
	def modelStatesCb(self, msg):
		'''
			Callback for gazebo model states
			@param msg: received message
			@type msg: gazebo_msgs/ModelStates
		'''
		for i in range(len(msg.name)):
			if msg.name[i] in self._gazebo_robots:
				model_state = ModelState()
				model_state.model_name = msg.name[i]
				model_state.pose = msg.pose[i]
				model_state.twist = msg.twist[i]
				self._gazebo_robots[msg.name[i]]['model'].update(model_state, rospy.Time.now())
				
			elif msg.name[i] in self._gazebo_objects:
				model_state = ModelState()
				model_state.model_name = msg.name[i]
				model_state.pose = msg.pose[i]
				model_state.twist = msg.twist[i]
				self._gazebo_objects[msg.name[i]]['model'].update(model_state, rospy.Time.now())
				
				
	def linkStatesCb(self, msg):
		'''
			Callback for gazebo link states
			@param msg: received message
			@type msg: gazebo_msgs/LinkStates
		'''

		for i in range(len(msg.name)):
			model_id = msg.name[i].split('::')[0]
			
			# Adds dynamically all the links for all the models
			if model_id in self._gazebo_robots:
				link_state = LinkState()
				link_state.link_name = msg.name[i]
				link_state.pose = msg.pose[i]
				link_state.twist = msg.twist[i]

				
				# already exist?
				if link_state.link_name in self._gazebo_robots[model_id]['links']:
					self._gazebo_robots[model_id]['links'][link_state.link_name].update(link_state, rospy.Time.now())
				else:
					rospy.loginfo('%s::linkStatesCb: adding new link %s', self.node_name, link_state.link_name)
					self._gazebo_robots[model_id]['links'][link_state.link_name] = GazeboLinkState()
					self._gazebo_robots[model_id]['links'][link_state.link_name].update(link_state, rospy.Time.now())
			elif model_id in self._gazebo_objects:
				link_state = LinkState()
				link_state.link_name = msg.name[i]
				link_state.pose = msg.pose[i]
				link_state.twist = msg.twist[i]
				# already exist?
				if link_state.link_name in self._gazebo_objects[model_id]['links']:
					self._gazebo_objects[model_id]['links'][link_state.link_name].update(link_state, rospy.Time.now())
				else:
					rospy.loginfo('%s::linkStatesCb: adding new link %s', self.node_name, link_state.link_name)
					self._gazebo_objects[model_id]['links'][link_state.link_name] = GazeboLinkState()
					self._gazebo_objects[model_id]['links'][link_state.link_name].update(link_state, rospy.Time.now())

	def pickServiceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: robotnik_base_hw_sim/Pick
		'''
		pick_id = '%s->%s'%(req.robot_model, req.object_model)
		#
		# check that model and links exists
		if req.object_model in self._gazebo_objects:
			link='%s::%s'%(req.object_model, req.object_link)
			if link in self._gazebo_objects[req.object_model]['links']:
				if not self._gazebo_objects[req.object_model]['links'][link].is_received():
					return False, "Link %s is not being received anymore"%req.object_link
				
			else:
				return False, "Link %s doesn't exist"%req.object_link
			
		else:
			return False, "Model %s doesn't exist"%req.object_model
		
		if req.robot_model in self._gazebo_robots:
			link='%s::%s'%(req.robot_model, req.robot_link)
			if link in self._gazebo_robots[req.robot_model]['links']:
				if not self._gazebo_robots[req.robot_model]['links'][link].is_received():
					return False, "Link %s is not being received anymore"%req.robot_link
				
			else:
				return False, "Link %s doesn't exist"%req.robot_link
			
		else:
			return False, "Model %s doesn't exist"%req.robot_model
		
		#
		# Check is not already pick
		
		#
		# Get link properties of the object
		object_link='%s::%s'%(req.object_model, req.object_link)
		ret, properties = self.getGazeboLinkProperties(object_link)
		
		if not ret:
			return False, "Error getting gazebo link properties of %s"%object_link
		
		# save the properties of the object link
		self._gazebo_objects[req.object_model]['links'][object_link].update_properties(properties)
		
		# set link properties of the object
		# 	remove gravity (TODO: from all the links of the object)
		set_link_properties_srv = SetLinkPropertiesRequest()
		set_link_properties_srv.link_name = object_link
		set_link_properties_srv.com = properties.com
		set_link_properties_srv.gravity_mode = False
		set_link_properties_srv.mass = properties.mass
		set_link_properties_srv.ixx = properties.ixx
		set_link_properties_srv.ixy = properties.ixy
		set_link_properties_srv.ixz = properties.ixz
		set_link_properties_srv.iyy = properties.iyy
		set_link_properties_srv.iyz = properties.iyz
		set_link_properties_srv.izz = properties.izz
		
		if not self.setGazeboLinkProperties(set_link_properties_srv):
			return False, "Error setting gazebo link properties of %s"%object_link
		
		
		#
		# get link state of the robot
		robot_link='%s::%s'%(req.robot_model, req.robot_link)
		'''
		robot_link_state = self._gazebo_robots[req.robot_model]['links'][robot_link].get()
		# adding pose transform to robot current one
		robot_link_state.pose.position.x += req.pose.position.x
		robot_link_state.pose.position.y += req.pose.position.y
		robot_link_state.pose.position.z += req.pose.position.z
		robot_link_state.pose.orientation.x += req.pose.orientation.x
		robot_link_state.pose.orientation.y += req.pose.orientation.y
		robot_link_state.pose.orientation.z += req.pose.orientation.z
		robot_link_state.link_name = '%s::%s'%(req.object_model, req.object_link)
		
		# place the obj into the same pose/orientation of the robot link and different Z
		if not self.setGazeboLinkState(robot_link_state):
			return False, "Error setting gazebo link state of %s"%robot_link
		'''
		
		# adding new pick
		self._current_picks[pick_id] = GazeboPickAndPlace(robot_link=robot_link, object_link = object_link, transform_pose = req.pose)
		
		rospy.loginfo('%s:pickServiceCb: attach %s::%s into %s::%s ', self.node_name, req.object_model, req.object_link, req.robot_model, req.robot_link  )	
	
		return True, "OK" 


	def placeServiceCb(self, req):
		'''
			ROS service server
			@param req: Required action
			@type req: robotnik_base_hw_sim/Place
		'''
		# locate current pick 
		pick_id = '%s->%s'%(req.robot_model, req.object_model)

		if not pick_id in self._current_picks:
			return False, 'The pick %s does not exist'%pick_id
		else:
			self._current_picks[pick_id].place()

		rospy.loginfo('%s:placeServiceCb: placing %s from %s', self.node_name, req.object_model, req.robot_model )	

		return True, "OK" 
		
	
	def getGazeboLinkProperties(self, link):
		'''
			Calls the ros service to get the link properties
			@param link as string, link name
			@return True/False, gazebo_msg/GetLinkPropertiesResponse
		'''
		try:
			ret = self.get_link_properties_service_client(link)
		except rospy.ServiceException, e:
			rospy.logerr('%s::getGazeboLinkProperties: %s', self.node_name, e)
			return False, None
		
		
		return True, ret
	
	
	def setGazeboLinkProperties(self, link_properties):
		'''
			Calls the ros service to set the link properties
			@param link_properties as gazebo_msg/SetLinkPropertiesRequest
			@return True/False, 
		'''
		
		try:
			ret = self.set_link_properties_service_client(link_properties)
		except rospy.ServiceException, e:
			rospy.logerr('%s::setGazeboLinkProperties: %s', e)
			return False
		
		if not ret.success:
			rospy.logerr('%s::setGazeboLinkProperties: error setting properties of %s: %s', self.node_name, link_properties.link_name, ret.status_message)
			return False
			
		return True
	
	
	def setGazeboLinkState(self, link_state):
		'''
			Calls the ros service to set the link state
			@param link_state as gazebo_msg/LinkState
			@return True/False, 
		'''	
		try:
			ret = self.set_link_state_service_client(link_state)
		except rospy.ServiceException, e:
			rospy.logerr('%s::setGazeboLinkState: %s', self.node_name, e)
			return False
		
		if not ret.success:
			rospy.logerr('%s::setGazeboLinkState: error setting state of %s', link_state.link_name, ret.status_message)
			return False
			
		return True
	
	
def main():

	rospy.init_node("elevator_fake_pickup_gazebo")
	
	
	_name = rospy.get_name().replace('/','')
	
	arg_defaults = {
	  'topic_state': 'state',
	  'desired_freq': DEFAULT_FREQ,
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
			
	
	rc_node = ElevatorFakePickup(args)
	
	rospy.loginfo('%s: starting'%(_name))

	rc_node.start()


if __name__ == "__main__":
	main()
