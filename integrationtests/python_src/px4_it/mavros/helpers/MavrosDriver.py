#!/usr/bin/env python

"""
Mavros Driver
"""

from __future__ import print_function

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped , Vector3Stamped
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse, CommandBool, CommandBoolRequest, CommandBoolResponse, CommandTOL, CommandTOLRequest
import time
from tf.transformations import *
import numpy as np
import threading


### constant
RATE_STATE = 1 # state rate subscription


### class for mavros subscription ###
class MavrosDriver():
    def __init__(self, nh):
   
        # lock is needed for switching between states
        self._lock =  threading.Lock()
        
        # state subscriber 
        self._rate_state = rospy.Rate(RATE_STATE)
        self.current_state = State()
        rospy.Subscriber('/mavros/state', State , self._current_state_cb)
        
        # wait until connection with FCU 
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.Rate(20)     
        print( 'FCU connection successful')

        
        # subscriber,
        self.local_pose = PoseStamped()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._local_pose_cb)
        self.local_vel = TwistStamped()
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self._local_vel_cb)
             
        #publisher
        self._pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self._pose_msg = PoseStamped()
        self._pose_state = "posctr"
         # vel 
        self._vel_pub =  rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10 )
        self._vel_msg = TwistStamped()
        self._vel_state = "velctr"
        # acc
        self._accel_pub = rospy.Publisher('mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
        self._accel_msg = Vector3Stamped()
        self._accel_state = "accelctr"
        
        # initialize with position
        self.set_state("posctr")
        
        # initial desired position: position and orientation
        ps = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0]
        self.set_msg(ps)
        
        
           
    ### callback functions ###   
    def _current_state_cb(self, data):
        self.current_state = data       
        
    def _local_pose_cb(self, data):
        self.local_pose = data
        
    def _local_vel_cb(self, data):
        self.local_vel = data
        

    ### service function ###        
    def set_mode(self, mode):
        if not self.current_state.connected:
            print( "No FCU connection")
        
        elif self.current_state.mode == mode:
            print( "Already in {0} mode").format(mode)
        
        else:

            # wait for service
            rospy.wait_for_service("mavros/set_mode")   

            # service client
            set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
                  
            # set request object
            req = SetModeRequest()
            req.custom_mode = mode
     
            # zero time 
            t0 = rospy.get_time()
                   
            # check response
            while not rospy.is_shutdown() and (self.current_state.mode != req.custom_mode):
                if rospy.get_time() - t0 > 0.005: # check every 0.005 seconds
                
                    try:
                        # request 
                        set_mode.call(req)
                        
                    except rospy.ServiceException, e:
                        print( "Service did not process request: {0}").format(e)
  
                    t0 = rospy.get_time()
                                   
            print("Mode: {0} established").format(self.current_state.mode)

    

    def set_state(self, arg):
        
        self._lock.acquire()
        if arg == "posctr":
            self.state = self._pose_state
            self.msg = self._pose_msg
            self.pub = self._pose_pub
            
        elif arg == "velctr":
            self.state = self._vel_state
            self.msg = self._vel_msg
            self.pub = self._vel_pub
            
        elif arg == "accelctr":
            self.state = self._accel_state
            self.msg = self._accel_msg
            self.pub = self._accel_pub
            
        else:
            print( "this state is not supported")
        self._lock.release()
        
    
    def set_msg(self, arg):
        
        if self.state == "posctr":
            if len(arg) == 7:
                self._lock.acquire()
                self._pose_msg.pose.position.x = arg[0]
                self._pose_msg.pose.position.y = arg[1]
                self._pose_msg.pose.position.z = arg[2]
    
                self._pose_msg.pose.orientation.x = arg[3]
                self._pose_msg.pose.orientation.y = arg[4]
                self._pose_msg.pose.orientation.z = arg[5]
                self._pose_msg.pose.orientation.w = arg[6]   
                self._lock.release()
            else:
                print( "posctr requires array of len 7")
                
            
        elif self.state == "velctr":
            if len(arg) == 3:
                self._lock.acquire()
                self._vel_msg.twist.linear.x = arg[0]
                self._vel_msg.twist.linear.y = arg[1]
                self._vel_msg.twist.linear.z = arg[2]
                
                #self._vel_msg.twist.angular.x = arg[3]
                #self._vel_msg.twist.angular.y = arg[4]
                #self._vel_msg.twist.angular.z = arg[5]
                self._lock.release()
            else:
                print( "velctr requires array of len 3")
                
                
        elif self.state == "accelctr":
            if len(arg) == 3:
                self._lock.acquire()
                self._accel_msg.vector.x = arg[0]
                self._accel_msg.vector.y = arg[1]
                self._accel_msg.vector.z = arg[2]
                self._lock.acquire()

            else:
                print( "accelctr requires array of len 3")
                
                
           
    def arm(self, do_arming):
        
        if self.current_state.armed and do_arming:
            print( "already armed")
            
        else:
            # wait for service
            rospy.wait_for_service("mavros/cmd/arming")   
                        
            # service client
            set_arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
                       
            # set request object
            req = CommandBoolRequest()
            req.value = do_arming
                       
             # zero time 
            t0 = rospy.get_time()
                       
            # check response
            if do_arming:
                while not rospy.is_shutdown() and not self.current_state.armed:
                    if rospy.get_time() - t0 > 2.0: # check every 2 seconds
                    
                        try:
                            # request 
                            set_arm.call(req)
                            
                        except rospy.ServiceException, e:
                            print( "Service did not process request: {0}").format(e)
      
                        t0 = rospy.get_time()
                
                print("armed: {0}").format(self.current_state.armed)
                
            else: 
                while not rospy.is_shutdown() and self.current_state.armed:
                    if rospy.get_time() - t0 > 0.5: # check every .5 seconds
                    
                        try:
                            # request 
                            set_arm.call(req)
                            
                        except rospy.ServiceException, e:
                            print( "Service did not process request: {0}").format(e)
      
                        t0 = rospy.get_time()
                    
                
            
    def land(self):  
        if not self.current_state.armed:    
            print( "not armed yet")
            
        else:
            
            # wait for service
            rospy.wait_for_service("mavros/cmd/land")   
               
            # service client
            set_rq = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)
                       
            # set request object
            req = CommandTOLRequest()
            req.yaw = 0.0
            req.latitude = 0.0
            req.longitude = 0.0
            req.altitude = 0.0
            req.min_pitch = 0.0
                       
            #zero time 
            t0 = rospy.get_time()
                       
            # check response
            while self.current_state.armed:
                if rospy.get_time() - t0 > 2.0: # check every 2 seconds
                
                    try:
                        # request 
                        set_rq.call(req)
                        
                    except rospy.ServiceException, e:
                        print( "Service did not process request: {0}").format(e)
  
                    t0 = rospy.get_time()
                                              
            print( "landed savely")
            
            
