#!/usr/bin/env python


import rospy
import threading
import sys
import signal
import MavrosDriver


# rate at which msg are sent ot mavros
run_event = threading.Event() # signal flag for running threads
msg_thread = threading.Thread()
def pub_msg(msg_rate,driver):
    
    # frequency of publishing
    # minimum is 50hz to remain in offboard
    rate = rospy.Rate(msg_rate)
    
    # publish desired pose
    while run_event.is_set() and not rospy.is_shutdown():
        #driver.pub.publish(driver.msg)
        rate.sleep()


    
def run_tests(arg):
    
    """ Initialization """
    
    # catch ctrl-c
    
    
    # interaction test node
    nh = rospy.init_node('integration_tests', anonymous=True)
    
    # create driver for receiving and sending mavros msg
    drv = MavrosDriver.MavrosDriver(nh)
    
    # set event flag
    run_event.set()
   
    # start thread that just publishes msg to mavros 
    msg_rate = 50
    msg_thread = threading.Thread(target=pub_msg, args=(msg_rate, drv))
    msg_thread.setDaemon(True)
    #
    msg_thread.start()

   



if __name__ == '__main__':
    
    arg = ["LandingTest"]
    try: 
        run_tests(arg)
        sys.exit()
        
    except rospy.ROSInterruptException:
        pass