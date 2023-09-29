#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
import moveit_commander
import moveit_msgs.msg

# buttons GPIO pins 
#   button 1 - gpio 11
#   button 2 - gpio 12

class sick_zone():
    
    def __init__(self, moveGroup):
     
        # define publisher
        # rospy.Publisher("topic_name", varType, queue_size)
        self.sub = rospy.Subscriber('/sick_safetyscanners/scan', LaserScan, self.laser_callback)
        self.state = False
        self.ctrl_c = False
        self.mg = moveGroup
        rospy.on_shutdown(self.shutdownhook)
    
    def laser_callback(self, msgScan):
        # the code that is executed when data is received
        # turn on LED
        razdalje=msgScan.ranges[300:1300]
        
        count = 0
        for r in razdalje:
            #print(r)
            if(r < 0.3):
                self.state = True
                count += 1
                self.mg.stop()
                 
        #print(self.state)
        if not count:
            self.state = False         
                   
       
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        # this code is run at ctrl + c

        # clear all GPIO settings
        self.ctrl_c = True
            
if __name__ == '__main__':
    # initialise node
    rospy.init_node('sick_listener', anonymous=True)
    sick = sick_zone()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass