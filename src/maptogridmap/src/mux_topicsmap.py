#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid


class intention_class():
    
    def __init__(self):
        rospy.init_node('topic_muxermap')
        self.last_state = 0
        self.pub= rospy.Publisher('/map/realmap',OccupancyGrid, queue_size=10)
        self.sub= rospy.Subscriber('/map',OccupancyGrid, self.map_callback)
        self.sub= rospy.Subscriber('/map/do_serve', Bool, self.service_callback)
        self.strings = ["map/dummymap", "/map/realmap"]
        self.map=OccupancyGrid()

    def map_callback(self, data):        
        self.map=data

    def service_callback(self, data): 
        #if self.last_state==0 and data.data ==1:
        self.pub=rospy.Publisher(self.strings[data.data],OccupancyGrid, queue_size=10)
        self.pub.publish(self.map)
        #    self.last_state= 1
        #if  data.data==0:
        #    self.last_state =0

    def run(self): 
        try:
            while not rospy.is_shutdown():
                #self.pub.publish(self.map)
                r=rospy.Rate(1)
                r.sleep()
        except rospy.ROSInterruptException:                        
            pass      
                                      

if __name__ == '__main__':         
    mux = intention_class()                                                             
    try:
        mux.run()
    except rospy.ROSInterruptException:
        pass
