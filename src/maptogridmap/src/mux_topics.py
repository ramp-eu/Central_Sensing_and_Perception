#!/usr/bin/env python
import rospy
from maptogridmap.msg import Gridmap
from std_msgs.msg import Bool

class mux_class():
    def __init__(self):
        rospy.init_node('topic_muxer')
        self.pub= rospy.Publisher('/map/realtopology',Gridmap, queue_size=10)
        self.sub= rospy.Subscriber('/map/topology',Gridmap, self.map_callback)
        self.sub= rospy.Subscriber('/map/do_serve',Bool, self.service_callback)
        self.strings = ["/map/dummytopology", "/map/realtopology"]
        self.map=Gridmap()

    def map_callback(self, data):        
        self.map=data

    def service_callback(self, data): 
        #if self.last_state==0 and data.data ==1:
        self.pub=rospy.Publisher(self.strings[data.data],Gridmap, queue_size=10)
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
    mux = mux_class()                                                             
    try:
        mux.run()
    except rospy.ROSInterruptException:
        pass
