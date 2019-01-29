#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from maptogridmap.msg import Gridmap

class map_class():
    def __init__(self):
        rospy.init_node('map_show')
        self.marker_publisher = rospy.Publisher('gridmap_markerListener', Marker, queue_size=10)
        self.map_sub= rospy.Subscriber('/map/realtopology',Gridmap, self.map_callback)
      
    def map_callback(self, scan):
        marker=Marker()
        marker.header.frame_id="map"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = .25
        marker.scale.y = .25
        marker.color.a = 1
        marker.color.r = 0
        marker.color.g = 0.0
        marker.color.b = 1
        width=scan.info.width;
        height=scan.info.height;
        #print "listening occupancy map data: res=%f, width=%d, height=%d\n", resolution, width, height;
        for i in range(width*height):
        	p=Point()
        	p.x=scan.x[i]
        	p.y=scan.y[i]
        	if scan.occupancy[i]==0:
        		marker.points.append(p)
#        for i in range(width):
#            for j in range(height):
#                p=Point()
#                p.x=scan.x[i*height+j]
#                p.y=scan.y[i*height+j]
#                if scan.occupancy[i*height+j]>0:
#                    marker.points.append(p)
        self.marker_publisher.publish(marker)            

    def run(self): 
        try:
            while not rospy.is_shutdown():
                r=rospy.Rate(30)
                r.sleep()
        except rospy.ROSInterruptException:                        
            pass      
                                      

if __name__ == '__main__':         
    map_service = map_class()                                                             
    try:
        map_service.run()
    except rospy.ROSInterruptException:
        pass
