#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = rospy.Publisher('/odometry_fused', Odometry, queue_size=10) 

def callback(data):
    global pub
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    new_data= Odometry()

    new_data.header = data.header
    new_data.header.frame_id="odom_combined"
    new_data.pose = data.pose
    new_data.child_frame_id="base_link"
    # covar = list(data.pose.covariance) 
    # data.header.frame_id="/base_footprint"
    # covar[0]=1
    # covar[7]=1
    # covar[14]=1
    # covar[21]=10000
    # covar[28]=10000
    # covar[35]=10000
    # data.pose.covariance=tuple(covar)
    pub.publish(new_data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('add_covariance_node', anonymous=True)

    rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
