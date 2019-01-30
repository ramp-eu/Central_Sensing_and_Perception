import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, PointCloud, LaserScan
import laser_geometry.laser_geometry as lg
import math
import tf
from mapupdates.msg import NewObstacles
from geometry_msgs.msg import PoseWithCovarianceStamped, Point 
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node("laserscan_to_pointcloud")
listener = tf.TransformListener()
lp = lg.LaserProjection()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)
point_pub = rospy.Publisher("global_points", NewObstacles, queue_size=1)


# position and orientation of robot
def get_rotation (msg):
    global roll, pitch, yaw, tfx, tfy
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    tfx = msg.pose.pose.position.x
    tfy = msg.pose.pose.position.y
    
#sub = rospy.Subscriber ('/robot_opil_v1/pose_channel', PoseWithCovarianceStamped, get_rotation)

def scan_cb(msg):
    # convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)
    pc_pub.publish(pc2_msg)

    point_msg = NewObstacles()
#    print msg.header.frame_id
    # convert PointCloud2 to a generator of the individual local points 
    point_generator = pc2.read_points(pc2_msg)

    listener.waitForTransform("/map", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    position,quaternion = listener.lookupTransform("/map", msg.header.frame_id, rospy.Time(0))
#    print (position, quaternion)
    tfx = position[0]
    tfy = position[1]
#    print(tfx,tfy)
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    globx = []
    globy = []
    for point in point_generator:     
        globx.append(Local_to_global_x(tfx, yaw, point[0], point[1]))
        globy.append(Local_to_global_y(tfy, yaw, point[0], point[1]))
    r = len(globx)
    for i in range(r):
        point_msg.x.append(globx[i])
        point_msg.y.append(globy[i])

    point_pub.publish(point_msg)

# convert local point to global point; Rx and Rth are part of robot position; pointX and pointY are x, y of local point wich we convert to global point
def Local_to_global_x(Rx, Rth, pointX, pointY):
    globalX = Rx + math.cos(Rth) * pointX - math.sin(Rth) * pointY
    return globalX

# convert local point to global point; Ry and Rth are part of robot position; pointX and pointY are x, y of local point wich we convert to global point
def Local_to_global_y(Ry, Rth, pointX, pointY):
    globalY = Ry + math.sin(Rth) * pointX + math.cos(Rth) * pointY
    return globalY

rospy.Subscriber("/base_scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()
