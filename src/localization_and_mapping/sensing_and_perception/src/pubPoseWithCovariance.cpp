#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <sstream>

using namespace std;
int robotId;

std::stringstream ss;
std::string base_frame="/base_link";
std::string map_frame="/map";
std::string amcl_topic="/amcl_pose";


class pubOdomWithCovariance {
    private:
    ros::NodeHandle n;
    ros::Subscriber refreshCovariance;
    ros::Subscriber poseSub;
    
    float current_x, current_y, current_theta;
    
    public:
    ros::Publisher posePub;
    geometry_msgs::PoseWithCovarianceStamped PoseWCS;
    pubOdomWithCovariance () {
		ss << robotId;			
        posePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_"+ss.str()+"/pose_channel", 1);
        
        refreshCovariance = n.subscribe(amcl_topic,1, &pubOdomWithCovariance::refreshCovarianceCallback, this);
//        poseSub = n.subscribe("/odom",1, &pubOdomWithCovariance::refreshPoseCallback, this);
    }
    
    void refreshPoseCallback(const nav_msgs::Odometry &msg) {
    
        this->PoseWCS.header = msg.header;
        this->PoseWCS.header.frame_id = map_frame;
//        this->PoseWCS.pose.pose.position.x = transform.getOrigin().x();;
//        this->PoseWCS.pose.pose.position.y = transform.getOrigin().y();
        this->PoseWCS.pose.pose.position.z = 0;
//        this->PoseWCS.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
//        posePub.publish(this->PoseWCS);
    }
    
    void refreshCovarianceCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
        this->PoseWCS.pose.covariance = msg.pose.covariance;
    }
    
};


int main(int argc, char **argv) {

	if(argc < 2){
	  	ROS_ERROR("Robot ID MUST be specified!");
		  return -1;	
	}
	robotId = atoi(argv[1]); 
	ROS_INFO("Hello, I am robot %d",robotId);
  ros::init(argc, argv, "PoseWithCovariance");
	ros::NodeHandle nh;

  nh.getParam("/publishPoseWithCovariance/amcl_topic", amcl_topic);
  nh.getParam("/publishPoseWithCovariance/map_frame", map_frame);
  nh.getParam("/publishPoseWithCovariance/base_frame", base_frame);

  std::cout << map_frame <<std::endl;
  
  pubOdomWithCovariance publishOdometryWithCovariance;
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  double yaw, pitch, roll, tfx, tfy;
  tf_listener.waitForTransform(map_frame, base_frame, ros::Time::now(), ros::Duration(3.0));

  ros::Rate rate(10.0);

  while (nh.ok()) {
    ros::spinOnce(); 
    
    try {
        tf_listener.waitForTransform(map_frame, base_frame, ros::Time(0), ros::Duration(1.0));
        tf_listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);

				transform.getBasis().getRPY(roll, pitch, yaw);
				publishOdometryWithCovariance.PoseWCS.header.stamp = ros::Time::now();
				publishOdometryWithCovariance.PoseWCS.header.frame_id = map_frame;
				publishOdometryWithCovariance.PoseWCS.pose.pose.position.x = transform.getOrigin().x();
				publishOdometryWithCovariance.PoseWCS.pose.pose.position.y = transform.getOrigin().y();
				publishOdometryWithCovariance.PoseWCS.pose.pose.position.z = 0;
				publishOdometryWithCovariance.PoseWCS.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
				publishOdometryWithCovariance.posePub.publish(publishOdometryWithCovariance.PoseWCS);
    }
    catch (tf::TransformException ex) {
        ROS_INFO("Pose With Covariance: %s", ex.what());
    }
    
 	  rate.sleep();
  }
   
    return 0;
}
