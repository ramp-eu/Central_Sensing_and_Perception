#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <sstream>

using namespace std;
int robotId;

std::stringstream ss;

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
        
        refreshCovariance = n.subscribe("/amcl_pose",1, &pubOdomWithCovariance::refreshCovarianceCallback, this);
//        poseSub = n.subscribe("/odom",1, &pubOdomWithCovariance::refreshPoseCallback, this);
    }
    
    void refreshPoseCallback(const nav_msgs::Odometry &msg) {
        tf::StampedTransform transform;
        tf::TransformListener listener;
        double yaw, pitch, roll;
        try {
            listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }
        transform.getBasis().getRPY(roll, pitch, yaw);
    
        this->PoseWCS.header = msg.header;
        this->PoseWCS.header.frame_id = "map";
        this->PoseWCS.pose.pose.position.x = transform.getOrigin().x();;
        this->PoseWCS.pose.pose.position.y = transform.getOrigin().y();
        this->PoseWCS.pose.pose.position.z = 0;
        this->PoseWCS.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        posePub.publish(this->PoseWCS);
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
  
  pubOdomWithCovariance publishOdometryWithCovariance;
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  double yaw, pitch, roll, tfx, tfy;
  tf_listener.waitForTransform("map", "/base_link", ros::Time::now(), ros::Duration(3.0));

  ros::Rate rate(10.0);

  while (nh.ok()) {
    ros::spinOnce(); 
    tf_listener.lookupTransform("map", "/base_link", ros::Time(0), transform);	
		transform.getBasis().getRPY(roll, pitch, yaw);
		publishOdometryWithCovariance.PoseWCS.header.stamp = ros::Time::now();
    publishOdometryWithCovariance.PoseWCS.header.frame_id = "map";
    publishOdometryWithCovariance.PoseWCS.pose.pose.position.x = transform.getOrigin().x();
    publishOdometryWithCovariance.PoseWCS.pose.pose.position.y = transform.getOrigin().y();
    publishOdometryWithCovariance.PoseWCS.pose.pose.position.z = 0;
    publishOdometryWithCovariance.PoseWCS.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		publishOdometryWithCovariance.posePub.publish(publishOdometryWithCovariance.PoseWCS);
    
 	  rate.sleep();
  }
   
    return 0;
}
