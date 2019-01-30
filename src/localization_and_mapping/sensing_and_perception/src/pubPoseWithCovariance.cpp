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
    ros::Publisher posePub;
    ros::Subscriber refreshCovariance;
    ros::Subscriber poseSub;
    geometry_msgs::PoseWithCovarianceStamped PoseWCS;
    
    float current_x, current_y, current_theta;
    
    public:
    pubOdomWithCovariance () {
		ss << robotId;			
        posePub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_"+ss.str()+"/pose_channel", 1);
        
        refreshCovariance = n.subscribe("/amcl_pose",1, &pubOdomWithCovariance::refreshCovarianceCallback, this);
        poseSub = n.subscribe("/odom",1, &pubOdomWithCovariance::refreshPoseCallback, this);
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
    
    pubOdomWithCovariance publishOdometryWithCovarianve;
    
    ros::spin();
    
    return 0;
}
