#include <sys/time.h>

#include "ros/ros.h"

#include <maptogridmap/GetMap.h>
#include <visualization_msgs/Marker.h>

class VisualizationPublisherGML
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher gridmapls_pub;



    visualization_msgs::Marker gridmapls;

  VisualizationPublisherGML(ros::NodeHandle n) :
      nh_(n),  target_frame_("map") 
  {

		gridmapls_pub=nh_.advertise<visualization_msgs::Marker>("/gridmap_markerListener",10);

    gridmapls.header.frame_id = target_frame_;
    gridmapls.header.stamp = ros::Time::now();
    gridmapls.ns =  "maplistener";
    gridmapls.action = visualization_msgs::Marker::ADD;
    gridmapls.pose.orientation.w  = 1.0;
    gridmapls.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    gridmapls.scale.x = 0.25; 
    gridmapls.scale.y = 0.25; 
    gridmapls.color.r = 0.;
    gridmapls.color.g = 0.;
    gridmapls.color.b = 0.8;
    gridmapls.color.a = 1.0;
      

  }

  void visualizationduringmotion();


};

void gridmapCallback(const maptogridmap::GridmapConstPtr& gmMsg)
{
	int width = gmMsg->info.width;
	int height = gmMsg->info.height;
  double resolution = gmMsg->info.resolution;
  printf("listening message occupancy map data: res=%f, width=%d, height=%d\n", resolution, width, height);
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "maplistener");
  ros::NodeHandle nh;
  VisualizationPublisherGML visualGML(nh);
  
  maptogridmap::GetMap map;
  ros::service::call("grid_map",map);
 
  double resolution=map.response.map.info.resolution;
  int width=map.response.map.info.width;
  int height=map.response.map.info.height;
  printf("listening service occupancy map data: res=%f, width=%d, height=%d\n", resolution, width, height);

  ros::Subscriber gml_sub = nh.subscribe("topology",1,gridmapCallback);


  ros::Rate rate(10.0);


  while (nh.ok()) {


    ros::spinOnce(); 

	  visualGML.visualizationduringmotion();	
    
    
	  rate.sleep();
  }
  return 0;
}


void VisualizationPublisherGML::visualizationduringmotion(){

  maptogridmap::GetMap map;
  ros::service::call("grid_map",map);
 
  double resolution=map.response.map.info.resolution;
  int width=map.response.map.info.width;
  int height=map.response.map.info.height;
  printf("listening occupancy map data: res=%f, width=%d, height=%d\n", resolution, width, height);

      gridmapls.points.clear();
      geometry_msgs::Point p; 
      int sizex=width;
      int sizey=height;
			for (int i=0; i<sizex; i++){
				for (int j=0; j<sizey; j++){
						p.x=map.response.map.gmc[i*sizey+j].x;
            p.y=map.response.map.gmc[i*sizey+j].y;
//						printf("(%f,%f) (%d,%d)\t",p.x,p.y,i,j);
					if ((map.response.map.gmc[i*sizey+j].occupancy>0)){
						p.x=map.response.map.gmc[i*sizey+j].x;
            p.y=map.response.map.gmc[i*sizey+j].y;
						gridmapls.points.push_back(p);
//						printf("(%f,%f) (%d,%d)\t",p.x,p.y,i,j);
					}
				}
				printf("\n");
			}
			gridmapls_pub.publish(gridmapls);
//			printf("num of points %d\n and cells %d\n",gridmapls.points.size(),map.response.map.gmc.size());


}


