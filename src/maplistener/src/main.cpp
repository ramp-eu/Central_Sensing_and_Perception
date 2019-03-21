#include <sys/time.h>
#include <iostream>
#include <string>
#include "ros/ros.h"
//#include <algorithm>

//#include <maptogridmap/GetMap.h>
#include <maptogridmap/Gridmap.h>
#include <maptogridmap/Graph.h>
#include <maptogridmap/Nodes.h>
#include <maptogridmap/Edges.h>
#include <mapupdates/NewObstacles.h>
#include <visualization_msgs/Marker.h>

//std::vector<std::string> nodeuuid;
//std::vector<float> nodex, nodey;
maptogridmap::Nodes gmnode;

class VisualizationPublisherGML
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher gridmapls_pub, graphvs_pub, stc_pub, globalpoints_pub, graph_pub;
 	ros::Subscriber gml_sub, nodes_sub, edges_sub, newobs_sub, graph_sub;


    visualization_msgs::Marker gridmapls, graphvs, stc, glp, footprint;

  VisualizationPublisherGML(ros::NodeHandle n) :
      nh_(n),  target_frame_("map") 
  {

	 	gml_sub = nh_.subscribe("map/topology",1,&VisualizationPublisherGML::gridmapCallback, this);
	 	graph_sub = nh_.subscribe("map/graph",1,&VisualizationPublisherGML::graphCallback, this);
	 	nodes_sub = nh_.subscribe("map/nodes",1,&VisualizationPublisherGML::nodesCallback, this);
	 	edges_sub = nh_.subscribe("map/edges",1,&VisualizationPublisherGML::edgesCallback, this);
	 	newobs_sub = nh_.subscribe("/robot_0/newObstacles",1,&VisualizationPublisherGML::newObstaclesCallback, this);
		gridmapls_pub=nh_.advertise<visualization_msgs::Marker>("/gridmap_markerListener",10);

    gridmapls.header.frame_id = target_frame_;
    gridmapls.header.stamp = ros::Time::now();
    gridmapls.ns =  "maplistener";
    gridmapls.action = visualization_msgs::Marker::ADD;
    gridmapls.pose.orientation.w  = 1.0;
    gridmapls.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    gridmapls.scale.x = 0.25; 
    gridmapls.scale.y = 0.25; 
    gridmapls.color.r = 0.8;
    gridmapls.color.g = 0.;
    gridmapls.color.b = 0.8;
    gridmapls.color.a = 1.0;
      
	globalpoints_pub=nh_.advertise<visualization_msgs::Marker>("/newobstacles_markerListener",10);

    glp.header.frame_id = target_frame_;
    glp.header.stamp = ros::Time::now();
    glp.ns =  "maplistener";
    glp.action = visualization_msgs::Marker::ADD;
    glp.pose.orientation.w  = 1.0;
    glp.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    glp.scale.x = 0.05; 
    glp.scale.y = 0.05; 
    glp.color.r = 0.;
    glp.color.g = 0.;
    glp.color.b = 0.8;
    glp.color.a = 1.0;

	graphvs_pub=nh_.advertise<visualization_msgs::Marker>("/nodes_markerListener",10);

    graphvs.header.frame_id = target_frame_;
    graphvs.header.stamp = ros::Time::now();
    graphvs.ns =  "maplistener";
    graphvs.action = visualization_msgs::Marker::ADD;
    graphvs.pose.orientation.w  = 1.0;
    graphvs.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    graphvs.scale.x = 0.25; 
    graphvs.scale.y = 0.25; 
    graphvs.color.r = 0.;
    graphvs.color.g = 0.;
    graphvs.color.b = 0.8;
    graphvs.color.a = 1.0;
    
	graph_pub=nh_.advertise<visualization_msgs::Marker>("/footprint_markerListener",10);

    footprint.header.frame_id = target_frame_;
    footprint.header.stamp = ros::Time::now();
    footprint.ns =  "maplistener";
    footprint.action = visualization_msgs::Marker::ADD;
    footprint.pose.orientation.w  = 1.0;
    footprint.type = visualization_msgs::Marker::LINE_LIST;
    footprint.scale.x = 0.05; 
    footprint.scale.y = 0.05; 
    footprint.color.r = 0.2;
    footprint.color.g = 0.8;
    footprint.color.b = 0.8;
    footprint.color.a = 1.0;

  	stc_pub=nh_.advertise<visualization_msgs::Marker>("/edges_markerListener",10);

  	stc.header.frame_id = target_frame_;
    stc.header.stamp = ros::Time::now();
    stc.ns =  "maplistener";
    stc.action = visualization_msgs::Marker::ADD;
    stc.pose.orientation.w  = 1.0;
    stc.type = visualization_msgs::Marker::LINE_LIST;//POINTS; //LINE_STRIP;
    stc.scale.x = 0.1; 
    stc.scale.y = 0.1; 
    stc.color.r = 0.;
    stc.color.g = 0.3;
    stc.color.b = 0.5;
    stc.color.a = 1.0;

  }

  void visualizationduringmotion();
  void gridmapCallback(const maptogridmap::GridmapConstPtr& gmMsg);
  void graphCallback(const maptogridmap::GraphConstPtr& gmMsg);
  void nodesCallback(const maptogridmap::NodesConstPtr& gmMsg);
  void edgesCallback(const maptogridmap::EdgesConstPtr& gmMsg);
	void newObstaclesCallback(const mapupdates::NewObstaclesConstPtr& msg);

};



int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "maplistener");
  ros::NodeHandle nh;
  VisualizationPublisherGML visualGML(nh);


  ros::Rate rate(10.0);


  while (nh.ok()) {


    ros::spinOnce(); 

	  visualGML.visualizationduringmotion();	
    
    
	  rate.sleep();
  }
  return 0;
}


void VisualizationPublisherGML::visualizationduringmotion(){

			gridmapls_pub.publish(gridmapls);
			graphvs_pub.publish(graphvs);
			globalpoints_pub.publish(glp);
			stc_pub.publish(stc);
			graph_pub.publish(footprint);


}

void VisualizationPublisherGML::gridmapCallback(const maptogridmap::GridmapConstPtr& gmMsg)
{
	int width = gmMsg->info.width;
	int height = gmMsg->info.height;
  double resolution = gmMsg->info.resolution;
//  printf("listening message occupancy map data: res=%f, width=%d, height=%d\n", resolution, width, height);
  gridmapls.points.clear();
  geometry_msgs::Point p; 
  int sizex=width;
  int sizey=height;
	for (int i=0; i<sizex*sizey; i++){
		p.x=gmMsg->x[i];
	  p.y=gmMsg->y[i];
		if ((gmMsg->occupancy[i]>0)){
			gridmapls.points.push_back(p);
		}
	}
}

void VisualizationPublisherGML::graphCallback(const maptogridmap::GraphConstPtr& gmMsg)
{
  footprint.points.clear();
  geometry_msgs::Point p; 
	for (int i=0; i<gmMsg->vertices.size(); i++){
		for (int d=0; d<4;d++){
			p.x=gmMsg->vertices[i].footprint[d].x;
			p.y=gmMsg->vertices[i].footprint[d].y;
			footprint.points.push_back(p);
			if (d<3){
			p.x=gmMsg->vertices[i].footprint[d+1].x;
			p.y=gmMsg->vertices[i].footprint[d+1].y;
			footprint.points.push_back(p);
			}
		}
		p.x=gmMsg->vertices[i].footprint[0].x;
		p.y=gmMsg->vertices[i].footprint[0].y;
		footprint.points.push_back(p);
	}
}

void VisualizationPublisherGML::nodesCallback(const maptogridmap::NodesConstPtr& gmMsg)
{
  graphvs.points.clear();
  geometry_msgs::Point p; 
	gmnode.x.clear();
	gmnode.y.clear();
	gmnode.name.clear();
	gmnode.uuid.clear();
	for (int i=0; i<gmMsg->x.size(); i++){
		p.x=gmMsg->x[i];
		p.y=gmMsg->y[i];
		graphvs.points.push_back(p);
		gmnode.x.push_back(gmMsg->x[i]);
		gmnode.y.push_back(gmMsg->y[i]);
		gmnode.name.push_back(gmMsg->name[i]);
		gmnode.uuid.push_back(gmMsg->uuid[i]);
	}
}

void VisualizationPublisherGML::edgesCallback(const maptogridmap::EdgesConstPtr& gmMsg)
{
  stc.points.clear();
  geometry_msgs::Point p; 
  int foundsrcdest;
	for (int i=0; i<gmMsg->name.size(); i++){
		foundsrcdest=0;
		for (int j=0; j<gmnode.name.size(); j++){
			if (gmnode.uuid[j]==gmMsg->uuid_src[i]){
				p.x=gmnode.x[j];
				p.y=gmnode.y[j];
				stc.points.push_back(p);
				foundsrcdest++;
				if (foundsrcdest==2)
					break;
			}
			if (gmnode.uuid[j]==gmMsg->uuid_dest[i]){
				p.x=gmnode.x[j];
				p.y=gmnode.y[j];
				stc.points.push_back(p);
				foundsrcdest++;
				if (foundsrcdest==2)
					break;
			}
		}
		
	}
}

void VisualizationPublisherGML::newObstaclesCallback(const mapupdates::NewObstaclesConstPtr& msg)
{
  glp.points.clear();
  geometry_msgs::Point p; 
	for (int i =0; i<msg->x.size(); i++){
		p.x=msg->x[i];
		p.y=msg->y[i];
		glp.points.push_back(p);
	}
}


