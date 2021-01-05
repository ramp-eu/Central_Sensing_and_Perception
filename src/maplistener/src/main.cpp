#include <sys/time.h>
#include <iostream>
#include <string>
#include "ros/ros.h"

#include <maptogridmap/Graph.h>
#include <mapupdates/NewObstacles.h>
#include <visualization_msgs/Marker.h>


class VisualizationPublisherGML
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher stc_pub, globalpoints_pub, graph_pub;
 	ros::Subscriber gml_sub, newobs_sub, graph_sub;


    visualization_msgs::Marker graphvs, stc, glp, graphvertex;

  VisualizationPublisherGML(ros::NodeHandle n) :
      nh_(n),  target_frame_("map") 
  {

	 	graph_sub = nh_.subscribe("map/graph",1,&VisualizationPublisherGML::graphCallback, this);
	 	newobs_sub = nh_.subscribe("/robot_0/newObstacles",1,&VisualizationPublisherGML::newObstaclesCallback, this);

      
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

    
	graph_pub=nh_.advertise<visualization_msgs::Marker>("/graph_vertices_markerListener",10);

    graphvertex.header.frame_id = target_frame_;
    graphvertex.header.stamp = ros::Time::now();
    graphvertex.ns =  "maplistener";
    graphvertex.action = visualization_msgs::Marker::ADD;
    graphvertex.pose.orientation.w  = 1.0;
    graphvertex.type = visualization_msgs::Marker::POINTS;
    graphvertex.scale.x = 0.25; 
    graphvertex.scale.y = 0.25; 
    graphvertex.color.r = 0.2;
    graphvertex.color.g = 0.8;
    graphvertex.color.b = 0.8;
    graphvertex.color.a = 1.0;

  	stc_pub=nh_.advertise<visualization_msgs::Marker>("/graph_edges_markerListener",10);

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
  void graphCallback(const maptogridmap::GraphConstPtr& gmMsg);
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

			globalpoints_pub.publish(glp);
			stc_pub.publish(stc);
			graph_pub.publish(graphvertex);


}


void VisualizationPublisherGML::graphCallback(const maptogridmap::GraphConstPtr& gmMsg)
{
  graphvertex.points.clear();
  stc.points.clear();
  geometry_msgs::Point p; 
	for (int i=0; i<gmMsg->vertices.size(); i++){
		p.x=gmMsg->vertices[i].x;
		p.y=gmMsg->vertices[i].y;
		graphvertex.points.push_back(p);
	}
	int foundsrcdest;
	for (int i=0; i<gmMsg->edges.size(); i++){
		foundsrcdest=0;
		for (int j=0; j<gmMsg->vertices.size(); j++){
			if (gmMsg->vertices[j].uuid==gmMsg->edges[i].uuid_src){
				p.x=gmMsg->vertices[j].x;
				p.y=gmMsg->vertices[j].y;
				stc.points.push_back(p);
				foundsrcdest++;
				if (foundsrcdest==2)
					break;
			}
			if (gmMsg->vertices[j].uuid==gmMsg->edges[i].uuid_dest){
				p.x=gmMsg->vertices[j].x;
				p.y=gmMsg->vertices[j].y;
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


