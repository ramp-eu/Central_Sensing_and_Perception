#include <gmap.h>

static boost::uuids::random_generator mUUIDGen;
GridMapCell *GMC;
std::vector<float> pointx, pointy;
int cycle_number;

class VisualizationPublisherGM
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher gridmapvs_pub, graphvs_pub, stc_pub, globalpoints_pub;



    visualization_msgs::Marker gridmapvs, graphvs, stc, glp;

  VisualizationPublisherGM(ros::NodeHandle n) :
      nh_(n),  target_frame_("map") 
  {

	gridmapvs_pub=nh_.advertise<visualization_msgs::Marker>("/gridmap_marker",10);

    gridmapvs.header.frame_id = target_frame_;
    gridmapvs.header.stamp = ros::Time::now();
    gridmapvs.ns =  "mapupdates";
    gridmapvs.action = visualization_msgs::Marker::ADD;
    gridmapvs.pose.orientation.w  = 1.0;
    gridmapvs.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    gridmapvs.scale.x = 0.25; 
    gridmapvs.scale.y = 0.25; 
    gridmapvs.color.r = 0.8;
    gridmapvs.color.g = 0.;
    gridmapvs.color.b = 0.;
    gridmapvs.color.a = 1.0;
      
	globalpoints_pub=nh_.advertise<visualization_msgs::Marker>("/globalpoints_marker",10);

    glp.header.frame_id = target_frame_;
    glp.header.stamp = ros::Time::now();
    glp.ns =  "mapupdates";
    glp.action = visualization_msgs::Marker::ADD;
    glp.pose.orientation.w  = 1.0;
    glp.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    glp.scale.x = 0.05; 
    glp.scale.y = 0.05; 
    glp.color.r = 0.;
    glp.color.g = 0.;
    glp.color.b = 0.8;
    glp.color.a = 1.0;

	graphvs_pub=nh_.advertise<visualization_msgs::Marker>("/graph_marker",10);

    graphvs.header.frame_id = target_frame_;
    graphvs.header.stamp = ros::Time::now();
    graphvs.ns =  "mapupdates";
    graphvs.action = visualization_msgs::Marker::ADD;
    graphvs.pose.orientation.w  = 1.0;
    graphvs.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    graphvs.scale.x = 0.25; 
    graphvs.scale.y = 0.25; 
    graphvs.color.r = 0.;
    graphvs.color.g = 0.;
    graphvs.color.b = 0.8;
    graphvs.color.a = 1.0;
    
  	stc_pub=nh_.advertise<visualization_msgs::Marker>("/spanningtree_marker",10);

  	stc.header.frame_id = target_frame_;
    stc.header.stamp = ros::Time::now();
    stc.ns =  "mapupdates";
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


};


void globalPointsCallback(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	pointx.clear();
	pointy.clear();
	for (int i =0; i<msg->x.size(); i++){
		pointx.push_back(msg->x[i]);
		pointy.push_back(msg->y[i]);
	}
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "mapupdates");
  ros::NodeHandle nh;
  
  ros::Publisher newObstacles_pub = nh.advertise<mapupdates::NewObstacles>("map/newObstacles",1); 
  ros::Subscriber read_global_points = nh.subscribe("/global_points", 1, globalPointsCallback);
  VisualizationPublisherGM visualGM(nh);
  nav_msgs::GetMap map;
  ros::service::call("static_map",map);
	double cellsize=0.1;
  nh.getParam("/mapup/cell_size", cellsize);
 
  double resolution=map.response.map.info.resolution;
  int width=map.response.map.info.width;
  int height=map.response.map.info.height;
  int sizex = int (floor (width*resolution / cellsize))+1;
  int sizey = int (floor (height*resolution / cellsize))+1;
  printf("converting the map data to gridmap: cell size %f, res=%f, width=%d, height=%d, size gridmap (%d,%d)\n",cellsize, resolution, width, height, sizex, sizey);
  GMC = new GridMapCell(sizex, sizey, cellsize);
	int ii,jj;
	GMcell **gmap=GMC->GetMap();
	for (int j=0; j < width; j++)
	{
			for (int i=0; i < height; i++){
				ii=(int)floor(j*resolution/cellsize);
				jj=(int)floor(i*resolution/cellsize);
				if (gmap[ii][jj].visited==-1){
					gmap[ii][jj].visited=0;
					gmap[ii][jj].x=ii*cellsize+cellsize/2.;
					gmap[ii][jj].y=jj*cellsize+cellsize/2.;
					boost::uuids::uuid lUUID=mUUIDGen();
					gmap[ii][jj].uuid=boost::uuids::to_string(lUUID);
					gmap[ii][jj].name="vertex_"+std::to_string(ii*sizey+jj);
				}
				if ((gmap[ii][jj].occupancy==0)&&(map.response.map.data[i*width+j]>0)){
					gmap[ii][jj].occupancy=1;
					gmap[ii][jj].staticcell=true;
				}
			}
	}
//	GMC->spanningTree(13,13);
	GMC->createEdges();
	
	mapupdates::NewObstacles gmobstacles; 
  gmobstacles.header.stamp = ros::Time::now();
	newObstacles_pub.publish(gmobstacles); 

  ros::Rate rate(10.0);
  cycle_number=0;

  while (nh.ok()) {
    cycle_number++;
//    gmobstacles.x.clear();
//    gmobstacles.y.clear();
    gmobstacles.header.stamp = ros::Time::now();
	for(int i = 0; i<pointx.size(); i++){
		ii=(int)floor(pointx[i]/cellsize);
		jj=(int)floor(pointy[i]/cellsize);
		if (ii>0 && jj>0 && ii<sizex && jj<sizey){
//			if ((gmap[ii][jj].occupancy==0) && (gmap[ii][jj].visited!=cycle_number))
			if ((gmap[ii][jj].occupancy==0))
			{
				std::cout <<pointx[i]<<" "<<pointy[i]<<std::endl;
				gmobstacles.x.push_back(gmap[ii][jj].x);
				gmobstacles.y.push_back(gmap[ii][jj].y);
//				gmap[ii][jj].visited = cycle_number;
				gmap[ii][jj].occupancy = 1;
			}
		}	
	}
	newObstacles_pub.publish(gmobstacles); 

    ros::spinOnce(); 

//	visualGM.visualizationduringmotion();	
    
    
	  rate.sleep();
  }
  return 0;
}




void VisualizationPublisherGM::visualizationduringmotion(){


      gridmapvs.points.clear();
      graphvs.points.clear();
      stc.points.clear();
      glp.points.clear();
      geometry_msgs::Point p; 
      GMcell **gmap=GMC->GetMap();
      int sizex=GMC->GetMapSizeX();
      int sizey=GMC->GetMapSizeY();
			int cellsize=GMC->GetSizeCell();
			for (int i=0; i<sizex; i++){
				for (int j=0; j<sizey; j++){
					if ((gmap[i][j].occupancy>0)||(gmap[i][j].visited==cycle_number)){
						p.x=gmap[i][j].x;
            			p.y=gmap[i][j].y;
						gridmapvs.points.push_back(p);
					}else{
						if (gmap[i][j].visited!=cycle_number){
							p.x=gmap[i][j].x;
		          			p.y=gmap[i][j].y;
							graphvs.points.push_back(p);
						}					
					}
				}
			}
			gridmapvs_pub.publish(gridmapvs);
			graphvs_pub.publish(graphvs);

			if(GMC->spanningpath.size()>0){
				int temp_length=GMC->spanningpath.size();
				for(int pathLength=0; pathLength<temp_length;pathLength++){
			        	p.x = GMC->spanningpath[pathLength].x*cellsize+0.5*cellsize;
	    					p.y = GMC->spanningpath[pathLength].y*cellsize+0.5*cellsize;

    					stc.points.push_back(p);
				}
			//publish path			
			stc_pub.publish(stc);
		
			}
//edges
			if(GMC->edges.size()>0){
				int temp_length=GMC->edges.size();
				for(int pathLength=0; pathLength<temp_length;pathLength++){
			        	p.x = GMC->edges[pathLength].xs*cellsize+0.5*cellsize;
	    				p.y = GMC->edges[pathLength].ys*cellsize+0.5*cellsize;
    					stc.points.push_back(p);
			        	p.x = GMC->edges[pathLength].xg*cellsize+0.5*cellsize;
	    				p.y = GMC->edges[pathLength].yg*cellsize+0.5*cellsize;
    					stc.points.push_back(p);
    					
				}
			//publish path			
			stc_pub.publish(stc);
		
			}
			//global points from laser
			for(int i = 0; i<pointx.size(); i++){
				p.x = pointx[i];
				p.y = pointy[i];
				glp.points.push_back(p);
			}
			globalpoints_pub.publish(glp);

}
