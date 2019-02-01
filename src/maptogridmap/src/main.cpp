#include <gmap.h>

static boost::uuids::random_generator mUUIDGen;
GridMapCell *GMC;
maptogridmap::GetMap::Response map_resp_;
int cycle_number;
mapupdates::NewObstacles obstacles;

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
    gridmapvs.ns =  "maptogridmap";
    gridmapvs.action = visualization_msgs::Marker::ADD;
    gridmapvs.pose.orientation.w  = 1.0;
    gridmapvs.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    gridmapvs.scale.x = 0.25; 
    gridmapvs.scale.y = 0.25; 
    gridmapvs.color.r = 0.8;
    gridmapvs.color.g = 0.;
    gridmapvs.color.b = 0.;
    gridmapvs.color.a = 1.0;
      
	globalpoints_pub=nh_.advertise<visualization_msgs::Marker>("/newobstacles_marker",10);

    glp.header.frame_id = target_frame_;
    glp.header.stamp = ros::Time::now();
    glp.ns =  "maptogridmap";
    glp.action = visualization_msgs::Marker::ADD;
    glp.pose.orientation.w  = 1.0;
    glp.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    glp.scale.x = 0.05; 
    glp.scale.y = 0.05; 
    glp.color.r = 0.;
    glp.color.g = 0.;
    glp.color.b = 0.8;
    glp.color.a = 1.0;

	graphvs_pub=nh_.advertise<visualization_msgs::Marker>("/nodes_marker",10);

    graphvs.header.frame_id = target_frame_;
    graphvs.header.stamp = ros::Time::now();
    graphvs.ns =  "maptogridmap";
    graphvs.action = visualization_msgs::Marker::ADD;
    graphvs.pose.orientation.w  = 1.0;
    graphvs.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    graphvs.scale.x = 0.25; 
    graphvs.scale.y = 0.25; 
    graphvs.color.r = 0.;
    graphvs.color.g = 0.;
    graphvs.color.b = 0.8;
    graphvs.color.a = 1.0;
    
  	stc_pub=nh_.advertise<visualization_msgs::Marker>("/edges_marker",10);

  	stc.header.frame_id = target_frame_;
    stc.header.stamp = ros::Time::now();
    stc.ns =  "maptogridmap";
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

bool mapCallback(maptogridmap::GetMap::Request  &req, maptogridmap::GetMap::Response &res )
     {
       // request is empty; we ignore it
 
       // = operator is overloaded to make deep copy (tricky!)
       res = map_resp_;
       ROS_INFO("Sending map");
 
       return true;
     }
void newObstaclesCallback(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	obstacles.x.clear();
	obstacles.y.clear();
	for (int i =0; i<msg->x.size(); i++){
		obstacles.x.push_back(msg->x[i]);
		obstacles.y.push_back(msg->y[i]);
	}
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "maptogridmap");
  ros::NodeHandle nh;
  
  ros::Publisher gmap_pub = nh.advertise<maptogridmap::Gridmap>("map/topology",1);
  ros::Publisher nodes_pub = nh.advertise<maptogridmap::Nodes>("map/nodes",1);
  ros::Publisher edges_pub = nh.advertise<maptogridmap::Edges>("map/edges",1);
  ros::Subscriber gmu_sub = nh.subscribe("/robot_0/newObstacles",1,newObstaclesCallback);

  ros::ServiceServer service = nh.advertiseService("grid_map", mapCallback);
  VisualizationPublisherGM visualGM(nh);
  nav_msgs::GetMap map;
  ros::service::call("static_map",map);
	double cellsize=0.1;
  nh.getParam("/map2gm/cell_size", cellsize);
 
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
	
//	maptogridmap::GridmapCell gmcell;
	maptogridmap::Gridmap gm;
	maptogridmap::Nodes gmnode;
	maptogridmap::Edges gmedge;
	gm.info.width=sizex;
	gm.info.height=sizey;
	gm.info.resolution=cellsize;
	gm.info.map_load_time = ros::Time::now();
	gm.header.frame_id = "map";
  gm.header.stamp = ros::Time::now();
  gmnode.header=gm.header;
  gmnode.info=gm.info;
  gmedge.header=gm.header;
	map_resp_.map.header=gm.header;
  map_resp_.map.info=gm.info;
			for (int i=0; i<sizex; i++){
				for (int j=0; j<sizey; j++){
					if (1){
						gm.x.push_back(gmap[i][j].x);
            gm.y.push_back(gmap[i][j].y);
            gm.occupancy.push_back(gmap[i][j].occupancy);
            if (gmap[i][j].occupancy==0){
            	gmnode.x.push_back(gmap[i][j].x);
            	gmnode.y.push_back(gmap[i][j].y);
            	gmnode.name.push_back(gmap[i][j].name);
//            	boost::uuids::uuid lUUID=mUUIDGen();
            	gmnode.uuid.push_back(gmap[i][j].uuid);
            }
//           	gmcell.x=gmap[i][j].x;
//           	gmcell.y=gmap[i][j].y;
//           	gmcell.occupancy=gmap[i][j].occupancy;
//						gm.gmc.push_back(gmcell);
					}
				}
			}
			for(int i=0; i<GMC->edges.size(); i++){
				gmedge.uuid_src.push_back(gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid);
				gmedge.uuid_dest.push_back(gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid);
				boost::uuids::uuid lUUID=mUUIDGen();
				gmedge.uuid.push_back(boost::uuids::to_string(lUUID));
				gmedge.name.push_back("edge_"+std::to_string(GMC->edges[i].xs*sizey+GMC->edges[i].ys)+"_"+std::to_string(GMC->edges[i].xg*sizey+GMC->edges[i].yg));
			}
	gmap_pub.publish(gm);
	nodes_pub.publish(gmnode);
	edges_pub.publish(gmedge);
	//map_resp_.map.gmc=gm.gmc;

  ros::Rate rate(10.0);
  cycle_number=0;
  int update_nodes_edges;

  while (nh.ok()) {
    cycle_number++;
    update_nodes_edges = 0;
		for(int i = 0; i<obstacles.x.size(); i++){
			ii=(int)floor(obstacles.x[i]/cellsize);
			jj=(int)floor(obstacles.y[i]/cellsize);
			if (ii>0 && jj>0 && ii<sizex && jj<sizey){
	//			if ((gmap[ii][jj].occupancy==0) && (gmap[ii][jj].visited!=cycle_number))
				if ((gmap[ii][jj].occupancy==0))
				{
	//				std::cout <<pointx[i]<<" "<<pointy[i]<<std::endl;
					gmap[ii][jj].occupancy = 1;
					gm.occupancy[ii*sizey+jj] = 1;
					update_nodes_edges = 1;
				}
			}	
		}

		if (update_nodes_edges){
			gmnode.x.clear();
			gmnode.y.clear();	
			gmnode.uuid.clear();
			gmnode.name.clear();
			for (int i=0; i<sizex; i++){
						for (int j=0; j<sizey; j++){
								gmap[i][j].visited=0;
				        if (gmap[i][j].occupancy==0){
				        	gmnode.x.push_back(gmap[i][j].x);
				        	gmnode.y.push_back(gmap[i][j].y);
				        	gmnode.name.push_back(gmap[i][j].name);
				        	gmnode.uuid.push_back(gmap[i][j].uuid);
				        }
						}
			}
			GMC->createEdges();
			gmedge.uuid_src.clear();
			gmedge.uuid_dest.clear();
			gmedge.uuid.clear();
			gmedge.name.clear();
			for(int i=0; i<GMC->edges.size(); i++){
						gmedge.uuid_src.push_back(gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid);
						gmedge.uuid_dest.push_back(gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid);
						boost::uuids::uuid lUUID=mUUIDGen();
						gmedge.uuid.push_back(boost::uuids::to_string(lUUID));
						gmedge.name.push_back("edge_"+std::to_string(GMC->edges[i].xs*sizey+GMC->edges[i].ys)+"_"+std::to_string(GMC->edges[i].xg*sizey+GMC->edges[i].yg));
			}	
		}

		gmnode.header.stamp = ros::Time::now();
		gm.header.stamp = ros::Time::now();
		gmedge.header.stamp = ros::Time::now();
		gmap_pub.publish(gm);
		nodes_pub.publish(gmnode);
		edges_pub.publish(gmedge);

    ros::spinOnce(); 

		visualGM.visualizationduringmotion();	
    
    
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
			//points from newObstacles
			for(int i = 0; i<obstacles.x.size(); i++){
				p.x = obstacles.x[i];
				p.y = obstacles.y[i];
				glp.points.push_back(p);
			}
			globalpoints_pub.publish(glp);

}
