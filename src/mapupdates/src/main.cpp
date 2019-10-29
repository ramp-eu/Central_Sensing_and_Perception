#include <mapup.h>
#include <sstream>

static boost::uuids::random_generator mUUIDGen;
GridMapCell *GMU;
std::vector<float> pointx, pointy, locx, locy;
int cycle_number=0;
std::string laser_tf_frame="/base_laser_link";
bool laser_inverted=false;

using namespace std;

class VisualizationPublisherGM
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher gridmapvs_pub, globalpoints_pub;



    visualization_msgs::Marker gridmapvs, graphvs, stc, glp;

  VisualizationPublisherGM(ros::NodeHandle n) :
      nh_(n),  target_frame_("map") 
  {

	gridmapvs_pub=nh_.advertise<visualization_msgs::Marker>("/finegridmap_marker",10);

    gridmapvs.header.frame_id = target_frame_;
    gridmapvs.header.stamp = ros::Time::now();
    gridmapvs.ns =  "mapupdates";
    gridmapvs.action = visualization_msgs::Marker::ADD;
    gridmapvs.pose.orientation.w  = 1.0;
    gridmapvs.type = visualization_msgs::Marker::POINTS; //LINE_STRIP;
    gridmapvs.scale.x = 0.05; 
    gridmapvs.scale.y = 0.05; 
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




  }

  void visualizationduringmotion();


};


//void globalPointsCallback(const mapupdates::NewObstaclesConstPtr& msg)
//{
////	std::cout << msg->x.size()<<std::endl;
//	pointx.clear();
//	pointy.clear();
//	for (int i =0; i<msg->x.size(); i++){
//		pointx.push_back(msg->x[i]);
//		pointy.push_back(msg->y[i]);
//	}
//}
double loc2glob_x(double Rx, double Rth, double X, double Y){
	double globX;
	globX=Rx+cos(Rth)*X-sin(Rth)*Y;
	return globX;
}

double loc2glob_y(double Ry, double Rth, double X, double Y){
	double globY;
	globY=Ry+sin(Rth)*X+cos(Rth)*Y;
	return globY;
}

void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
	locx.clear();
	locy.clear();
	laser_tf_frame=msg->header.frame_id;
	double ranges;
	for(int i_LS=0;i_LS<msg->ranges.size();i_LS++){
		if (laser_inverted){
	        ranges=msg->ranges[msg->ranges.size()-1-i_LS];
        }else{
	        ranges=msg->ranges[i_LS];
		}
		if (ranges<msg->range_max){
			  locx.push_back(ranges*cos(msg->angle_min+i_LS * msg->angle_increment));
			  locy.push_back(ranges*sin(msg->angle_min+i_LS * msg->angle_increment));
		}
	}

}


int main(int argc, char** argv)
{
  
  int robotId=0;
	if(argc < 2){
	  	ROS_ERROR("You did not specify the Robot ID, default ID 0 is used!");
//		  return -1;	
	}else{
		robotId = atoi(argv[1]); 
	}
	std::stringstream ss;
	std::string scan_topic_;
  ss << robotId;			
	ROS_INFO("Hello, I am robot %d",robotId);
	ros::init(argc, argv, "mapupdates");
  ros::NodeHandle nh;
  
  ros::Publisher newObstacles_pub = nh.advertise<mapupdates::NewObstacles>("/robot_"+ss.str()+"/newObstacles",1); 
//  ros::Subscriber read_global_points = nh.subscribe("/global_points", 1, globalPointsCallback);
  scan_topic_="/base_scan";
  nh.getParam("/mapup/scan_topic", scan_topic_);
  nh.getParam("/mapup/laser_inverted", laser_inverted);
  tf::TransformListener tf_listener;
  tf::StampedTransform transform;

	ros::Subscriber read_laser = nh.subscribe(scan_topic_, 1, laserCallback);
  VisualizationPublisherGM visualGM(nh);
  nav_msgs::GetMap map;
  ros::service::waitForService("static_map", 5000);
  ros::service::call("static_map",map);
  double cellsize=0.1;
  nh.getParam("/mapup/cell_size", cellsize);
 
  double resolution=map.response.map.info.resolution;
  geometry_msgs::Pose mappose=map.response.map.info.origin;
  std::cout << mappose <<std::endl;
  double xorigin=mappose.position.x;
  double yorigin=mappose.position.y; 
  int width=map.response.map.info.width;
  int height=map.response.map.info.height;
  int sizex = int (floor (width*resolution / cellsize))+1;
  int sizey = int (floor (height*resolution / cellsize))+1;
  printf("converting the map data to gridmap: cell size %f, res=%f, width=%d, height=%d, xorigin=%f, yorigin=%f, size gridmap (%d,%d)\n",cellsize, resolution, width, height, xorigin, yorigin, sizex, sizey);
  GMU = new GridMapCell(sizex, sizey, cellsize);
	int ii,jj;
	GMcell **gmap=GMU->GetMap();
//	sleep(3);
	for (int j=0; j < width; j++)
	{
			for (int i=0; i < height; i++){
				ii=(int)floor(j*resolution/cellsize);
				jj=(int)floor(i*resolution/cellsize);
				if (gmap[ii][jj].visited==-1){
					gmap[ii][jj].visited=0;
					gmap[ii][jj].x=ii*cellsize+cellsize/2.+xorigin;
					gmap[ii][jj].y=jj*cellsize+cellsize/2.+yorigin;
					boost::uuids::uuid lUUID=mUUIDGen();
					gmap[ii][jj].uuid=boost::uuids::to_string(lUUID);
					gmap[ii][jj].name="vertex_"+std::to_string(ii*sizey+jj);
				}
				if ((gmap[ii][jj].occupancy==0)&&(map.response.map.data[i*width+j]!=0)){
					gmap[ii][jj].occupancy=1;
					gmap[ii][jj].staticcell=true;
				}
			}
	}
	
	mapupdates::NewObstacles gmobstacles; 
  gmobstacles.header.stamp = ros::Time::now();
	newObstacles_pub.publish(gmobstacles); 

  tf_listener.waitForTransform("map", laser_tf_frame, ros::Time::now(), ros::Duration(3.0));//robot_0/
  double yaw, pitch, roll, tfx, tfy;

  ros::Rate rate(10.0);

  while (nh.ok()) {
    ros::spinOnce(); 
    cycle_number++;
//    std::cout << "main " << cycle_number <<std::endl;
    try {
        tf_listener.waitForTransform("/map", laser_tf_frame, ros::Time(0), ros::Duration(0.1));
        tf_listener.lookupTransform("/map", laser_tf_frame, ros::Time(0), transform);

				transform.getBasis().getRPY(roll, pitch, yaw);
				tfx=transform.getOrigin().x();
				tfy=transform.getOrigin().y();
				pointx.clear();
				pointy.clear();
				for (int i=0; i<locx.size(); i++){
					pointx.push_back(loc2glob_x(tfx, yaw,locx[i],locy[i]));
					pointy.push_back(loc2glob_y(tfy, yaw,locx[i],locy[i]));
				}
		    gmobstacles.x.clear();
		    gmobstacles.y.clear();
				gmobstacles.header.stamp = ros::Time::now();
				gmobstacles.header.frame_id = "map";
				for (int i=0; i<sizex; i++){
						for (int j=0; j<sizey; j++){
								if ((gmap[i][j].staticcell==false) && (gmap[i][j].occupancy>0)){
									gmap[i][j].occupancy=gmap[i][j].occupancy-1;
								}
						}
				}
				for(int i = 0; i<pointx.size(); i++){
					ii=(int)floor((pointx[i]-xorigin)/cellsize);
					jj=(int)floor((pointy[i]-yorigin)/cellsize);
					if (ii>0 && jj>0 && ii<sizex && jj<sizey){
			//			if ((gmap[ii][jj].occupancy==0) && (gmap[ii][jj].visited!=cycle_number))
						if ((gmap[ii][jj].occupancy<=50))
						{
			//				std::cout <<pointx[i]<<" "<<pointy[i]<<std::endl;
							gmobstacles.x.push_back(gmap[ii][jj].x);
							gmobstacles.y.push_back(gmap[ii][jj].y);
			//				gmap[ii][jj].visited = cycle_number;
							gmap[ii][jj].occupancy = 100;
						}
					}	
				}
    }
    catch (tf::TransformException ex) {
        ROS_INFO("Local Map Updates: %s", ex.what());
    }
		newObstacles_pub.publish(gmobstacles); 


		visualGM.visualizationduringmotion();	
    
    
	  rate.sleep();
  }
  return 0;
}




void VisualizationPublisherGM::visualizationduringmotion(){


      gridmapvs.points.clear();
      glp.points.clear();
      geometry_msgs::Point p; 
      GMcell **gmap=GMU->GetMap();
      int sizex=GMU->GetMapSizeX();
      int sizey=GMU->GetMapSizeY();
			int cellsize=GMU->GetSizeCell();
			for (int i=0; i<sizex; i++){
				for (int j=0; j<sizey; j++){
					if ((gmap[i][j].occupancy>0)||(gmap[i][j].visited==cycle_number)){
						p.x=gmap[i][j].x;
            			p.y=gmap[i][j].y;
						gridmapvs.points.push_back(p);
					}
				}
			}
			gridmapvs_pub.publish(gridmapvs);
			//points from global lasers
			for(int i = 0; i<pointx.size(); i++){
				p.x = pointx[i];
				p.y = pointy[i];
				glp.points.push_back(p);
			}
			globalpoints_pub.publish(glp);


}
