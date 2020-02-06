#include <gmap.h>

//static boost::uuids::random_generator mUUIDGen;
//static const std::wstring DNS_NAMESPACE_UUID = L"6ba7b810-9dad-11d1-80b4-00c04fd430c8";
using namespace boost::uuids;
uuid dns_namespace_uuid;
//name_generator lUUIDNameGen(dns_namespace_uuid);
name_generator lUUIDNameGen(string_generator()("6ba7b810-9dad-11d1-80b4-00c04fd430c8"));
//boost::uuids::name_generator lUUIDNameGen = boost::uuids::name_generator(mUUIDStringGen(DNS_NAMESPACE_UUID));

GridMapCell *GMC;
maptogridmap::GetMap::Response map_resp_;
int cycle_number;
double cellsize=2.;
mapupdates::NewObstacles obstacles;
mapupdates::NewObstacles obstacles1; //for robot_1
mapupdates::NewObstacles obstacles2; //for robot_2
//maptogridmap::Nodes annotations;
maptogridmap::Annotations annotations;

class VisualizationPublisherGM
{
protected:
  // Our NodeHandle
  ros::NodeHandle nh_;
  std::string target_frame_;

public:
  ros::Publisher gridmapvs_pub, graphvs_pub, stc_pub, globalpoints_pub, annt_pub;



    visualization_msgs::Marker gridmapvs, graphvs, stc, glp, annt;

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

  	annt_pub=nh_.advertise<visualization_msgs::Marker>("/annotation_marker",1);

  	annt.header.frame_id = target_frame_;
    annt.header.stamp = ros::Time::now();
    annt.ns =  "maptogridmap";
    annt.action = visualization_msgs::Marker::ADD;
    annt.pose.orientation.w  = 1.0;
    annt.type = visualization_msgs::Marker::ARROW;//POINTS; //LINE_STRIP;
    annt.scale.x = 0.5; 
    annt.scale.y = 0.3; 
    annt.scale.z = 0.3;
//		annt.points.resize(2);
//		annt.points[0].x = 0.;
//		annt.points[0].y = 0.;
//		annt.points[0].z = 0.;
//		annt.points[1].x = 0.;
//		annt.points[1].y = 0.;
//		annt.points[1].z = 0.5;
    annt.color.r = 1.;
    annt.color.g = 1.;
    annt.color.b = 0.;
    annt.color.a = 1.0;
    annt.lifetime = ros::Duration(0.);

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

void newObstaclesCallback1(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	obstacles1.x.clear();
	obstacles1.y.clear();
	for (int i =0; i<msg->x.size(); i++){
		obstacles1.x.push_back(msg->x[i]);
		obstacles1.y.push_back(msg->y[i]);
	}
}

void newObstaclesCallback2(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	obstacles2.x.clear();
	obstacles2.y.clear();
	for (int i =0; i<msg->x.size(); i++){
		obstacles2.x.push_back(msg->x[i]);
		obstacles2.y.push_back(msg->y[i]);
	}
}

int readAnnotations(std::string annotation_file)
{
	FILE	*F;
	char rdLine[36]="";
	char *line;
	char *word;
	maptogridmap::Annotation annt;
	char * cstr = new char [annotation_file.length()+1];
  std::strcpy (cstr, annotation_file.c_str());
  char * p = strsep (&cstr,"\n");
  while (p!=0)
  {
//    std::cout << p << '\n';
    line = &p[0];
//    std::cout << line << '\n';
    p = strsep(&cstr,"\n");

		if (line[0] == '#' || line[0] == '\n')
		{
//			std::cout << "comment or empty row "<<std::endl;
			if (p==NULL)
				std::cout << "why?" <<'\n';
			continue;
		}
		if (line[0] == '['){
//			std::cout << "annotation" <<std::endl;
			word = strtok(line,"]");
			if (word != NULL){
//				std::cout << word <<std::endl;
			}
			word = &word[1];
			if (word != NULL){
//				std::cout << word <<std::endl;
				annt.name=word;
//				annotations.name.push_back(word);
				uuid lUUID = lUUIDNameGen(word);
				annt.uuid=to_string(lUUID);
//				annotations.uuid.push_back(to_string(lUUID));
				continue;
			}
		}
		word = strtok (line,"=");
		if (word == NULL){
//			std::cout << "no input" <<std::endl;
			continue;
		}
		if (word != NULL){
//			std::cout << word <<std::endl;
			if (strcmp(word,"point_x ")==0){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
//					annotations.x.push_back(atof(word));
					annt.x=(atof(word));
				}
			}
			if (strcmp(word,"point_y ")==0){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
//					annotations.y.push_back(atof(word));
					annt.y=(atof(word));
				}
			}
			if (strcmp(word,"theta ")==0){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
//					annotations.theta.push_back(atof(word));
					annt.theta=(atof(word));
				}
			}
			if (strcmp(word,"distance ")==0){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
//					annotations.distance.push_back(atof(word));
					annt.distance=(atof(word));
					float annttempx = annt.x-annt.distance*cos(annt.theta*M_PI/180.);
					float annttempy = annt.y-annt.distance*sin(annt.theta*M_PI/180.);
					for (int k=0; k<annotations.annotations.size(); k++){
						float tempx = annotations.annotations[k].x-annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
						float tempy = annotations.annotations[k].y-annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);
						if ((fabs(tempx-annttempx)<cellsize) && (fabs(tempy-annttempy)<cellsize)){
							ROS_ERROR("Annotations are too close!!! Change the coordinates of the annotation or decrease the cell size. Here are the details:");
							std::cout << "The annotation 1:" <<'\n';
							std::cout << annt <<std::endl;
							printf("has the topology vertex at (%f,%f), which is closer than cell_size (%f m) to the vertex (%f,%f) of the annotation 2:\n",annttempx,annttempy,cellsize,tempx,tempy);
							std::cout << annotations.annotations[k] <<std::endl;
							return 0;
						}
					}
					annotations.annotations.push_back(annt);
				}
			}
		}    

  }

//this stays for debugging when running the code from the devel/lib/maptogridmap	
//	if ( (F = fopen("annotations.ini","r")) == NULL ){
//		std::cout << "no file to read "<<std::endl;
//	}else{
//		while (fgets(rdLine,35,F) != NULL)
//		{
//			line=&rdLine[0];
////			std::cout << line <<std::endl;
//			if (line[0] == '#' || line[0] == '\n')
//			{
//				std::cout << "comment or empty row "<<std::endl;
//				continue;
//			}
//			if (line[0] == '['){
//				std::cout << "annotation" <<std::endl;
//				word = strtok(line,"]");
//				if (word != NULL){
//					std::cout << word <<std::endl;
//				}
//				word = &word[1];
//				if (word != NULL){
//					std::cout << word <<std::endl;
//					annotations.name.push_back(word);
//					continue;
//				}
//			}
//			word = strtok (line,"=");
//			if (word == NULL){
//				std::cout << "no input" <<std::endl;
//				continue;
//			}
////			validconv = 0;
//			if (word != NULL){
//				std::cout << word <<std::endl;
//				if (strcmp(word,"point_x ")==0){
//					word = strtok (NULL," ");
//					if (word!= NULL){
//						std::cout << word <<std::endl;
//						annotations.x.push_back(atof(word));
//					}
//				}
//				if (strcmp(word,"point_y ")==0){
//					word = strtok (NULL," ");
//					if (word!= NULL){
//						std::cout << word <<std::endl;
//						annotations.y.push_back(atof(word));
//					}
//				}
//				if (strcmp(word,"theta ")==0){
//					word = strtok (NULL," ");
//					if (word!= NULL){
//						std::cout << word <<std::endl;
//						annotations.theta.push_back(atof(word));
//					}
//				}
//			}
//		}
//		fclose(F);
//	}
	std::cout << annotations <<std::endl;
	return 1;
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "maptogridmap");
  ros::NodeHandle nh;
  
  ros::Publisher gmap_pub = nh.advertise<maptogridmap::Gridmap>("map/topology",1);
  ros::Publisher graph_pub = nh.advertise<maptogridmap::Graph>("map/graph",1);
  ros::Publisher nodes_pub = nh.advertise<maptogridmap::Nodes>("map/nodes",1);
  ros::Publisher edges_pub = nh.advertise<maptogridmap::Edges>("map/edges",1);
  ros::Publisher annotation_pub = nh.advertise<maptogridmap::Annotations>("map/annotations",1);
  ros::Subscriber gmu_sub = nh.subscribe("/robot_0/newObstacles",1,newObstaclesCallback);
  ros::Subscriber gmu_sub1 = nh.subscribe("/robot_1/newObstacles",1,newObstaclesCallback1);
  ros::Subscriber gmu_sub2 = nh.subscribe("/robot_2/newObstacles",1,newObstaclesCallback2);

  ros::ServiceServer service = nh.advertiseService("grid_map", mapCallback);
  VisualizationPublisherGM visualGM(nh);
  nav_msgs::GetMap map;
  ros::service::waitForService("static_map", 5000);
  ros::service::call("static_map",map);
  nh.getParam("/map2gm/cell_size", cellsize);
  std::string annotation_file;
  nh.getParam("/map2gm/annotation_file", annotation_file);
//	std::cout << annotation_file <<std::endl; 
  
  double resolution=map.response.map.info.resolution;
  geometry_msgs::Pose mappose=map.response.map.info.origin;
//  std::cout << mappose <<std::endl;
  double xorigin=mappose.position.x;
  double yorigin=mappose.position.y; 

  int width=map.response.map.info.width;
  int height=map.response.map.info.height;
  int sizex = int (floor (width*resolution / cellsize));
  int sizey = int (floor (height*resolution / cellsize));
  printf("converting the map data to gridmap: cell size %f, res=%f, width=%d, height=%d, xorigin=%f, yorigin=%f, size gridmap (%d,%d)\n",cellsize, resolution, width, height, xorigin, yorigin, sizex, sizey);
//  printf("dns_namespace_uuid=%s",dns_namespace_uuid);
//  std::cout << dns_namespace_uuid<<std::endl;
//  std::cout << lUUIDNameGen(dns_namespace_uuid)<<std::endl;
//  std::cout << string_generator()("6ba7b810-9dad-11d1-80b4-00c04fd430c8") <<std::endl;
//  std::cout << lUUIDNameGen("vertex_0")<<std::endl;
  GMC = new GridMapCell(sizex, sizey, cellsize, xorigin, yorigin);
	int ii,jj;
	GMcell **gmap=GMC->GetMap();
	for (int j=0; j < width; j++)
	{
			for (int i=0; i < height; i++){
				ii=(int)floor(j*resolution/cellsize);
				jj=(int)floor(i*resolution/cellsize);
				if (ii<sizex && jj<sizey){
					if (gmap[ii][jj].visited==-1){
						gmap[ii][jj].visited=0;
						gmap[ii][jj].x=ii*cellsize+cellsize/2.+xorigin;
						gmap[ii][jj].y=jj*cellsize+cellsize/2.+yorigin;
						gmap[ii][jj].name="vertex_"+std::to_string(ii*sizey+jj);
//						boost::uuids::uuid lUUID=mUUIDGen();
						uuid lUUID = lUUIDNameGen(gmap[ii][jj].name);
						gmap[ii][jj].uuid=to_string(lUUID);
					}
					if ((gmap[ii][jj].occupancy==0)&&(map.response.map.data[i*width+j]!=0)){
						gmap[ii][jj].occupancy=1;
						gmap[ii][jj].staticcell=true;
					}
				}
			}
	}
//	GMC->spanningTree(13,13);
	
	//read annotations from file
	if (readAnnotations(annotation_file)!=1){
		std::cout << "shutting down the topology calculation..." <<'\n';
		return 0;
	}
	
	maptogridmap::GridmapCell gmcell;
	maptogridmap::Gridmap gm;
	maptogridmap::Nodes gmnode;
	maptogridmap::Edges gmedge;
	maptogridmap::Vertex vertex;
	maptogridmap::Edge edge;
	maptogridmap::Graph graph;
	gm.info.width=sizex;
	gm.info.height=sizey;
	gm.info.resolution=cellsize;
	gm.info.map_load_time = ros::Time::now();
	gm.header.frame_id = "map";
	gm.header.stamp = ros::Time::now();
	gmnode.header=gm.header;
	gmnode.info=gm.info;
	gmedge.header=gm.header;
	graph.header=gm.header;
	map_resp_.map.header=gm.header;
	map_resp_.map.info=gm.info;
	double tempx,tempy,midx,midy;
//  double thirdx,thirdy;
	geometry_msgs::Point p;
	int is,js,ig,jg;
//  double gx,gy;
  	int rofs[8]={ 0, -1, 0, 1, 1, -1, 1, -1};   
	int cofs[8]={ 1, 0, -1, 0, 1, 1, -1, -1};

  //write down the annotations into gridmap first
	for (int k=0; k<annotations.annotations.size(); k++){
		tempx = annotations.annotations[k].x-annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
		tempy = annotations.annotations[k].y-annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);
		is=floor((tempx-xorigin)/cellsize);
		js=floor((tempy-yorigin)/cellsize);
		if (is>=0 && js>=0 && is<sizex && js<sizey){
			//check if annotation vertex is occupied - do not do anything for the occupied annotations
			if (gmap[is][js].occupancy==0){
				gmap[is][js].annotation=true;
				gmap[is][js].x=tempx;
            	gmap[is][js].y=tempy;
            	gmap[is][js].theta=annotations.annotations[k].theta;
            	gmap[is][js].name=annotations.annotations[k].name;
				gmap[is][js].uuid=annotations.annotations[k].uuid; 
				//cells under the annotation arrow will be changed to occupied
				for (int l=0; l<5; l++){
					midx = annotations.annotations[k].x-l*0.2*annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
					midy = annotations.annotations[k].y-l*0.2*annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);
					ig=floor((midx-xorigin)/cellsize);
					jg=floor((midy-yorigin)/cellsize);
					if (ig>=0 && jg>=0 && ig<sizex && jg<sizey){
						if (gmap[ig][jg].annotation==false)
						{
							gmap[ig][jg].occupancy=1;
            				gmap[ig][jg].staticcell=true;
						}
					}
				}
			}else{
				ROS_ERROR("Annotations are occupied!!! Change the coordinates or distance of the annotation or decrease the cell size. Here are the details:");
				std::cout << annotations.annotations[k] <<std::endl;
				printf("Annotation vertex in the topology graph is at the occupied position (%f,%f) for the cell_size = %f m. Shutting down...\n",tempx,tempy,cellsize);
				return 0;
			}
		}
	}
	
	for (int i=0; i<sizex; i++){
		for (int j=0; j<sizey; j++){
			if (1){
				gm.x.push_back(gmap[i][j].x);
	            gm.y.push_back(gmap[i][j].y);
	            gm.occupancy.push_back(gmap[i][j].occupancy);
	            if (gmap[i][j].occupancy==0){


					if (gmap[i][j].annotation==false){
//check the cell is neighbor of the annotation vertex
						for (int d=0; d<8; d++){
							I_point point;
					        js=j+rofs[d];
					        is=i+cofs[d];
					        if (is>=0 && js>=0 && is<sizex && js<sizey){
					        	if ((gmap[is][js].occupancy==0) && (gmap[is][js].annotation)){
					        	//if neighbor cells are closer than the cellsize from the annotation vertex, hide them under the annotation vertex
					        		if ((fabs(gmap[i][j].x-gmap[is][js].x)<cellsize) && (fabs(gmap[i][j].y-gmap[is][js].y)<cellsize)){
					        			gmap[i][j].x=gmap[is][js].x;
        								gmap[i][j].y=gmap[is][js].y;
        								continue;
					        		}
					        	}
					        }
						}					
					}
					if (i!=floor((gmap[i][j].x-xorigin)/cellsize) || j!=floor((gmap[i][j].y-yorigin)/cellsize)){
            			continue;
            		}


//            	for (int k=0; k<annotations.annotations.size(); k++){
//            			tempx = annotations.annotations[k].x-annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
//						tempy = annotations.annotations[k].y-annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);
//            			midx = annotations.annotations[k].x-0.2*annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
//						midy = annotations.annotations[k].y-0.2*annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);
//            			thirdx = annotations.annotations[k].x-0.7*annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
//						thirdy = annotations.annotations[k].y-0.7*annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);

//        				is=floor((tempx-xorigin)/cellsize);
//						js=floor((tempy-yorigin)/cellsize);
//						if (is>=0 && js>=0 && is<sizex && js<sizey){
//						//check if annotation vertex is occupied - do not do anything for the occupied annotations
//							if (gmap[is][js].occupancy==0){
//							//coordinates of the cell centre (some cells have changed centre - close to annotations)
//								gx=i*cellsize+cellsize/2.+xorigin;
//								gy=j*cellsize+cellsize/2.+yorigin;
//        			if (((fabs(tempx-gx)<=cellsize) && (fabs(tempy-gy)<=cellsize)) && ((fabs(tempx-gx)>cellsize/2) || (fabs(tempy-gy)>cellsize/2)))
//        			{//neighbor cell to annotation cell
//        				gmap[i][j].x=tempx;
//        				gmap[i][j].y=tempy;
//        				continue;
//        			}
//            		if ( ((fabs(tempx-gx)<=cellsize/2) && (fabs(tempy-gy)<=cellsize/2)) || ((fabs(midx-gx)<=cellsize/2) && (fabs(midy-gy)<=cellsize/2)) || ((fabs(thirdx-gx)<=cellsize/2) && (fabs(thirdy-gy)<=cellsize/2))){
//            		//cells "under" the annotation arrow are skipped - artificial obstacle
//            			if ((fabs(tempx-gx)>cellsize/2) || (fabs(tempy-gy)>cellsize/2)){
//            				gmap[i][j].occupancy=1;
//            				gmap[i][j].staticcell=true;
//            				continue;
//            			}
//            			gmap[i][j].x=tempx;
//            			gmap[i][j].y=tempy;
//            			gmap[i][j].theta=annotations.annotations[k].theta;
//            			gmap[i][j].name=annotations.annotations[k].name;
////            			uuid lUUID = lUUIDNameGen(gmap[i][j].name);
//						gmap[i][j].uuid=annotations.annotations[k].uuid; //to_string(lUUID);
////						std::cout << gmap[i][j].uuid <<std::endl;
//            		}
//							}
//						}
//            	}
//            	if (gmap[i][j].occupancy){
//            		continue;
//            	}
//            	if (i!=floor((gmap[i][j].x-xorigin)/cellsize) || j!=floor((gmap[i][j].y-yorigin)/cellsize)){
//            		continue;
//            	}
					gmnode.x.push_back(gmap[i][j].x);
					gmnode.y.push_back(gmap[i][j].y);
					gmnode.theta.push_back(gmap[i][j].theta);
					gmnode.name.push_back(gmap[i][j].name);
					gmnode.uuid.push_back(gmap[i][j].uuid);
					vertex.x=gmap[i][j].x;
					vertex.y=gmap[i][j].y;
					vertex.theta=gmap[i][j].theta;
					vertex.name=gmap[i][j].name;
					vertex.uuid=gmap[i][j].uuid;
					//            	vertex.footprint.clear();
					//            	for (int d=0; d<4; d++){
					//            		p.x=vertex.x+cellsize/2.*xofs[d];
					//            		p.y=vertex.y+cellsize/2.*yofs[d];
					//            		vertex.footprint.push_back(p);
					//            	}
					graph.vertices.push_back(vertex);
		        }
		   		gmcell.x=gmap[i][j].x;
		   		gmcell.y=gmap[i][j].y;
		   		gmcell.occupancy=gmap[i][j].occupancy;
				gm.gmc.push_back(gmcell);
			}
		}
	}

			GMC->createEdges();
//			std::cout << GMC->edges.size() <<std::endl;
			std::cout << graph.vertices.size() <<std::endl;

			for(int i=0; i<GMC->edges.size(); i++){
				gmedge.uuid_src.push_back(gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid);
				gmedge.uuid_dest.push_back(gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid);
//				boost::uuids::uuid lUUID=mUUIDGen();
				edge.uuid_src=gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid;
				edge.uuid_dest=gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid;
				edge.name="edge_"+std::to_string(GMC->edges[i].xs*sizey+GMC->edges[i].ys)+"_"+std::to_string(GMC->edges[i].xg*sizey+GMC->edges[i].yg);
				uuid lUUID = lUUIDNameGen(edge.name);
				edge.uuid=to_string(lUUID);
				gmedge.uuid.push_back(edge.uuid);
				gmedge.name.push_back(edge.name);
				graph.edges.push_back(edge);
			}
	std::cout << graph.edges.size() <<std::endl;
	gmap_pub.publish(gm);
	nodes_pub.publish(gmnode);
	edges_pub.publish(gmedge);
	graph_pub.publish(graph);
	annotation_pub.publish(annotations);
	//map_resp_.map.gmc=gm.gmc;
//	std::cout << gm <<std::endl;

  ros::Rate rate(10.0);
  cycle_number=0;
  int update_nodes_edges;

  while (nh.ok()) {
    cycle_number++;
    update_nodes_edges = 0;
		for(int i = 0; i<obstacles.x.size(); i++){
			ii=(int)floor((obstacles.x[i]-xorigin)/cellsize);
			jj=(int)floor((obstacles.y[i]-yorigin)/cellsize);
			if (ii>=0 && jj>=0 && ii<sizex && jj<sizey){
	//			if ((gmap[ii][jj].occupancy==0) && (gmap[ii][jj].visited!=cycle_number))
				if ((gmap[ii][jj].occupancy<=50))
				{
	//				std::cout <<pointx[i]<<" "<<pointy[i]<<std::endl;
					gmap[ii][jj].occupancy = 100;
					gm.occupancy[ii*sizey+jj] = 1;
					update_nodes_edges = 1;
				}
			}	
		}
		for(int i = 0; i<obstacles1.x.size(); i++){
			ii=(int)floor((obstacles1.x[i]-xorigin)/cellsize);
			jj=(int)floor((obstacles1.y[i]-yorigin)/cellsize);
			if (ii>=0 && jj>=0 && ii<sizex && jj<sizey){
	//			if ((gmap[ii][jj].occupancy==0) && (gmap[ii][jj].visited!=cycle_number))
				if ((gmap[ii][jj].occupancy<=50))
				{
	//				std::cout <<pointx[i]<<" "<<pointy[i]<<std::endl;
					gmap[ii][jj].occupancy = 100;
					gm.occupancy[ii*sizey+jj] = 1;
					update_nodes_edges = 1;
				}
			}	
		}
		for(int i = 0; i<obstacles2.x.size(); i++){
			ii=(int)floor((obstacles2.x[i]-xorigin)/cellsize);
			jj=(int)floor((obstacles2.y[i]-yorigin)/cellsize);
			if (ii>=0 && jj>=0 && ii<sizex && jj<sizey){
	//			if ((gmap[ii][jj].occupancy==0) && (gmap[ii][jj].visited!=cycle_number))
				if ((gmap[ii][jj].occupancy<=50))
				{
	//				std::cout <<pointx[i]<<" "<<pointy[i]<<std::endl;
					gmap[ii][jj].occupancy = 100;
					gm.occupancy[ii*sizey+jj] = 1;
					update_nodes_edges = 1;
				}
			}	
		}

		for (int i=0; i<sizex; i++){
						for (int j=0; j<sizey; j++){
								if ((gmap[i][j].staticcell==false) && (gmap[i][j].occupancy>0)){
									gmap[i][j].occupancy=gmap[i][j].occupancy-1;
									if (gmap[i][j].occupancy==0)
										update_nodes_edges = 1;
								}
						}
		}

		if (update_nodes_edges){
			gmnode.x.clear();
			gmnode.y.clear();	
			gmnode.theta.clear();
			gmnode.uuid.clear();
			gmnode.name.clear();
			graph.vertices.clear();
			graph.edges.clear();
			for (int i=0; i<sizex; i++){
						for (int j=0; j<sizey; j++){
								gmap[i][j].visited=0;
				        if (gmap[i][j].occupancy==0){
				        	gmnode.x.push_back(gmap[i][j].x);
				        	gmnode.y.push_back(gmap[i][j].y);
		            	gmnode.theta.push_back(gmap[i][j].theta);
				        	gmnode.name.push_back(gmap[i][j].name);
				        	gmnode.uuid.push_back(gmap[i][j].uuid);
				        	vertex.x=gmap[i][j].x;
							vertex.y=gmap[i][j].y;
							vertex.theta=gmap[i][j].theta;
							vertex.name=gmap[i][j].name;
							vertex.uuid=gmap[i][j].uuid;
//							vertex.footprint.clear();
//							for (int d=0; d<4; d++){
//								p.x=vertex.x+cellsize/2.*xofs[d];
//								p.y=vertex.y+cellsize/2.*yofs[d];
//								vertex.footprint.push_back(p);
//							}
							graph.vertices.push_back(vertex);
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
//						boost::uuids::uuid lUUID=mUUIDGen();
						edge.uuid_src=gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid;
						edge.uuid_dest=gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid;
						edge.name="edge_"+std::to_string(GMC->edges[i].xs*sizey+GMC->edges[i].ys)+"_"+std::to_string(GMC->edges[i].xg*sizey+GMC->edges[i].yg);
						uuid lUUID = lUUIDNameGen(edge.name);
						edge.uuid=to_string(lUUID);
						gmedge.uuid.push_back(edge.uuid);
						gmedge.name.push_back(edge.name);
						graph.edges.push_back(edge);
			}	
		}

		if ((cycle_number % 20)==0 || update_nodes_edges){
		gmnode.header.stamp = ros::Time::now();
		gm.header.stamp = ros::Time::now();
		gmedge.header.stamp = ros::Time::now();
		graph.header.stamp = ros::Time::now();
		gmap_pub.publish(gm);
		nodes_pub.publish(gmnode);
		edges_pub.publish(gmedge);
		graph_pub.publish(graph);
		annotation_pub.publish(annotations);
		}

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
//			double cellsize=GMC->GetSizeCell();
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


//edges
			if(GMC->edges.size()>0){
				int temp_length=GMC->edges.size();
				for(int pathLength=0; pathLength<temp_length;pathLength++){
//			        	p.x = GMC->edges[pathLength].xs*cellsize+0.5*cellsize;
//	    				p.y = GMC->edges[pathLength].ys*cellsize+0.5*cellsize;
	    				p.x = gmap[GMC->edges[pathLength].xs][GMC->edges[pathLength].ys].x;
	    				p.y = gmap[GMC->edges[pathLength].xs][GMC->edges[pathLength].ys].y;
    					stc.points.push_back(p);
//			        	p.x = GMC->edges[pathLength].xg*cellsize+0.5*cellsize;
//	    				p.y = GMC->edges[pathLength].yg*cellsize+0.5*cellsize;
	    				p.x = gmap[GMC->edges[pathLength].xg][GMC->edges[pathLength].yg].x;
	    				p.y = gmap[GMC->edges[pathLength].xg][GMC->edges[pathLength].yg].y;
    					stc.points.push_back(p);
    					
				}
			//publish path			
			}
			stc_pub.publish(stc);
			//points from newObstacles
			for(int i = 0; i<obstacles.x.size(); i++){
				p.x = obstacles.x[i];
				p.y = obstacles.y[i];
				glp.points.push_back(p);
			}
			for(int i = 0; i<obstacles1.x.size(); i++){
				p.x = obstacles1.x[i];
				p.y = obstacles1.y[i];
				glp.points.push_back(p);
			}
			for(int i = 0; i<obstacles2.x.size(); i++){
				p.x = obstacles2.x[i];
				p.y = obstacles2.y[i];
				glp.points.push_back(p);
			}
			globalpoints_pub.publish(glp);

			//points from annotations
			geometry_msgs::Pose pose; 
			for(int i = 0; i<annotations.annotations.size(); i++){
				pose.position.x = annotations.annotations[i].x-annotations.annotations[i].distance*cos(annotations.annotations[i].theta*M_PI/180.);
				pose.position.y = annotations.annotations[i].y-annotations.annotations[i].distance*sin(annotations.annotations[i].theta*M_PI/180.);
				pose.position.z = 0;
				pose.orientation = tf::createQuaternionMsgFromYaw(annotations.annotations[i].theta*M_PI/180.);
				annt.scale.x = annotations.annotations[i].distance;
				annt.pose = pose;
		    annt.header.stamp = ros::Time::now();
		    annt.id = i;
				annt_pub.publish(annt);
			}
}
