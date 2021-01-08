#include <gmap.h>

#define USE_SPANNING_TREE 1
//static boost::uuids::random_generator mUUIDGen;
//static const std::wstring DNS_NAMESPACE_UUID = L"6ba7b810-9dad-11d1-80b4-00c04fd430c8";
using namespace boost::uuids;
uuid dns_namespace_uuid;
//name_generator lUUIDNameGen(dns_namespace_uuid);
name_generator lUUIDNameGen(string_generator()("6ba7b810-9dad-11d1-80b4-00c04fd430c8"));
//boost::uuids::name_generator lUUIDNameGen = boost::uuids::name_generator(mUUIDStringGen(DNS_NAMESPACE_UUID));

GridMapCell *GMC;
int cycle_number;
mapupdates::NewObstacles obstacles;
mapupdates::NewObstacles obstacles1; //for robot_1
mapupdates::NewObstacles obstacles2; //for robot_2
maptogridmap::Annotations annotations;
maptogridmap::Graph graph;


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
    annt.color.r = 1.;
    annt.color.g = 1.;
    annt.color.b = 0.;
    annt.color.a = 1.0;
    annt.lifetime = ros::Duration(0.);

  }

  void visualizationduringmotion();


};

void newObstaclesCallback(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	obstacles.x.clear();
	obstacles.y.clear();
	for (uint i =0; i<msg->x.size(); i++){
		obstacles.x.push_back(msg->x[i]);
		obstacles.y.push_back(msg->y[i]);
	}
}

void newObstaclesCallback1(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	obstacles1.x.clear();
	obstacles1.y.clear();
	for (uint i =0; i<msg->x.size(); i++){
		obstacles1.x.push_back(msg->x[i]);
		obstacles1.y.push_back(msg->y[i]);
	}
}

void newObstaclesCallback2(const mapupdates::NewObstaclesConstPtr& msg)
{
//	std::cout << msg->x.size()<<std::endl;
	obstacles2.x.clear();
	obstacles2.y.clear();
	for (uint i =0; i<msg->x.size(); i++){
		obstacles2.x.push_back(msg->x[i]);
		obstacles2.y.push_back(msg->y[i]);
	}
}

int readAnnotations(std::string annotation_file)
{
	char *line;
	char *word;
	char illegalword[35]="<>\"\'=;()+-\\* /#$&,.!:?@[]^`{|}~";
	bool validname, unchanged=true;
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
				validname=true;
				for (int it=0; word[it]!='\0'; it++){
					for (int jt=0; illegalword[jt]!='\0';jt++){
						if (word[it]==illegalword[jt]){
//							ROS_ERROR("Annotation name contains an illegal character. Here are the details:");
							ROS_WARN("Annotation name contains an illegal character. Here are the details:");
							std::cout << "The annotation name \""<< word << "\" contains the illegal character \"" << word[it]<< "\"."<< std::endl;
							std::cout << "Please name annotations only with a combination of letters, numbers and _ (underscore)."<< std::endl;
							std::cout << "Do not use any character from the list of illegal characters: " <<std::endl;
							std::cout << illegalword <<std::endl;
							annt.name = "location_"+std::to_string(annotations.annotations.size());
							std::cout << "Renaming to " << annt.name << std::endl;
							validname=false;
							unchanged=false;
							break;
//							return 0;
						}
					}
					if (!validname)
						break;
				}
				if (validname)
					annt.name=word;
				for (uint k=0; k<annotations.annotations.size(); k++){
						if (annotations.annotations[k].name.compare(annt.name)==0){
							ROS_WARN("Two annotations have the same name! Names must be unique! Here are the details:");
							std::cout << "The annotation " << annotations.annotations.size()+1 <<" has the same name as the annotation " << k+1 << std::endl;
							std::cout << annt <<std::endl;
							std::cout << annotations.annotations[k] <<std::endl;
							validname=false;
							unchanged=false;
							annt.name = "location_"+std::to_string(annotations.annotations.size());
							std::cout << "Renaming to " << annt.name << std::endl;
							break;
						}
				}
				uuid lUUID = lUUIDNameGen(word);
				annt.uuid=to_string(lUUID);
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
			if ((strcmp(word,"point_x ")==0) || (strcmp(word,"point_x")==0)){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
					annt.x=(atof(word));
				}
			}
			if ((strcmp(word,"point_y ")==0)||(strcmp(word,"point_y")==0)){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
					annt.y=(atof(word));
				}
			}
			if ((strcmp(word,"theta ")==0)||(strcmp(word,"theta")==0)){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
					annt.theta=(atof(word));
				}
			}
			if ((strcmp(word,"distance ")==0)||(strcmp(word,"distance")==0)){
				word = strtok (NULL," ");
				if (word!= NULL){
//					std::cout << word <<std::endl;
					annt.distance=(atof(word));
					annotations.annotations.push_back(annt);
				}
			}
		}    

  }

	std::cout << annotations <<std::endl;
	return (unchanged);
}

int annotationsToGraph(int file_flag){

	maptogridmap::Vertex vertex;
	maptogridmap::Edge edge;

	GMcell **gmap=GMC->GetMap();
  int sizex=GMC->GetMapSizeX();
  int sizey=GMC->GetMapSizeY();
	double cellsize=GMC->GetSizeCell();
  double xorigin=GMC->GetOriginX();
  double yorigin=GMC->GetOriginY(); 

	double tempx,tempy,midx,midy;
	double distance;
	int is,js,ig,jg;
  int rofs[8]={ 0, -1, 0, 1, 1, -1, 1, -1};   
	int cofs[8]={ 1, 0, -1, 0, 1, 1, -1, -1};
	
	bool unchanged=true;

	graph.header.frame_id = "map";
	graph.header.stamp = ros::Time::now();
	graph.vertices.clear();
	graph.edges.clear();
	
  //write down the annotations into gridmap first
	uint k=0;
	while (k<annotations.annotations.size()){
		tempx = annotations.annotations[k].x-annotations.annotations[k].distance*cos(annotations.annotations[k].theta*M_PI/180.);
		tempy = annotations.annotations[k].y-annotations.annotations[k].distance*sin(annotations.annotations[k].theta*M_PI/180.);
		is=floor((tempx-xorigin)/cellsize);
		js=floor((tempy-yorigin)/cellsize);
		if (is>=0 && js>=0 && is<sizex && js<sizey){
			//check if annotation vertex is occupied - do not do anything for the occupied annotations
			if (gmap[is][js].occupancy==0){
			
				//check if any other annotation falls into the same grid cell and if annotations are closer than the cell size (requirement from TP)
				for (uint l=k+1; l<annotations.annotations.size(); l++){
					midx = annotations.annotations[l].x-annotations.annotations[l].distance*cos(annotations.annotations[l].theta*M_PI/180.);
					midy = annotations.annotations[l].y-annotations.annotations[l].distance*sin(annotations.annotations[l].theta*M_PI/180.);
					ig=floor((midx-xorigin)/cellsize);
					jg=floor((midy-yorigin)/cellsize);
					if ((ig==is) && (jg==js)){
						ROS_WARN("Annotations fall into the same grid cell!!! Deleting this annotation... Change the coordinates of the annotation or decrease the cell size. Here are the details:");
						std::cout << "The annotation " << annotations.annotations[k].name <<" is too close to the annotation " << annotations.annotations[l].name << std::endl;
						std::cout << annotations.annotations[k] <<std::endl;
						std::cout << annotations.annotations[l] <<std::endl;
						std::cout << "Both annotations fall into the same topology vertex at ("<<is<<", "<<js<<" )" <<std::endl;
						annotations.annotations.erase(annotations.annotations.begin()+l);
						unchanged=false;
						break;
					}
//					std::cout << "The annotation "<< annotations.annotations[k].name <<" has the topology vertex at ("<<tempx<<", "<<tempy<<" ), while the topology vertex at ("<<midx<<", "<<midy<<" ) of the annotation " << annotations.annotations[l].name <<std::endl;
					distance=sqrt(pow((tempx-midx),2)+pow((tempy-midy),2));
//					std::cout << distance << std::endl;
					if (distance<cellsize){
						ROS_WARN("Annotations are too close!!! Deleting this annotation... Change the coordinates of the annotation or decrease the cell size. Here are the details:");
						std::cout << "The annotation " << annotations.annotations[k].name <<" is too close to the annotation " << annotations.annotations[l].name << std::endl;
						std::cout << annotations.annotations[k] <<std::endl;
						std::cout << annotations.annotations[l] <<std::endl;
						std::cout << "The annotation "<< annotations.annotations[k].name <<" has the topology vertex at ("<<tempx<<", "<<tempy<<" ), which is distanced for " << distance << "(less than the cell_size = "<< cellsize <<") from the topology vertex at ("<<midx<<", "<<midy<<" ) of the annotation " << annotations.annotations[l].name <<std::endl;
						annotations.annotations.erase(annotations.annotations.begin()+l);
						unchanged=false;
						break;
					}
					
				}
					
				gmap[is][js].annotation=true;
				gmap[is][js].x=tempx;
        gmap[is][js].y=tempy;
        gmap[is][js].theta=annotations.annotations[k].theta;
        gmap[is][js].name=annotations.annotations[k].name;
				gmap[is][js].uuid=annotations.annotations[k].uuid; 
#if USE_SPANNING_TREE
				if (gmap[is][js].visited==0)
					GMC->spanningTree(js,is);
#endif
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
				std::cout << annotations.annotations[k] <<std::endl;
				if (file_flag){
					ROS_WARN("Annotation is occupied!!! Deleting occupied annotation... Add new annotation by pressing the 2D Nav Goal button in RViz or exit and edit manually annotations.ini file or decrease the cell size. Here are the details:");
					printf("The vertex of the annotation in the topology graph is at the occupied position (%f,%f) for the cell_size = %f m.\n",tempx,tempy,cellsize);
					annotations.annotations.erase(annotations.annotations.begin()+k);
					unchanged=false;
					continue;
				}else{
					ROS_WARN("Annotation is occupied!!! Deleting occupied annotation... Change the coordinates or exit and edit manually annotations.ini file or decrease the cell size. Here are the details:");
					printf("The vertex of the annotation in the topology graph is at the occupied position (%f,%f) for the cell_size = %f m.\n",tempx,tempy,cellsize);
					annotations.annotations.erase(annotations.annotations.begin()+k);
					unchanged=false;
					continue;
				}
			}
		}
		k++;
	}
	
	if (annotations.annotations.size()){
		for (int i=0; i<sizex; i++){
			for (int j=0; j<sizey; j++){
        if (gmap[i][j].occupancy==0){

					if (gmap[i][j].annotation==false){
//check if this cell is neighbor of the annotation vertex
						for (int d=0; d<8; d++){
					        js=j+rofs[d];
					        is=i+cofs[d];
					        if (is>=0 && js>=0 && is<sizex && js<sizey){
					        	if ((gmap[is][js].occupancy==0) && (gmap[is][js].annotation)){
					        	//if neighbor cells are closer than the cellsize from the annotation vertex, hide them under the annotation vertex -- reqirement from TP
					        		if ((gmap[i][j].x-gmap[is][js].x)*(gmap[i][j].x-gmap[is][js].x)+(gmap[i][j].y-gmap[is][js].y)*(gmap[i][j].y-gmap[is][js].y)<cellsize*cellsize){
					        			gmap[i][j].x=gmap[is][js].x;
        								gmap[i][j].y=gmap[is][js].y;
        								continue;
					        		}
					        	}
					        }
						}					
					}
					if (i!=floor((gmap[i][j].x-xorigin)/cellsize) || j!=floor((gmap[i][j].y-yorigin)/cellsize)){
						gmap[i][j].visited=0;
//						std::cout << "i= "<< is << "j= "<<js << std::endl;
            continue;
          }


#if USE_SPANNING_TREE
					if (gmap[i][j].visited==1){//only part of the graph that is connected to annotations
						gmap[i][j].visited=0;
#endif
						vertex.x=gmap[i][j].x;
						vertex.y=gmap[i][j].y;
						vertex.theta=gmap[i][j].theta;
						vertex.name=gmap[i][j].name;
						vertex.uuid=gmap[i][j].uuid;
						graph.vertices.push_back(vertex);
#if USE_SPANNING_TREE
					}else{ //the part of the graph disconnected from annotations is occupied
						gmap[i][j].occupancy=1;
            gmap[i][j].staticcell=true;
					}
#endif
		    }
			}
		}
	}
	if (annotations.annotations.size()){
			GMC->createEdges();
	}
//			std::cout << GMC->edges.size() <<std::endl;
			std::cout << "number of vertices: "<<graph.vertices.size() <<std::endl;

			for(uint i=0; i<GMC->edges.size(); i++){
//				boost::uuids::uuid lUUID=mUUIDGen();
				edge.uuid_src=gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid;
				edge.uuid_dest=gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid;
				edge.name="edge_"+std::to_string(GMC->edges[i].xs*sizey+GMC->edges[i].ys)+"_"+std::to_string(GMC->edges[i].xg*sizey+GMC->edges[i].yg);
				uuid lUUID = lUUIDNameGen(edge.name);
				edge.uuid=to_string(lUUID);
				graph.edges.push_back(edge);
			}
	std::cout << "number of edges: "<< graph.edges.size() <<std::endl;

	return unchanged;
}

void newAnnotationCallback(const geometry_msgs::PoseStamped::ConstPtr& goal){

	maptogridmap::Annotation annt;
	annt.x = goal->pose.position.x; //this is already postion of the vertex in the graph and needs to be transformed to tail of the arrow
	annt.y = goal->pose.position.y;
	annt.theta = tf::getYaw(goal->pose.orientation);
	annt.name = "location_"+std::to_string(annotations.annotations.size());
	annt.distance = 2.;
	uuid lUUID = lUUIDNameGen(annt.name);
  annt.uuid=to_string(lUUID);
	
	//input annotations into graph
	annt.x = annt.x + annt.distance*cos(annt.theta); //calculating the tail of the arrow
	annt.y = annt.y + annt.distance*sin(annt.theta);
	annt.theta = annt.theta*180/M_PI; //the user sets degrees in annotations.ini file
	annotations.annotations.push_back(annt);
	if (annotationsToGraph(0)){
	//print into annotation ini file:
		std::ofstream out("annotationsRviz.ini");
		for (uint k=0; k<annotations.annotations.size(); k++){
			out << "["<<annotations.annotations[k].name<<"]" <<std::endl;
			out << "point_x = "<<annotations.annotations[k].x<<std::endl;
			out << "point_y = "<<annotations.annotations[k].y<<std::endl;
			out << "theta = "<<annotations.annotations[k].theta<<std::endl;
			out << "distance = "<<annotations.annotations[k].distance<<std::endl<<std::endl;
		}
		out.close();

		std::cout << "New selected annotation:" <<std::endl<<std::endl;
		std::cout << annt <<std::endl;
		
	}
}


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "maptogridmap");
  ros::NodeHandle nh;
  maptogridmap::Vertex vertex;
	maptogridmap::Edge edge;
	double cellsize=2.;

  ros::Publisher graph_pub = nh.advertise<maptogridmap::Graph>("map/graph",1);
  ros::Publisher annotation_pub = nh.advertise<maptogridmap::Annotations>("map/annotations",1);
  ros::Subscriber gmu_sub = nh.subscribe("/robot_0/newObstacles",1,newObstaclesCallback);
  ros::Subscriber gmu_sub1 = nh.subscribe("/robot_1/newObstacles",1,newObstaclesCallback1);
  ros::Subscriber gmu_sub2 = nh.subscribe("/robot_2/newObstacles",1,newObstaclesCallback2);
  ros::Subscriber annt_sub = nh.subscribe("/annotation/goal",1,newAnnotationCallback);

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
	
	//read annotations from file
	if (readAnnotations(annotation_file)!=1){
		ROS_WARN("Read the output carefully since some annotations from the file were renamed due to illegal character or duplicate naming! Add new annotation by pressing the 2D Nav Goal button in RViz and select the position and hold to select the orientation of the goal!");
	}
	//input annotations into graph
	if (annotationsToGraph(1)!=1){
//		std::cout << annotations <<std::endl;
		ROS_WARN("Read the output carefully since some annotations from the file were deleted. Add new annotation by pressing the 2D Nav Goal button in RViz and select the position and hold to select the orientation of the goal!");
	}else{
		if (annotations.annotations.size()==0){
			ROS_WARN("No annotations yet! Add new annotation by pressing the 2D Nav Goal button in RViz and select the position and hold to select the orientation of the goal!");
		} else {
			std::cout << "All annotations from the annotations.ini file are valid! You can add more annotations by pressing the 2D Nav Goal button in RViz and select the position and hold to select the orientation of the goal!" << std::endl;
		}
	}
	
	graph_pub.publish(graph);
	annotation_pub.publish(annotations);

  ros::Rate rate(10.0);
  cycle_number=0;
  int update_nodes_edges;

  while (nh.ok()) {
    cycle_number++;
    update_nodes_edges = 0;
		for(uint i = 0; i<obstacles.x.size(); i++){
			ii=(int)floor((obstacles.x[i]-xorigin)/cellsize);
			jj=(int)floor((obstacles.y[i]-yorigin)/cellsize);
			if (ii>=0 && jj>=0 && ii<sizex && jj<sizey){
				if ((gmap[ii][jj].occupancy<=50))
				{
					gmap[ii][jj].occupancy = 100;
					update_nodes_edges = 1;
				}
			}	
		}
		for(uint i = 0; i<obstacles1.x.size(); i++){
			ii=(int)floor((obstacles1.x[i]-xorigin)/cellsize);
			jj=(int)floor((obstacles1.y[i]-yorigin)/cellsize);
			if (ii>=0 && jj>=0 && ii<sizex && jj<sizey){
				if ((gmap[ii][jj].occupancy<=50))
				{
					gmap[ii][jj].occupancy = 100;
					update_nodes_edges = 1;
				}
			}	
		}
		for(uint i = 0; i<obstacles2.x.size(); i++){
			ii=(int)floor((obstacles2.x[i]-xorigin)/cellsize);
			jj=(int)floor((obstacles2.y[i]-yorigin)/cellsize);
			if (ii>=0 && jj>=0 && ii<sizex && jj<sizey){
				if ((gmap[ii][jj].occupancy<=50))
				{
					gmap[ii][jj].occupancy = 100;
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
			graph.vertices.clear();
			graph.edges.clear();
			for (int i=0; i<sizex; i++){
				for (int j=0; j<sizey; j++){
					gmap[i][j].visited=0;
	        if (gmap[i][j].occupancy==0){
	        	vertex.x=gmap[i][j].x;
						vertex.y=gmap[i][j].y;
						vertex.theta=gmap[i][j].theta;
						vertex.name=gmap[i][j].name;
						vertex.uuid=gmap[i][j].uuid;
						graph.vertices.push_back(vertex);
	        }
				}
			}
			GMC->createEdges();
			for(uint i=0; i<GMC->edges.size(); i++){
//						boost::uuids::uuid lUUID=mUUIDGen();
						edge.uuid_src=gmap[GMC->edges[i].xs][GMC->edges[i].ys].uuid;
						edge.uuid_dest=gmap[GMC->edges[i].xg][GMC->edges[i].yg].uuid;
						edge.name="edge_"+std::to_string(GMC->edges[i].xs*sizey+GMC->edges[i].ys)+"_"+std::to_string(GMC->edges[i].xg*sizey+GMC->edges[i].yg);
						uuid lUUID = lUUIDNameGen(edge.name);
						edge.uuid=to_string(lUUID);
						graph.edges.push_back(edge);
			}	
		}

		if ((cycle_number % 20)==0) { // || update_nodes_edges){
		graph.header.stamp = ros::Time::now();
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
      if ((cycle_number % 20)==0){
			for (int i=0; i<sizex; i++){
				for (int j=0; j<sizey; j++){
					if ((gmap[i][j].occupancy>0)){
						p.x=gmap[i][j].x;
            p.y=gmap[i][j].y;
						gridmapvs.points.push_back(p);
					}else{
						p.x=gmap[i][j].x;
	          p.y=gmap[i][j].y;
						graphvs.points.push_back(p);
					}
				}
			}
			gridmapvs_pub.publish(gridmapvs);
			graphvs_pub.publish(graphvs);


//edges
			if(GMC->edges.size()>0){
				uint temp_length=GMC->edges.size();
				for(uint pathLength=0; pathLength<temp_length;pathLength++){
	    				p.x = gmap[GMC->edges[pathLength].xs][GMC->edges[pathLength].ys].x;
	    				p.y = gmap[GMC->edges[pathLength].xs][GMC->edges[pathLength].ys].y;
    					stc.points.push_back(p);
	    				p.x = gmap[GMC->edges[pathLength].xg][GMC->edges[pathLength].yg].x;
	    				p.y = gmap[GMC->edges[pathLength].xg][GMC->edges[pathLength].yg].y;
    					stc.points.push_back(p);
    					
				}
			//publish path	
				stc_pub.publish(stc);
			}
		}
		
			//points from newObstacles
			for(uint i = 0; i<obstacles.x.size(); i++){
				p.x = obstacles.x[i];
				p.y = obstacles.y[i];
				glp.points.push_back(p);
			}
			for(uint i = 0; i<obstacles1.x.size(); i++){
				p.x = obstacles1.x[i];
				p.y = obstacles1.y[i];
				glp.points.push_back(p);
			}
			for(uint i = 0; i<obstacles2.x.size(); i++){
				p.x = obstacles2.x[i];
				p.y = obstacles2.y[i];
				glp.points.push_back(p);
			}
			if (glp.points.size()>0)
			globalpoints_pub.publish(glp);

			//points from annotations
			geometry_msgs::Pose pose; 
			for(uint i = 0; i<annotations.annotations.size(); i++){
				pose.position.x = annotations.annotations[i].x-annotations.annotations[i].distance*cos(annotations.annotations[i].theta*M_PI/180.);
				pose.position.y = annotations.annotations[i].y-annotations.annotations[i].distance*sin(annotations.annotations[i].theta*M_PI/180.);
				pose.position.z = 0;
				pose.orientation = tf::createQuaternionMsgFromYaw(annotations.annotations[i].theta*M_PI/180.);
				annt.scale.x = std::max(annotations.annotations[i].distance,0.5);
				annt.pose = pose;
		    annt.header.stamp = ros::Time::now();
		    annt.id = i;
				annt_pub.publish(annt);
			}
}
