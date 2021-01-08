#include "gmap.h"

void GridMapCell::spanningTree(int r, int c){

  std::vector<I_point> neighbour;
  neighbour.reserve(100000); 
  neighbour.clear();
  I_point temp;
  temp.x=c;
  temp.y=r;
  neighbour.push_back(temp);

  spanningpath.clear();
  
  map[c][r].visited=1;
  int rofs[4]={ 0, -1, 0, 1};   
  int cofs[4]={ 1, 0, -1, 0};
  int num_n=neighbour.size();
  int temp_start=0;
  int in=temp_start;
  uint maxspanninglength=200000000;

   while (in<num_n){
//   		 printf("neighbor tree size %d\n",neighbour.size());
		 if (spanningpath.size()>maxspanninglength){
		 	break;
		 }
         temp=neighbour[in]; 
				 for (int d = 0; d<4; d++){
                  	I_point point;
					        point.y=temp.y+rofs[d];
					        point.x=temp.x+cofs[d];
//					        printf("point (%d,%d) visited=%d, occupancy=%d\n",point.x,point.y,map[point.x][point.y].visited, map[point.x][point.y].occupancy);
                     if (point.x>0 && point.y>0 && point.x<MapSizeX && point.y<MapSizeY){
                         if ((map[point.x][point.y].occupancy==0)&&(map[point.x][point.y].visited==0)){
                             neighbour.push_back(point);
                             map[point.x][point.y].visited=1;
                         }
                     }
                 }
                 in=in+1;
                 num_n=neighbour.size();
                 temp_start=temp_start+1;
             }
//	printf("spanning tree size %d neighbour tree size %d\n",spanningpath.size(),neighbour.size());

  
}

void GridMapCell::createEdges(){

	edges.clear();
	int rofs[4]={ 0, -1, 0, 1};   
	int cofs[4]={ 1, 0, -1, 0};
	int is,js,ig,jg;
	bool duplicate_edge;
	for (int i=0; i<MapSizeX; i++){
		for (int j=0; j<MapSizeY; j++){
				
			if ((map[i][j].occupancy==0)){
				I_edge edge;
				edge.xs=i;
				edge.ys=j;
				map[i][j].visited=1;
				for (int d = 0; d<4; d++){
        	I_point point;
					point.y=j+rofs[d];
					point.x=i+cofs[d];
					if (point.x>=0 && point.y>=0 && point.x<MapSizeX && point.y<MapSizeY){
						if ((map[point.x][point.y].occupancy==0)&&(map[point.x][point.y].visited==0)){
							edge.xg=point.x;
							edge.yg=point.y;
							is=floor((map[edge.xs][edge.ys].x-xorigin)/size_cell);
							js=floor((map[edge.xs][edge.ys].y-yorigin)/size_cell);
							ig=floor((map[edge.xg][edge.yg].x-xorigin)/size_cell);
							jg=floor((map[edge.xg][edge.yg].y-yorigin)/size_cell);
							if (is>=0 && js>=0 && is<MapSizeX && js<MapSizeY && ig>=0 && jg>=0 && ig<MapSizeX && jg<MapSizeY){
						 		if ((fabs(map[edge.xs][edge.ys].x-map[point.x][point.y].x)<size_cell/2) && (fabs(map[edge.xs][edge.ys].y-map[point.x][point.y].y)<size_cell/2)){
					   		}else{
					        		
									if ((is!=edge.xs || js!=edge.ys || ig!=edge.xg || jg!=edge.yg)) {
										//added condition that start and goal vertices are not equal
										if ((map[is][js].occupancy==0) && (map[ig][jg].occupancy==0) && ((is!=ig) || (js!=jg))){
											edge.xs=is;
											edge.ys=js;
											edge.xg=ig;
											edge.yg=jg;
											//test for duplicates
											duplicate_edge=false;
											for (uint k=0; k<edges.size();k++){
												if ((edges[k].xs==edge.xs) && (edges[k].ys==edge.ys) && (edges[k].xg==edge.xg) && (edges[k].yg==edge.yg)){
//														printf("duplicate edge exists (%d,%d) (%d,%d)\n",is,js,ig,jg);
													duplicate_edge=true;
													break;
												}
											}
											if (!duplicate_edge){
												edges.push_back(edge);
											}
										}
					       	}else{
                  	edges.push_back(edge);
                  }
								}
							}
                            	
						}
					}
				}
			}
		}
	}

}
