#include "gmap.h"

void GridMapCell::spanningTree(int r, int c){

  std::vector<I_point> neighbour;
  neighbour.reserve(100); 
  neighbour.clear();
  I_point temp, temp2;
  temp.x=c;
  temp.y=r;
  neighbour.push_back(temp);

  spanningpath.clear();
  
  map[c][r].visited=1;
  int num_put=0;
  int rofs[4]={ 0, -1, 0, 1};   
  int cofs[4]={ 1, 0, -1, 0};
  int num_n=neighbour.size();
  int temp_start=0;
  int in=temp_start;

   while (in<num_n){
         temp=neighbour[in]; 
                 if (temp_start==0){
                     spanningpath.push_back(temp); 
                 } else{
                    for (int ti=in; ti<num_n; ti++){        //BFS
                         temp2=neighbour[ti];
                         if (abs(spanningpath[num_put].x-temp2.x)+abs(spanningpath[num_put].y-temp2.y)<=1){
                            num_put=num_put+1;
                            spanningpath.push_back(temp2);
                            neighbour[in]=temp2;
                            neighbour[ti]=temp;
                            temp.x=temp2.x;
                            temp.y=temp2.y;
                            break;
                         }
                     }

                     if (abs(spanningpath[num_put].x-temp2.x)+abs(spanningpath[num_put].y-temp2.y)>1){
                         int temp_length=num_put;
                         for (int tp=0; tp<temp_length; tp++){
                             num_put=num_put+1;
                             spanningpath.push_back(spanningpath[temp_length-tp-1]);//
                             for (int ti=in; ti<num_n; ti++){ 
                                 temp2=neighbour[ti];
                                 if (abs(spanningpath[num_put].x-temp2.x)+abs(spanningpath[num_put].y-temp2.y)<=1){
                                    num_put=num_put+1;
                                    spanningpath.push_back(temp2);
                                    neighbour[in]=temp2;
                                    neighbour[ti]=temp;
                                    temp.x=temp2.x;
                                    temp.y=temp2.y;
                                    break;
                                 }
                             }
                             if (abs(spanningpath[num_put].x-temp2.x)+abs(spanningpath[num_put].y-temp2.y)<=1){
                                 break;
                             }
                         }
                     }
                 }
               
				        for (int d = 0; d<4; d++){
                  I_point point;
					        point.y=temp.y+rofs[d];
					        point.x=temp.x+cofs[d];
                     if (point.x>0 && point.y>0 && point.x<MapSizeX && point.y<MapSizeY){
                         if ((map[point.x][point.y].occupancy==0)&&(map[point.x][point.y].visited==0)){
                             neighbour.push_back(point);//dodaj ga u skupinu povezanih
                             map[point.x][point.y].visited=1;
                         }
                     }
                 }
                 in=in+1;
                 num_n=neighbour.size();
                 temp_start=temp_start+1;
             }

  
}

void GridMapCell::createEdges(){

	edges.clear();
	int rofs[4]={ 0, -1, 0, 1};   
	int cofs[4]={ 1, 0, -1, 0};
	printf("size_cell = %f, origin = (%f, %f)\n",size_cell, xorigin, yorigin);
	int is,js,ig,jg;
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
					        		if ((fabs(map[edge.xs][edge.ys].x-map[point.x][point.y].x)<size_cell/2) && (fabs(map[edge.xs][edge.ys].y-map[point.x][point.y].y)<size_cell/2)){
//										printf("slicni su (%f,%f) (%f,%f)\n",map[edge.xs][edge.ys].x,map[edge.xs][edge.ys].y,map[edge.xg][edge.yg].x,map[edge.xg][edge.yg].y);
//										printf("indeks u gridmapi (%d,%d) (%d,%d)\n",edge.xs,edge.ys,edge.xg,edge.yg);
//										printf("provjera indeksa iz realnih koordinata (%d,%d) (%d,%d)\n",is,js,ig,jg);
					        		}else{
					        		
										if (is!=edge.xs || js!=edge.ys || ig!=edge.xg || jg!=edge.yg){
//											printf("\n\nAAAAAAAAAAAAAAAAAAAAAAAAAAAAa\n");
//											printf("realne koordinate (%f,%f) (%f,%f)\n",map[edge.xs][edge.ys].x,map[edge.xs][edge.ys].y,map[edge.xg][edge.yg].x,map[edge.xg][edge.yg].y);
//											printf("indeks u gridmapi (%d,%d) (%d,%d)\n",edge.xs,edge.ys,edge.xg,edge.yg);
//											printf("provjera indeksa iz realnih koordinata (%d,%d) (%d,%d)\n",is,js,ig,jg);
											edge.xs=is;
											edge.ys=js;
											edge.xg=ig;
											edge.yg=jg;
					        			}
                            			edges.push_back(edge);
                            		}
                            	
                         		}
                         	}
					    }
					}
				}
	}

}
