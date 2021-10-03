// Keeghan Campbell, 18032681
// Sophie Francis,   16062898
///////////////////////////////////////////////////////////////////////////////////////////
//
//
//  
//                        
//
//	 	      Program Name: Incremental Search 
//	 	       Description: start-up code for simulating LPA* and D*Lite
//                        - implements a gridworld that can be loaded from file, and 
//                          modified through a user-interface 
//
//        Run Parameters: 
//
//    Keys for Operation: 
//
//	 		        History:  date of revision
//                         11/Aug/2021
//                         09/Aug/2020  
//                         28/July/2015  
//                         03/Aug/2014  
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////


#include <windows.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <deque>
#include <set>
#include <vector>
#include <chrono>

//-------------------------
#include "globalVariables.h"
#include "transform.h"
#include "graphics.h"
#include "AstarSearch.h"
#include "LPAstar.h"
#include "gridworld.h"
#include "Dstar.h"

// colour constants
int BACKGROUND_COLOUR;
int LINE_COLOUR;

int robotWidth;
int GRIDWORLD_ROWS; //duplicated in GridWorld
int GRIDWORLD_COLS; //duplicated in GridWorld


//----------------------------
unsigned int HEURISTIC;
//~ bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;
int numberOfExpandedStates;
int MAX_MOVES;
int maxQLength;
int qLengthAfterSearch;

///////////////////////////////////////////////////////////////////////////////
LpaStar* lpa_star;
Dstar * d_star;
GridWorld grid_world;
DstarCell * sLast;
bool SHOW_MAP_DETAILS;
bool CONTROL_KEY_FLAG;

bool pathIsFound = false;
bool dstarPathFound = false;
int dStarCalls = 0;
bool initialPlanning = false;
bool initialPlanning2 = false;
///////////////////////////////////////////////////////////////////////////////
vector<LpaStarCell *> getLPAPath();
char getUnknownCellType(LpaStarCell *item);
bool traversePathAndDetectChanges(vector<LpaStarCell *> currentPath);
//--------------------------------------------------------------
//copy maze (from LPA*) to map (of GridWorld)
void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa){
	
	std::cout << "LPA STAR Copy maze to map" << std::endl;
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			gWorld.map[i][j].type = lpa->maze[i][j].type;
		   gWorld.map[i][j].h = lpa->maze[i][j].h;
			gWorld.map[i][j].g = lpa->maze[i][j].g;
			gWorld.map[i][j].rhs = lpa->maze[i][j].rhs;
			gWorld.map[i][j].row = lpa->maze[i][j].y;
			gWorld.map[i][j].col = lpa->maze[i][j].x;
			
			for(int k=0; k < 2; k++){
			  gWorld.map[i][j].key[k] = lpa->maze[i][j].key[k];			  
			}
			
			
		}
	}
	gWorld.map[lpa->start->y][lpa->start->x].h = lpa->start->h;
	gWorld.map[lpa->start->y][lpa->start->x].g = lpa->start->g;
	gWorld.map[lpa->start->y][lpa->start->x].rhs = lpa->start->rhs;
	gWorld.map[lpa->start->y][lpa->start->x].row = lpa->start->y;
	gWorld.map[lpa->start->y][lpa->start->x].col = lpa->start->x;
	for(int k=0; k < 2; k++){
			  gWorld.map[lpa->start->y][lpa->start->x].key[k] = lpa->start->key[k];			  
	}
	
	
	gWorld.map[lpa->goal->y][lpa->goal->x].h = lpa->goal->h;
	gWorld.map[lpa->goal->y][lpa->goal->x].g = lpa->goal->g;
	gWorld.map[lpa->goal->y][lpa->goal->x].rhs = lpa->goal->rhs;
	gWorld.map[lpa->goal->y][lpa->goal->x].row = lpa->goal->y;
	gWorld.map[lpa->goal->y][lpa->goal->x].col = lpa->goal->x;
	for(int k=0; k < 2; k++){
			  gWorld.map[lpa->goal->y][lpa->goal->x].key[k] = lpa->goal->key[k];			  
	}
	
}

//--------------------------------------------------------------
//copy map (of GridWorld)to maze (of LPA*)
void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa){
	std::cout << "LPA STAR Copy display map to maze" << std::endl;
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			lpa->maze[i][j].type = gWorld.map[i][j].type;
			lpa->maze[i][j].x = gWorld.map[i][j].col;
			lpa->maze[i][j].y = gWorld.map[i][j].row;
			
		   //lpa->maze[i][j].g = gWorld.map[i][j].g;
			//lpa->maze[i][j].rhs = gWorld.map[i][j].rhs;
		}
	}
	
	vertex startV = gWorld.getStartVertex();
	vertex goalV = gWorld.getGoalVertex();
	
	//lpa->start->g = gWorld.map[startV.row][startV.col].g ;
	//lpa->start->rhs = gWorld.map[startV.row][startV.col].rhs ;
	lpa->start->x = gWorld.map[startV.row][startV.col].col;
	lpa->start->y = gWorld.map[startV.row][startV.col].row;
	
	//lpa->goal->g = gWorld.map[goalV.row][goalV.col].g;
	//lpa->goal->rhs = gWorld.map[goalV.row][goalV.col].rhs;
	lpa->goal->x = gWorld.map[goalV.row][goalV.col].col;
	lpa->goal->y = gWorld.map[goalV.row][goalV.col].row;
	
}

void copyMazeToDisplayMap(GridWorld &gWorld, Dstar* d){
	// std::cout << "D STAR Copy maze to map" << std::endl;
	
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			gWorld.map[i][j].type = d->maze[i][j].type;
		    gWorld.map[i][j].h = d->maze[i][j].h;
			gWorld.map[i][j].g = d->maze[i][j].g;
			gWorld.map[i][j].rhs = d->maze[i][j].rhs;
			gWorld.map[i][j].row = d->maze[i][j].y;
			gWorld.map[i][j].col = d->maze[i][j].x;
			
			for(int k=0; k < 2; k++){
			  gWorld.map[i][j].key[k] = d->maze[i][j].key[k];			  
			}
			
			
		}
	}
	gWorld.map[d->start->y][d->start->x].h = d->start->h;
	gWorld.map[d->start->y][d->start->x].g = d->start->g;
	gWorld.map[d->start->y][d->start->x].rhs = d->start->rhs;
	gWorld.map[d->start->y][d->start->x].row = d->start->y;
	gWorld.map[d->start->y][d->start->x].col = d->start->x;
	for(int k=0; k < 2; k++){
			  gWorld.map[d->start->y][d->start->x].key[k] = d->start->key[k];			  
	}
	
	// std::cout << "Goal x,y " << d->goal->x << "," << d->goal->y << std::endl;
	gWorld.map[d->goal->y][d->goal->x].h = d->goal->h;
	gWorld.map[d->goal->y][d->goal->x].g = d->goal->g;
	gWorld.map[d->goal->y][d->goal->x].rhs = d->goal->rhs;
	gWorld.map[d->goal->y][d->goal->x].row = d->goal->y;
	gWorld.map[d->goal->y][d->goal->x].col = d->goal->x;
	for(int k=0; k < 2; k++){
			  gWorld.map[d->goal->y][d->goal->x].key[k] = d->goal->key[k];			  
	}
	
}

//--------------------------------------------------------------
//copy map (of GridWorld)to maze (of LPA*)
void copyDisplayMapToMaze(GridWorld &gWorld, Dstar* d){
	std::cout << "D STAR Copy display map to maze" << std::endl;
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			d->maze[i][j].type = gWorld.map[i][j].type;
			d->maze[i][j].x = gWorld.map[i][j].col;
			d->maze[i][j].y = gWorld.map[i][j].row;
			
		   //d->maze[i][j].g = gWorld.map[i][j].g;
			//d->maze[i][j].rhs = gWorld.map[i][j].rhs;
		}
	}
	
	vertex startV = gWorld.getStartVertex();
	vertex goalV = gWorld.getGoalVertex();
	
	//d->start->g = gWorld.map[startV.row][startV.col].g ;
	//d->start->rhs = gWorld.map[startV.row][startV.col].rhs ;
	d->start->x = gWorld.map[startV.row][startV.col].col;
	d->start->y = gWorld.map[startV.row][startV.col].row;
	
	//d->goal->g = gWorld.map[goalV.row][goalV.col].g;
	//d->goal->rhs = gWorld.map[goalV.row][goalV.col].rhs;
	d->goal->x = gWorld.map[goalV.row][goalV.col].col;
	d->goal->y = gWorld.map[goalV.row][goalV.col].row;
	
}

///////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES


///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////




void drawInformationPanel(int x, int y, char* info){
   ///////////////////////////////////////////////////////////////////////////////////////////
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,CENTER_TEXT);
	setcolor(YELLOW);
	outtextxy(x ,y, info);
	///////////////////////////////////////////////////////////////////////////////////////////
}

int getKey(){
	
	if(GetAsyncKeyState(VK_UP) < 0) { //UP ARROW
		return 2000;
	} 
	 
	if(GetAsyncKeyState(VK_DOWN) < 0) { //DOWN ARROW
		return 2001;
	}
	
    if(GetAsyncKeyState(VK_F4) < 0) { 
       SHOW_MAP_DETAILS=false;
		 return 104;
    } 
  
    if(GetAsyncKeyState(VK_F5) < 0) {
        SHOW_MAP_DETAILS=true;
		  return 105;
    }
	 
	 if(GetAsyncKeyState(VK_F6) < 0) {
        //execute A* with strict expanded list
		  return 106;
    }
	 if(GetAsyncKeyState(VK_F7) < 0) {
        //execute LPA*
		  return 107;
    }
	 if(GetAsyncKeyState(VK_F8) < 0) {
        //execute D*Lite
		  return 108;
    }
	 
	 //copy display map to algorithm's maze
	 if(GetAsyncKeyState(VK_F9) < 0) {
	 	if(CONTROL_KEY_FLAG){
	 		return 109;
	 	}
		  
    }
	 
	 //copy algorithm's maze to display map
	 if(GetAsyncKeyState(VK_F10) < 0) {
		  return 110;
    }

    if(GetAsyncKeyState(VK_F12) < 0) {
		  if(CONTROL_KEY_FLAG){
    	  	   return 19;
    	  }
    }

    if(GetAsyncKeyState(0x54) < 0) { //T-key (Test)
		  if(CONTROL_KEY_FLAG){
    	  	   return 19;
    	  }
    }
	 	 
	 if(GetAsyncKeyState(0x53) < 0) { //S-key (start cell)
		  return 6;
    }
	 
	 if(GetAsyncKeyState(0x58) < 0) { //X-key (goal cell)
		  return 7;
    }
	 
	 if(GetAsyncKeyState(0x42) < 0) { //B-key (block cell)
	 	  // cout << "block cell." << endl;
		  return 1;
    }
	 
	 if(GetAsyncKeyState(0x47) < 0) {  //G-key
		  return 9;
    }
	 
	 if(GetAsyncKeyState(0x48) < 0) {  //H-key
		  return 10;
    }
	 
	 if(GetAsyncKeyState(0x4B) < 0) {  //K-key
		  return 11;
    }
	 
	 if(GetAsyncKeyState(0x55) < 0) { //U-key (Unblock cell)
		  // cout << "unblock cell." << endl;
		  return 12;
    }
	 
	 if(GetAsyncKeyState(0x50) < 0) { //P-key (position of cells)
		  return 14;
    }
	 
	 if(GetAsyncKeyState(0x43) < 0) { //C-key (connections of cells)
		  return 15;
    }
	 
	 if(GetAsyncKeyState(0x4D) < 0) { //M-key (entire map connections)
		  return 16;
    }

    if(GetAsyncKeyState(VK_CONTROL) < 0) { //Ctrl key
   	 CONTROL_KEY_FLAG=true;
   	 
       return 111;
    }

    if(GetAsyncKeyState(VK_SPACE) < 0) { //Initial planning
		 if(CONTROL_KEY_FLAG){
		   cout << "Initial planning..." << endl; 
    	   return 100;	
		 }		 
    	  
    }

    if(GetAsyncKeyState(VK_RETURN) < 0) { //Re-planning
		 if(CONTROL_KEY_FLAG){
		   cout << "Re-planning..." << endl; 
    	   return 200;	
		 }		 
    	  
    }

    if(GetAsyncKeyState(0x53) & 0x8000) { //Ctrl + F12-key (save map)
    
    	  if(CONTROL_KEY_FLAG){
    	  	   return 19;
    	  }
		   
    }

    if(GetAsyncKeyState(0x59) < 0) { //Y-key (set as Type '9', Unknown to be blocked cell)
    	  cout << "unknown to be blocked cell (Type 9)." << endl;
		  return 20;
    }
    if(GetAsyncKeyState(0x5A) < 0) { //Z-key (set as Type '8', Unknown to be traversable cell)
    	  cout << "unknown to be traversable cell (Type 8)." << endl;
		  return 21;
    }
	 
	 
	 return 0;
 }
 
void runSimulation(char *fileName){
	WorldBoundaryType worldBoundary; //duplicated in GridWorld
    DevBoundaryType deviceBoundary; //duplicated in GridWorld
	bool ANIMATE_MOUSE_FLAG=false;
	bool validCellSelected=false;
	static BOOL page=false;
	int mX, mY;
	float worldX, worldY;
	worldX=0.0f;
	worldY=0.0f;
	
	int action=-1;
	//-----------------------
	CellPosition p;
	int rowSelected, colSelected;
	//-----------------------
   rowSelected=-1;
	colSelected=-1;
	
	int mouseRadius=1;
		
	srand(time(NULL));  // Seed the random number generator
			
	//Initialise the world boundaries
    grid_world.initSystemOfCoordinates();
	grid_world.loadMapAndDisplay(fileName);
	grid_world.initialiseMapConnections();
	
	//----------------------------------------------------------------
	//LPA*
	lpa_star = new LpaStar(grid_world.getGridWorldRows(), grid_world.getGridWorldCols());
	d_star = new Dstar(grid_world.getGridWorldRows(), grid_world.getGridWorldCols());
	vertex start = grid_world.getStartVertex();
	vertex goal = grid_world.getGoalVertex();
	
	cout << "(start.col = " << start.col << ", start.row = " << start.row << ")" << endl;
	cout << "(goal.col = " << goal.col << ", goal.row = " << goal.row << ")" << endl;
	
	lpa_star->initialise(start.col, start.row, goal.col, goal.row);
	d_star->initialise(start.col, start.row, goal.col, goal.row);
	//lpa_star->copyMazeToDisplayMap(grid_world);
	//copyMazeToDisplayMap(grid_world, lpa_star);
	copyDisplayMapToMaze(grid_world, lpa_star);
	copyDisplayMapToMaze(grid_world, d_star);
	//----------------------------------------------------------------
		
	worldBoundary = grid_world.getWorldBoundary();
	deviceBoundary = grid_world.getDeviceBoundary();
	GRIDWORLD_ROWS = grid_world.getGridWorldRows();
	GRIDWORLD_COLS = grid_world.getGridWorldCols();
	
	bool displayPath = true;
	bool initialPlanningCompleted = false;
	bool DstarDisplayPathPrint = true;
	//setvisualpage(page);
	// keep running the program until the ESC key is pressed   
	while((GetAsyncKeyState(VK_ESCAPE)) == 0 ) {
			 setactivepage(page);
			 cleardevice();
	
		    action = getKey(); 
		
		    if(SHOW_MAP_DETAILS) 
				 grid_world.displayMapWithDetails();
			 else
			    grid_world.displayMap();
			// getch();
			 
			 switch(action){
				case 1: //Block selected cell
				 		if( rowSelected != -1 && colSelected != -1){
							grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '1');
							grid_world.initialiseMapConnections(); 
							
							rowSelected=-1;
							colSelected=-1;
						}
						action = -1;
						break;
				
				case 105: 
					   grid_world.displayMapWithKeyDetails();
						break;
				
				case 106: 
					  
				
						break;
				
				case 107: 
					  
				
						break;
				
				case 108: 
					  
				
						break;
				
				case 15:
					 
					   if( rowSelected != -1 && colSelected != -1){
							grid_world.displayVertexConnections(colSelected-1, rowSelected-1);
						   //cout << "display connections" << endl;
						   rowSelected=-1;
						   colSelected=-1;
					   } else {
							cout << "invalid new START vertex, please select a new START vertex first." << endl;
							break;
						}
						//--------------------------------------------
					   action = -1;
					    break;
						
				case 16:
					 
					   if(grid_world.isGridMapInitialised()){
							grid_world.displayMapConnections();
						   //cout << "display connections" << endl;
						   //~ rowSelected=-1;
						   //~ colSelected=-1;
					   } else {
							cout << "map has not been initialised yet." << endl;
							break;
						}
						//--------------------------------------------
					   action = -1;
					    break;		
				
				case 6: //set cell as new START vertex 
				   {
					   //--------------------------------------------
				      // retrieve current START vertex
				      vertex s = 	grid_world.getStartVertex();
				      if( (s.row != -1) && (s.col != -1) ){
							//set current START VERTEX to an ordinary TRAVERSABLE CELL
							grid_world.setMapTypeValue(s.row, s.col, '0'); 
							grid_world.initialiseMapConnections(); 
							//ok, proceed
						} else {
							cout << "invalid START vertex" << endl;
							break;
						}
				      //--------------------------------------------
						//set selected cell as the NEW START VERTEX
					   if( rowSelected != -1 && colSelected != -1){
						   grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '6');
						   s.row = rowSelected-1;
							s.col = colSelected-1;
							grid_world.setStartVertex(s);
							
						   rowSelected=-1;
						   colSelected=-1;
					   } else {
							cout << "invalid new START vertex, please select a new START vertex first." << endl;
							break;
						}
						//--------------------------------------------
					   action = -1;
						break;
					}
				
				case 7: //set cell as new GOAL vertex 
				   {
					   //--------------------------------------------
				      // retrieve current GOAL vertex
				      vertex s = 	grid_world.getGoalVertex();
				      if( (s.row != -1) && (s.col != -1) ){
							//set current GOAL VERTEX to an ordinary TRAVERSABLE CELL
							grid_world.setMapTypeValue(s.row, s.col, '0'); 
							
							//ok, proceed
						} else {
							cout << "invalid GOAL vertex" << endl;
							action = -1;
							break;
						}
				      //--------------------------------------------
						//set selected cell as the NEW GOAL VERTEX
					   if( rowSelected != -1 && colSelected != -1){
						   grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '7');
						   s.row = rowSelected-1;
							s.col = colSelected-1;
							grid_world.setGoalVertex(s);
							grid_world.initialiseMapConnections(); 
							
						   rowSelected=-1;
						   colSelected=-1;
					   } else {
							cout << "invalid new GOAL vertex, please select a new GOAL vertex first." << endl;
							action = -1;
							break;
						}
						//--------------------------------------------
					   action = -1;
						break;
					}
							
            case 22:	//Test			 
                  if(CONTROL_KEY_FLAG)	{
	                  CONTROL_KEY_FLAG=false;
	                  copyDisplayMapToMaze(grid_world, lpa_star);
					      cout << "copied display map to algorithm's maze" << endl;					      
                  }
					   action = -1;	
				      break;
				
				case 100:	//Initial planning
                  if(CONTROL_KEY_FLAG)	{
						CONTROL_KEY_FLAG=false;

						auto start = chrono::high_resolution_clock::now();
						lpa_star->replanning = false;

						copyDisplayMapToMaze(grid_world, lpa_star);
						lpa_star->computeShortestPath();
						copyMazeToDisplayMap(grid_world, lpa_star);
						if (lpa_star->getGoal()->rhs == INF) {
							std::cout << "Compute Shortest path was done with no solution found" << std::endl;
							pathIsFound = false;
						}
						else {
							vector<LpaStarCell *> pathToGoal = getLPAPath();
							pathIsFound = true;
							initialPlanningCompleted = true;
						}
					      //To do:	      
					      //  you need to add more statements here to allow for initial planning
					      //

						auto end = chrono::high_resolution_clock::now();
						auto diff = chrono::duration<double, milli>(end - start);

						cout << endl;
						cout << "Max Q length:" << lpa_star->maxQLength << endl;
						cout << "Path length:" << getLPAPath().size() << endl;
						cout << "State Expantions:" << lpa_star->stateExpansions << endl;
						cout << "Vertex Accesses:" << lpa_star->accessVertex << endl;
						cout << "Running Time for Initial Search: " << diff.count() << " msec." << endl;
		
                  }
					   action = -1;	
				      break;
				
            	case 200:	//Replanning ctrl return
						if(CONTROL_KEY_FLAG)	{
	                    	CONTROL_KEY_FLAG=false;

							lpa_star->maxQLength = 0;
							lpa_star->stateExpansions = 0;
							lpa_star->accessVertex = 0;
							auto start = chrono::high_resolution_clock::now();

							vector<LpaStarCell *> pathToGoal;
					    	// Compute shortest path should have already been called so we need to run lines 20,21,22,23 before our forever loop.
							copyDisplayMapToMaze(grid_world, lpa_star);
							if (!initialPlanningCompleted){
								lpa_star->replanning = false;
								std::cout << "Replanning called without initial planning being done. We will do it for you now." << std::endl;
								lpa_star->computeShortestPath();
								copyMazeToDisplayMap(grid_world, lpa_star);
								if (lpa_star->getGoal()->rhs == INF) {
									std::cout << "Compute Shortest path was done with no solution found" << std::endl;
									break;
									pathIsFound = false;
							    } else {
									initialPlanningCompleted = true;
									std::cout << "Automatic initial planning done" << std::endl;
								}
							}
							if (initialPlanningCompleted){
								std::cout << "Commencing replanning" << std::endl;
								lpa_star->replanning = true;
								// Compute shortest path should have already been called so we need to run lines 20,21,22,23 before our forever loop.
								vector<LpaStarCell *> pathToGoal = getLPAPath();
								bool solving = true;
								bool solutionNotFound = false;
								if (!traversePathAndDetectChanges(pathToGoal)) std::cout << "No changes were found on the way to the goal. Algorithm complete" << std::endl;
								else {
									while (solving){
										lpa_star->computeShortestPath();
										if (lpa_star->getGoal()->rhs == INF) {
											std::cout << "Compute Shortest path was done with no solution found" << std::endl;
											pathIsFound = false;
											solutionNotFound = true;
											break;
							    		}
										vector<LpaStarCell *> pathToGoal = getLPAPath();
										solving = traversePathAndDetectChanges(pathToGoal);
									}
								}
								// while (1){

								// }
								if (solutionNotFound) pathIsFound = false;
								else pathIsFound = true;
								copyMazeToDisplayMap(grid_world, lpa_star);

								auto end = chrono::high_resolution_clock::now();
								auto diff = chrono::duration<double, milli>(end - start);

								cout << endl;
								cout << "Max Q length:" << lpa_star->maxQLength << endl;
								cout << "Path length:" << getLPAPath().size() << endl;
								cout << "State Expantions:" << lpa_star->stateExpansions << endl;
								cout << "Vertex Accesses:" << lpa_star->accessVertex << endl;
								cout << "Running Time for Replanning: " << diff.count() << " msec." << endl;
							}
						}
					   action = -1;	
				      break; 
				case 2000:	//Up arrow once for initial planning, once for second planning
					if(CONTROL_KEY_FLAG)	{
						CONTROL_KEY_FLAG=false;
						copyDisplayMapToMaze(grid_world, d_star);
						// Do initial planning
						if (dStarCalls == 0){
							std::cout << "--- D* initial planning ---" << std::endl;	
							auto start = chrono::high_resolution_clock::now();				      
							sLast = d_star->start;
							// Initialise already called
							d_star->computeShortestPath();
							if (d_star->start->g == INF){
								std::cout << "D* Lite found no solution" << std::endl;
								return;
							}
							// ---------------------------- Printout information for results ----------------------------
								auto end = chrono::high_resolution_clock::now();
								auto diff = chrono::duration<double, milli>(end - start);
								d_star->computedPathLength =  d_star->getComputedPathLength();
								std::cout << "Path length                        : " << d_star->computedPathLength << std::endl;
								std::cout << "MaxQLength:                        : " << d_star->maxQLength         << std::endl;
								std::cout << "State Expantions                   : " << d_star->numStateExpansions << std::endl;
								std::cout << "Vertex Accesses                    : " << d_star->vertexAccesses 	   << std::endl;
								std::cout << "Running time for initial Replanning: " << diff.count() << " msec."   << std::endl;

								// Reset
								d_star->maxQLength         = 0;
								d_star->numStateExpansions = 0;
								d_star->vertexAccesses     = 0;
								d_star->computedPathLength = 0; 
							// ------------------------------------------------------------------------------------------
							copyMazeToDisplayMap(grid_world, d_star);
							initialPlanning = true;
							dStarCalls++;
						}
						else if (dStarCalls >= 1){
							initialPlanning = false;
							std::cout << "---------- D* replanning -----------" << std::endl;
							auto start = chrono::high_resolution_clock::now();	
							while (!( (d_star->start->x == d_star->goal->x) && (d_star->start->y == d_star->goal->y))){ //24
								if (d_star->start->g == INF){ //25
									std::cout << "D* Lite found no solution" << std::endl;
									return;
								}
								
								sLast = d_star->start; // 21	
								d_star->pathLength++;
								sLast->type = '0';
								d_star->start = d_star->getMinGPlusC(d_star->start); // 26 && 27
								d_star->start ->type ='6';


								if (d_star->detectChanges(d_star->start)) { // 28
									// std::cout << "A change was detected with dstarcalls at: " << dStarCalls << std::endl;
									d_star->km = d_star->km + d_star->getHeuristic(sLast, d_star->start, HEURISTIC); // 30
									sLast = d_star->start; // 31
									for (int i = 0; i < DIRECTIONS; i++){
										if (sLast->move[i]->type == '8'){
											sLast->move[i]->type = '0';
											for (int k = 0; k < DIRECTIONS; k++){
												if (sLast->move[k] != NULL) d_star->updateVertex(sLast->move[i]->move[k]);
												// if (sLast->move[k] != NULL) d_star->updateVertex(sLast->move[i]);
											}
										}
										if (sLast->move[i]->type == '9'){
											sLast->move[i]->type = '1';
											for (int k = 0; k < DIRECTIONS; k++){
												if (sLast->move[k] != NULL) d_star->updateVertex(sLast->move[i]->move[k]);
												// if (sLast->move[k] != NULL) d_star->updateVertex(sLast->move[i]);
											}
										}
									}
									if (dStarCalls == 2) {std::cout << "Break check" << std::endl;
									// d_star->updateVertex(sLast);
										initialPlanning2 = false;
										copyMazeToDisplayMap(grid_world, d_star);
										// break;
									}
									d_star->computeShortestPath();
									break;
								}
							}
								auto end = chrono::high_resolution_clock::now();
								auto diff = chrono::duration<double, milli>(end - start);
								std::cout << "------ D* replanning complete ------" << std::endl;
								// ---------------------------- Printout information for results ----------------------------
								d_star->computedPathLength =  d_star->getComputedPathLength();
								std::cout << "Path length                        : " << d_star->computedPathLength << std::endl;
								std::cout << "MaxQLength:                        : " << d_star->maxQLength         << std::endl;
								std::cout << "State Expantions                   : " << d_star->numStateExpansions << std::endl;
								std::cout << "Vertex Accesses                    : " << d_star->vertexAccesses 	   << std::endl;
								std::cout << "Running time for initial Replanning: " << diff.count() << " msec."   << std::endl;

								// Reset
								d_star->maxQLength         = 0;
								d_star->numStateExpansions = 0;
								d_star->vertexAccesses     = 0;
								d_star->computedPathLength = 0; 
							// ------------------------------------------------------------------------------------------	
							// d_star->computedPathLength =  d_star->getComputedPathLength2();
							copyMazeToDisplayMap(grid_world, d_star);
							if (dStarCalls == 1) initialPlanning2 = true;
							dStarCalls++;
						} 
						else std::cout << "D* initial and second planning have already been called, doing nothing." << std::endl;
					}
					action = -1;	
					break;
		case 2001: //Down arrow
			if (CONTROL_KEY_FLAG){
				CONTROL_KEY_FLAG = false;
				copyDisplayMapToMaze(grid_world, d_star);
				std::cout << "Running full D* Lite search" << endl;
				auto start = chrono::high_resolution_clock::now();
				//  --------------- DStar ---------------
				sLast = d_star->start; // 21
				// Initialise already called // 22
				d_star->computeShortestPath(); // 23

				while (!( (d_star->start->x == d_star->goal->x) && (d_star->start->y == d_star->goal->y))){ //24
					if (d_star->start->g == INF){ //25
						std::cout << "D* Lite found no solution" << std::endl;
						return;
					}
					
					sLast = d_star->start; // 21	
					d_star->pathLength++;
					// sLast->type = '0';
					d_star->start = d_star->getMinGPlusC(d_star->start); // 26 && 27
					d_star->start ->type ='6';


					if (d_star->detectChanges(d_star->start)) { // 28
						d_star->km = d_star->km + d_star->getHeuristic(sLast, d_star->start, HEURISTIC); // 30
						sLast = d_star->start; // 31
						for (int i = 0; i < DIRECTIONS; i++){
							if (sLast->move[i]->type == '8'){
								sLast->move[i]->type = '0';
								for (int k = 0; k < DIRECTIONS; k++){
									if (sLast->move[k] != NULL) d_star->updateVertex(sLast->move[i]->move[k]);
								}
							}
							if (sLast->move[i]->type == '9'){
								sLast->move[i]->type = '1';
								for (int k = 0; k < DIRECTIONS; k++){
									if (sLast->move[k] != NULL) d_star->updateVertex(sLast->move[i]->move[k]);
								}
							}
						}
						// d_star->updateVertex(sLast);
						d_star->computeShortestPath();
					}
				}
				auto end = chrono::high_resolution_clock::now();
				auto diff = chrono::duration<double, milli>(end - start);
				std::cout << "----------------- D STAR LITE INFO -----------------" << std::endl;
				std::cout << "Max Q length:" 	 << d_star->maxQLength 						 << endl;
				std::cout << "Path length:" 	 << d_star->pathLength 						 << endl;
				std::cout << "State Expantions:" << d_star->numStateExpansions 				 << endl;
				std::cout << "Vertex Accesses:"  << d_star->vertexAccesses 					 << endl;
				std::cout << "Running Time for Initial Search: " << diff.count() << " msec." << endl;
				std::cout << "----------------------------------------------------" << std::endl;

				//  -------------------------------------
				copyMazeToDisplayMap(grid_world, d_star);
				dstarPathFound = true;
			}
			action = -1;
			break;
				case 110:		//f10	
						lpa_star->solveLPAStar(grid_world);		
					   lpa_star->updateHValues();
					   copyMazeToDisplayMap(grid_world, lpa_star);
				      cout << "copied algorithm's maze to display map" << endl;
				      action = -1;
				      break;
				
				case 9: //display g-values only
					   grid_world.displayMapWithSelectedDetails(true, false, false, false);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
				      action = -1;
						break;
            case 10: //display h-values only
					   grid_world.displayMapWithSelectedDetails(false, false, true, false);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
				 		action = -1;
				      break;
				case 11: //display key-values only
					   lpa_star->updateAllKeyValues();
				      copyMazeToDisplayMap(grid_world, lpa_star);
					   grid_world.displayMapWithSelectedDetails(false, false, false, true);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
						action = -1;
				      break;
				
				case 12: //make cell Traversable
			 
					 if( rowSelected != -1 && colSelected != -1){
						 grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '0');
						 
						 rowSelected=-1;
						 colSelected=-1;
					 }
					 action = -1;
					 break; 
					 
				case 14: 
					   grid_world.displayMapWithPositionDetails();
						action = -1;
				      break;	 
					 
				case 19: 
					   if(CONTROL_KEY_FLAG){
                     CONTROL_KEY_FLAG = false;

					   	string targetFileName;
                     cout << "Save map to file." << endl;
					   	cout << "Enter destination filename: ";
					   	cin >> targetFileName;
                     grid_world.saveNewMap(targetFileName.c_str());
                     
					   }
					   
						action = -1;
				      break;  
				case 20: //set cell as Type '9'
			 
					 if( rowSelected != -1 && colSelected != -1){
						 grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '9');
						 
						 rowSelected=-1;
						 colSelected=-1;
					 }
					 action = -1;
					 break; 

           case 21: //set cell as Type '8'
			 
					 if( rowSelected != -1 && colSelected != -1){
						 grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '8');
						 
						 rowSelected=-1;
						 colSelected=-1;
					 }
					 action = -1;
					 break; 
		  
				  
				 
		    };
		
		//----------------------------------------------------------------------------------------------------------------

      //To do: add statements to display the computed shortest path here 
      // if(path is Found){
      // 	display path
      // }
	  
	  	if (initialPlanning) grid_world.DstarDisplayPath(d_star->start->x, d_star->start->y);
		if (initialPlanning2){
			grid_world.DstarDisplayPath(d_star->start->x, d_star->start->y);
		}
	  	if (dstarPathFound){
			// std::cout << "Displaying d*" ;
		  	if (DstarDisplayPathPrint){
				std::cout << "Displaying D* solution graphically" << std::endl;
				DstarDisplayPathPrint = false;
				std::cout << "finished displaying d*";
			}
	  	}
	  	if (pathIsFound){
			string path = grid_world.LPAdisplayPath();
			if (displayPath) {
				displayPath = false;
				// std::cout << "Path found is (read right to left): " << path << std::endl;
			}
		}

	   //----------------------------------------------------------------------------------------------------------------	  
		// Mouse handling
		//
			 if(mousedown()){
						 				
				ANIMATE_MOUSE_FLAG=true;
				 			 
				mX = mousecurrentx();
				mY = mousecurrenty();
				 
				//if the goal selected is within the playing field boundaries
				if(mX >= grid_world.getFieldX1() && mX <= grid_world.getGridMaxX() && mY >= grid_world.getFieldY1() && mY <= grid_world.getGridMaxY()){
					
					    circle(mX, mY, 3);
					    validCellSelected = true;
  	            
				} else {
					validCellSelected = false;
				}
			 } //end of mousedown()
			 //------------------------------------------------------------------------------------------------------------------
			 /////////////////////////////////////////////////////////////////////////////
			 						 
			 if(ANIMATE_MOUSE_FLAG){
					
				  //draw Cross-hair to mark Goal	    
				  setcolor(RED);
				  circle(mX, mY, 20);
				  line(mX,mY-20,mX,mY+20);
				  line(mX-20,mY,mX+20,mY);
				  //end of draw Cross-hair 
			 
				  // special effect to display concentric circles locating the target
					setcolor(YELLOW);
					
					if(mouseRadius < 40) {
						mouseRadius += 1;
					}
					circle(mX, mY, mouseRadius);
					//Sleep(50);
									
					if(mouseRadius >= 40) {
						ANIMATE_MOUSE_FLAG=false;
						mouseRadius=0;
					}
					//end of special effect
			  }

			 
			 /////////////////////////////////////////////////////////////////////////////
			  char info[80]; 
			  float wX, wY;
			  
			  wX = xWorld(worldBoundary,deviceBoundary,mX);
			  wY = yWorld(worldBoundary,deviceBoundary,mY);
			  sprintf(info,"x: %d, y: %d",mX, mY); 
			  drawInformationPanel(grid_world.getFieldX2(), grid_world.getFieldY1() + textheight("H")*2, info);
			  
			 
			  sprintf(info,"wX: %3.0f, wY: %3.0f",wX, wY); 
			  drawInformationPanel(grid_world.getFieldX2(),grid_world.getFieldY1() + textheight("H")*5, info);
			 ///////////////////////////////////////////////////////////////////////////// 
			 
			  //~ CellPosition p;
			  //~ int rowSelected, colSelected;
			  
			  if(validCellSelected) {
				  p=grid_world.getCellPosition_markCell(mX, mY);
				  rowSelected = p.row;
				  colSelected = p.col;
				  
				  sprintf(info,"row: %d, col: %d",rowSelected, colSelected); 
			      drawInformationPanel(grid_world.getFieldX2(),grid_world.getFieldY1() + textheight("H")*6, info);
				  
			  }
			  setvisualpage(page);
			  page = !page;  //switch to another page
	}
}



//main.exe ./grids/grid_lpa_journal.map m

//main.exe ./grids/grid_Dstar_journal.map m

int main(int argc, char *argv[]) {	
	srand(time(0));
	// for (int i = 0; i < 100; i ++){
	// 	for (int j = 0; j < 100; j++){
	// 		if ((i == 0) || (j == 0) || (j == 99) || (i == 99)){
	// 			std::cout << "1";
	// 			continue;
	// 		}
	// 		if ((i == 1) && (j==1)) {
	// 			std::cout << "6";
	// 			continue;
	// 		}
	// 		if ((j==98) && (i==98)){
	// 			std::cout << "7";
	// 			continue;
	// 		}
	// 		// if ((i< 20) || (i > 79) || (j< 20) || (j > 79)) {
	// 		// 	std::cout << "0";
	// 		// 	continue;
	// 		// }
	// 		int iRand = (rand() % 10);
	// 		// if (iRand == 0) {
	// 		// 	std::cout << "8";
	// 		// 	continue;
	// 		// }
	// 		if (iRand == 1) {
	// 			std::cout << "9";
	// 			continue;
	// 		}
	// 		if (iRand == 2) {
	// 			std::cout << "1";
	// 			continue;
	// 		}
	// 		std::cout << "0";
	// 	}
	// 	std::cout << std::endl;
	// }
	char gridFileName[80];
	if (argc == 3){
	    string heuristic(argv[2]);
	    std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(),::tolower);
     
		strcpy(gridFileName, argv[1]);
        
		//heuristic function selection
		if((heuristic.compare("euclidean")==0) || (heuristic.compare("e")==0)){
			HEURISTIC = EUCLIDEAN;
			cout << "Heuristic function = EUCLIDEAN" << endl;
		}
		if((heuristic.compare("manhattan")==0) || (heuristic.compare("m")==0)){
			HEURISTIC = MANHATTAN;
			cout << "Heuristic function = MANHATTAN" << endl;
		}
		if((heuristic.compare("chebyshev")==0) || (heuristic.compare("c")==0)){
			HEURISTIC = CHEBYSHEV;
			cout << "Heuristic function = CHEBYSHEV" << endl;
		}	

	} else {
		cout << "missing parameters:  gridworld heuristic" << endl;
		cout << "Example: ./main .\\grids\\grid_Dstar_journal.map m" << endl;
	}
	
	int graphDriver = 0,graphMode = 0;
 	
	//initgraph(&graphDriver, &graphMode, "", 1440, 900); // Start Window
 	// initgraph(&graphDriver, &graphMode, "", 1280, 1024); // Start Window
	
    // initgraph(&graphDriver, &graphMode, "", 1360, 768); // Start Window - LAPTOP SCREEN
	initgraph(&graphDriver, &graphMode, "", 1920, 1080); // Start Window - Full-HD
	
	BACKGROUND_COLOUR = WHITE;
	LINE_COLOUR = GREEN;
	
	GRIDWORLD_ROWS = 0; //7; //6; //duplicated in GridWorld
    GRIDWORLD_COLS = 0; //15;//13; //duplicated in GridWorld
	SHOW_MAP_DETAILS=false;
	CONTROL_KEY_FLAG=false;

    try{
		runSimulation(gridFileName);
    }
	
    catch(...){
    	cout << "Exception caught!\n";
    }
	
	cout << "----<< The End.>>----" << endl;
	
	return 0;
} 

vector<LpaStarCell *> getLPAPath(){
	vector<LpaStarCell *> output;
	LpaStarCell *currentVertex = lpa_star->getGoal();
	// [goalVertex.row][goalVertex.col];
	LpaStarCell * neighbour;
	LpaStarCell * min_neighbour;
	output.push_back(currentVertex);
	// std::cout << lpa_star->cellName(output[0]) << std::endl;
	double min_g_plus_c = INF;
	while (1){
		
		// std::cout << "Neighbours of " << lpa_star->cellName(currentVertex) << ": ";
		for (int m = 0; m < DIRECTIONS; m++){

			neighbour = currentVertex->move[m];
			// std::cout << lpa_star->cellName(neighbour);
			if (neighbour != NULL && neighbour->type != '1'){
				// cout << "unblocked h = " << neighbour->h << endl;
				// cout << "g= " << neighbour->g << endl;

				if (neighbour->g < min_g_plus_c){
					min_g_plus_c = neighbour->g;
					min_neighbour = neighbour;
				}
			}
		}
		// std::cout << std::endl;
		output.push_back(min_neighbour);
		if (min_neighbour == lpa_star->getStart()) {
			std::reverse(output.begin(), output.end());
			for (int i = 0; i<output.size(); i++) std::cout << lpa_star->cellName(output[i]) << " -> ";
			return output;
		}
		currentVertex = min_neighbour;

		
	}
}

bool traversePathAndDetectChanges(vector<LpaStarCell *> currentPath){
	bool changeDetected = false;
	bool localNodeChangeDetected = false;
	// For each node in the path to the goal
	for (int i = 0; i < currentPath.size(); i++) {
		// Go through each neighbour
		for (int j = 0; j < DIRECTIONS; j++){
			if (currentPath[i]->move[j]->type == '8'){
				currentPath[i]->move[j]->type = '0';
				changeDetected = true;
				localNodeChangeDetected = true;
			}
			if (currentPath[i]->move[j]->type == '9'){
				currentPath[i]->move[j]->type = '1';
				changeDetected = true;
				localNodeChangeDetected = true;
			}
			// Change is detected so we must update all surrounding vertexes
			if (localNodeChangeDetected){
				for (int k = 0; k<DIRECTIONS; k++) lpa_star->updateVertex(currentPath[i]->move[j]->move[k]);
				localNodeChangeDetected = false;
			}
		}
		if (changeDetected) {
			return changeDetected;
		}
	}
	return changeDetected;
}

// bool traversePathAndDetectChanges(vector<DstarCell *> currentPath){
// 	bool changeDetected = false;
// 	bool localNodeChangeDetected = false;
// 	// For each node in the path to the goal
// 	for (int i = 0; i < currentPath.size(); i++) {
// 		// Go through each neighbour
// 		for (int j = 0; j < DIRECTIONS; j++){
// 			if (currentPath[i]->move[j]->type == '8'){
// 				currentPath[i]->move[j]->type = '0';
// 				changeDetected = true;
// 				localNodeChangeDetected = true;
// 			}
// 			if (currentPath[i]->move[j]->type == '9'){
// 				currentPath[i]->move[j]->type = '1';
// 				changeDetected = true;
// 				localNodeChangeDetected = true;
// 			}
// 			// Change is detected so we must update all surrounding vertexes
// 			if (localNodeChangeDetected){
// 				for (int k = 0; k<DIRECTIONS; k++) lpa_star->updateVertex(currentPath[i]->move[j]->move[k]);
// 				localNodeChangeDetected = false;
// 			}
// 		}
// 		if (changeDetected) {
// 			return changeDetected;
// 		}
// 	}
// 	return changeDetected;
// }


char getUnknownCellType(LpaStarCell *item){
	if (item->type == '8'){
		return '0';
	}
	if (item->type == '9'){
		return '1';
	}
	else{
		return item->type;
	}
}
