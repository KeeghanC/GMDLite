void copyMazeToDisplayMap(GridWorld &gWorld, Dstar* d){
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
