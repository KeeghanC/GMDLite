#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include <sstream>
#include "LPAstar.h"
#include "gridworld.h"
#include <algorithm>

 LpaStar::LpaStar(int rows_, int cols_){
		 rows = rows_;
	     cols = cols_;
	 
		 //Allocate memory 
		 maze.resize(rows);
		 for(int i=0; i < rows; i++){
		   maze[i].resize(cols);
		 }
}

void LpaStar::initialise(int startX, int startY, int goalX, int goalY){

	maxQLength = 0;
	stateExpansions = 0;
	accessVertex = 0;

	// Reset the priority queue
	this->U.clear();
	make_heap(U.begin(), U.end(), Comp());

	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].g = INF;
		   maze[i][j].rhs = INF;

			maze[i][j].move[0] = &maze[i-1][j-1];
			maze[i][j].move[1] = &maze[i-1][j];
			maze[i][j].move[2] = &maze[i-1][j+1];
			maze[i][j].move[3] = &maze[i][j-1];
			maze[i][j].move[4] = &maze[i][j+1];
			maze[i][j].move[5] = &maze[i+1][j-1];
			maze[i][j].move[6] = &maze[i+1][j];
			maze[i][j].move[7] = &maze[i+1][j+1];
		}
	}
	
	// start = new LpaStarCell;
	// goal = new LpaStarCell;

	start = &maze[startY][startX];
	goal = &maze[goalY][goalX];
	
	//START VERTEX
	start->g = INF;
	start->rhs = 0.0;
	start->x = startX;
	start->y = startY;
	
	//GOAL VERTEX
	goal->g = INF;
	goal->rhs = INF;
	goal->x = goalX;
	goal->y = goalY;
	//---------------------
	maze[start->y][start->x].g = start->g;
	maze[start->y][start->x].rhs = start->rhs;
	
	maze[goal->y][goal->x].g = goal->g;
	maze[goal->y][goal->x].rhs = goal->rhs;
	//---------------------

	updateHValues();
	Insert(start);
	// printPriorityQueue();
	
	//for debugging only
	//~ for(int i=0; i < rows; i++){
	   //~ for(int j=0; j < cols; j++){
		   //~ //cout << maze[i][j].g << ", ";
			//~ cout << maze[i][j].rhs << ", ";
			
		//~ }
		//~ cout << endl;
	//~ }
	
}

double LpaStar::minValue(double g_, double rhs_){
	if(g_ <= rhs_){
		return g_;
	} else {
		return rhs_;
	}	
}

int LpaStar::maxValue(int v1, int v2){
	
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

double LpaStar::calc_H(int x, int y){
	
	int diffY = abs(goal->y - y);
	int diffX = abs(goal->x - x);
	
	//maze[y][x].h = (double)maxValue(diffY, diffX);
	if (HEURISTIC == MANHATTAN)	return (double)(diffY + diffX);
	if (HEURISTIC == EUCLIDEAN) return (double)sqrt((diffY*diffY)+(diffX*diffX));
	if (HEURISTIC == CHEBYSHEV) return (double)maxValue(diffY, diffX);
	std::cout << "Unrecognised heuristic, check calc_h in LPAstar.cpp" << std::endl;
	return 0; 
}

void LpaStar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].h = calc_H(j, i);
		}
	}
	
	start->h = calc_H(start->x, start->y);
	goal->h = calc_H(goal->x, goal->y);
}

void LpaStar::calcKey(int x, int y){
	double key1, key2;
	
	key2 = minValue(maze[y][x].g, maze[y][x].rhs);
	key1 = key2 + maze[y][x].h;
}


void LpaStar::calcKey(LpaStarCell *cell){
	double key1, key2;
	
	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + cell->h;
	
	cell->key[0] = key1;
	cell->key[1] = key2;
}

void LpaStar::updateAllKeyValues(){	
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   calcKey(&maze[i][j]);
		}
	}
	
	calcKey(start);
	calcKey(goal);
}


void LpaStar::computeShortestPath(){
	bool printouts = false;
	int stepNumber = 0;
	if (printouts){
		std::cout << "------------------------" << std::endl;
		std::cout << "Step Number: " << stepNumber << std::endl;
		std::cout << "--- DeQueue ---" << std::endl;
		std::cout << std::endl;
		printPriorityQueue();
	}
	std::cout << "------------------------" << std::endl;
	stepNumber++;
	while ((topKeyLessThanGoal()) || (goal->rhs != goal->g)){
		int currentQLength = U.size();
		if (currentQLength > maxQLength){
			maxQLength = currentQLength;
		}
		if (printouts){
			std::cout << "------------------------" << std::endl;
			std::cout << "Step Number: " << stepNumber << std::endl;
		}
		LpaStarCell *deQueue = Pop();
		stateExpansions++;
		if (printouts){
			std::cout << "--- DeQueue ---" << std::endl;
			printLpaStarCell(deQueue);
		}
		if (deQueue->g > deQueue->rhs){
			deQueue->g = deQueue->rhs;
			for (LpaStarCell *cell : successors(deQueue)){
				updateVertex(cell);
			}
		}
		else {
			deQueue->g = INF;
			for (LpaStarCell *cell : successors(deQueue)){
				updateVertex(cell);
			}
		}
		if (printouts){
			printPriorityQueue();
			std::cout << "------------------------" << std::endl;
		}
		stepNumber++;
	}
	if (printouts) std::cout << "The algorithm completed after step " << stepNumber << std::endl;
}

void LpaStar::solveLPAStar(GridWorld gridworld){
	
}
LpaStarCell * LpaStar::getGoal(){
	return goal;
}
LpaStarCell * LpaStar::getStart(){
	return start;
}
void LpaStar::testPriorityQueueMethods(){
		printPriorityQueue();
	std::cout << "Adding fake data" << std::endl;
	// indexing goes [y][x]
	Insert(&maze[0][3], 2 ,2);
	Insert(&maze[1][3], 1 ,2);

	printPriorityQueue();

	std::cout << "Testing function: TopKey()" << std::endl;
	int key1 = *TopKey();
	int key2 = *(TopKey()+1);
	std::cout<< "TopKey returned: " << key1<< ", " <<key2<< std::endl;
	std::cout << "Congratulations you have solved LPA*" << std::endl;
	std::cout << "Testing function: Update(), chaning y =1, x = 3" << std::endl;
	Update(&maze[1][3], 99, 91);
	printPriorityQueue();
	std::cout << "Testing function: Pop" << std::endl;
	LpaStarCell *pop = Pop();
	printPriorityQueue();
	std::cout << "Testing function: remove" << std::endl;
	Remove(&maze[1][3]);
	printPriorityQueue();
}

double * LpaStar::TopKey(){
	static double keys[2];
	int minKey1 = INF;
	keys[0] = INF;
	keys[1] = INF;
	if (U.size() == 0) {
		// std::cout << "U size is zero" << std::endl;
		return keys;
	}
	for (int i = 0; i < U.size(); i++){
		if (U.at(i)->key[0] < minKey1){
			minKey1 = U.at(i)->key[0];
			keys[0] = minKey1;
			keys[1] = U.at(i)->key[1];
		}
	}
	// std::cout << "TopKey Called, returning: (key1, key2) = (" << keys[0] << ", " << keys[1] << ")" << std::endl;
	return keys;
}

LpaStarCell * LpaStar::Pop(){
	int pQSize = U.size();

	if (pQSize == 0){
		std::cout << "Updating with a pQSize of 0. This is an error" << std::endl;
	}

	double minKey1 = INF;
	double minKey2 = INF;
	LpaStarCell *result = U[0];

	pop_heap(U.begin(), U.end(), Comp());
	U.pop_back();
	return result;

	for (int i = 0; i < pQSize; i++){
		LpaStarCell* item = U.at(i);
		if (item->key[0] < minKey1){
			minKey1 = item->key[0];
		}
	}
	for (int i = 0; i < pQSize; i++){
		LpaStarCell* item = U.at(i);
		if ((item->key[0] == minKey1) && (item->key[1] < minKey2)){
			minKey2 = item->key[1];
		}
	}
	// std::cout << "Pop min key 1: "  << minKey1 << std::endl;
	// std::cout << "Pop min key 2: " << minKey2 << std::endl;
	int indexToDelete;
	for (int i = 0; i < pQSize; i++){
		LpaStarCell* item = U.at(i);
		if ((item->key[0] == minKey1) && (item->key[1] == minKey2)) {
			result = U.at(i);
			U.erase(U.begin() + i);
			make_heap(U.begin(), U.end(), Comp());
			return result;
		}
	}
	std::cout << "An error occured calling Pop, check minKey values" << std::endl;
	return result;
}

void LpaStar::Insert(LpaStarCell * item, int key1, int key2){
	item->key[0] = key1;
	item->key[1] = key2;
	U.push_back(item);
	push_heap(U.begin(), U.end(), Comp());
	return;
}

void LpaStar::Insert(LpaStarCell *item)
{
	accessVertex++;
	calcKey(item);
	U.push_back(item);

	push_heap(U.begin(), U.end(), Comp());

	return;
}

void LpaStar::Update(LpaStarCell * item, int key1, int key2){
	int pQSize = U.size();
	if (pQSize == 0){
		std::cout << "Updating with a pQSize of 0. This is an error" << std::endl;
	}
	LpaStarCell * current;
	for (int i = 0; i < pQSize; i++){
		current = U.at(i);
		if ((current->x == item->x) && (current->y == item->y)){
			current->key[0] = key1;
			current->key[1] = key2;
		}
	}
}
void LpaStar::Remove(LpaStarCell * item){
	int pQSize = U.size();
	if (pQSize == 0){
		std::cout << "Removing with a pQSize of 0. This is an error" << std::endl;
	}
	LpaStarCell * current;
	for (int i = 0; i < pQSize; i++){
		current = U.at(i);
		if ((current->x == item->x) && (current->y == item->y)){
			U.erase(U.begin() + i);
			return;
		}
	}
}

void LpaStar::printPriorityQueue(){
	std::cout << "--- EnQueue ---" << std::endl;
	for (int i = 0; i < U.size(); i++) printLpaStarCell(U.at(i));
	std::cout << "---------------" << std::endl;
}


void LpaStar::printLpaStarCell(LpaStarCell * currentItem){
	std::cout << cellName(currentItem) << std::endl;
	std::cout << " h   : " << currentItem->h 	   << std::endl;
	if (currentItem->g == INF) std::cout << " g   : INFINITY" << std::endl;
	else std::cout << " g   : " << currentItem->g 	   << std::endl;
	if (currentItem->rhs == INF) std::cout << " rhs   : INFINITY" << std::endl;
	else std::cout << " rhs : " << currentItem->rhs    << std::endl;
	std::cout << " key1: " << currentItem->key[0] << std::endl;
	std::cout << " key2: " << currentItem->key[1] << std::endl;
	std::cout << std::endl;
}

double LpaStar::getCost(LpaStarCell *item1, LpaStarCell *item2){
	if (item1->x == item2->x){
		return 1;//same column
	}
	if (item1->y == item2->y){
		return 1; //same row
	}
	return SQRT_2;
}

double LpaStar::minGPlusCOfPredessessors(LpaStarCell *item){
	vector<LpaStarCell*> predecessors = successors(item);
	double minGPlusC = INF;

	for (int i = 0; i < predecessors.size(); i++){
		double gPlusC = (predecessors.at(i)->g + getCost(item, predecessors[i]));
		// double gPlusC = (predecessors.at(i)->g + 1); // Change this later on to sqrt(2) when needed
		if (minGPlusC > gPlusC) minGPlusC = gPlusC;
	}
	// std::cout << "Update Vertex of item (x, y): (" << item->x-1 << ", " << item->y-1 << ")" << std::endl;  
	return minGPlusC;
}

//access surrounding nodes in order 
void LpaStar::updateVertex(LpaStarCell *item){
	if ( !( (item->x == start->x) && (item->y == start->y) ) )	item->rhs = minGPlusCOfPredessessors(item);
	if (itemInPriorityQueue(item)) Remove(item);
	if (item->g != item->rhs) Insert(item);
}

bool LpaStar::itemInPriorityQueue(LpaStarCell *item){
	for (LpaStarCell *node : U){
		if ((item->x == node->x) && (item->y == node->y)){
			return true;
			cout << "remove" << endl;
		}
	}
	return false;
}

bool LpaStar::topKeyLessThanGoal(){
	bool result;
	double TopKey1 = *TopKey();
	double TopKey2 = *(TopKey()+1);

	calcKey(goal); //calculate goal key before accessing

	if (TopKey1 < goal->key[0]) return true;
	if (TopKey1 == goal->key[0]) {
		if (TopKey2 < goal->key[1]){
			return true;
		}
	}
	// std::cout << "Topkey is bigger than goal" << std::endl;
	// std::cout << "TopKey1: " << TopKey1 << " TopKey2:" << TopKey2 << std::endl;
	// std::cout << "Goal: " << goal->key[0] << " Goal:" << goal->key[1] << std::endl;
	return false;

}

vector<LpaStarCell *> LpaStar::successors(LpaStarCell *item){
	vector<LpaStarCell *> succ;

	int currX = item->x;
	int currY = item->y;

	for (int j = -1; j < 2; j++){
		for (int i = -1; i < 2; i++){
			if ((i == 0) && (j == 0)) continue;
			
			if ((maze[currY + j][currX + i].type == '0') || (maze[currY + j][currX + i].type == '6') || (maze[currY + j][currX + i].type == '7') || (maze[currY + j][currX + i].type == '9')){
				// cout << "unblocked cell found at = " << (currY + j-1) << "," << (currX + i-1) << endl;
				//here we have found an unblocked node so can add to priority queue
				//with each addition to the priority rhs and key values are calculated

				succ.push_back(&maze[currY + j][currX + i]);
			}
			if (replanning)
			{
				if (maze[currY + j][currX + i].type == '8')
				{
					succ.push_back(&maze[currY + j][currX + i]);
				}
			}
		}
	}
	return succ;
}

char LpaStar::letterY(LpaStarCell *item){
	// int y = item->y - 1;
	// char letters[6] = {'A', 'B', 'C' ,'D','E','F'};
	// if (y>5) return static_cast<char>(y);
	// return letters[y];
}

string LpaStar::cellName(LpaStarCell * item){
	string result = "";
	result += to_string(item->x);
	std::stringstream temp;
    temp<<(item->y);
	result += temp.str();
	return result;
}