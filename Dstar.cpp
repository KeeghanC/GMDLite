#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include <sstream>
#include "Dstar.h"
#include "gridworld.h"

 Dstar::Dstar(int rows_, int cols_){
		 rows = rows_;
	     cols = cols_;
	 
		 //Allocate memory 
		 maze.resize(rows);
		 for(int i=0; i < rows; i++){
		   maze[i].resize(cols);
		 }
}

void Dstar::initialise(int startX, int startY, int goalX, int goalY){
	// 02 Reset the priority queue
	this->U.clear();
	maxQLength = 0;
    pathLength = 0;
    vertexAccesses = 0;
    numStateExpansions = 0;
	computedPathLength = 0;
	// 03
	km = 0.0;

	// 04
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].g = INF;
		   maze[i][j].rhs = INF;
		   maze[i][j].y = i;
		   maze[i][j].x = j;

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
	
	// start = new DstarCell;
	// goal = new DstarCell;
	std::cout << "Goal X = " << goalX << " Goal Y = " << goalY << std::endl;
	start = &maze[startY][startX];
	goal = &maze[goalY][goalX];
	
	std::cout << "Goal X = " << goal->x << " Goal Y = " << goal->y << std::endl;

	// 05
	goal->rhs = 0.0;

	updateHValues();
	
	//06
	Insert(goal);
	//Inserted
	// std::cout << "Inserted: ";
	// printDstarCell(goal);
}

double Dstar::minValue(double g_, double rhs_){
	if(g_ <= rhs_){
		return g_;
	} else {
		return rhs_;
	}	
}

int Dstar::maxValue(int v1, int v2){
	
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

double Dstar::calc_H(int x, int y){
	int diffY = abs(start->y - y);
	int diffX = abs(start->x - x);
	
	//maze[y][x].h = (double)maxValue(diffY, diffX);return (double)(diffY + diffX);
	if (HEURISTIC == MANHATTAN)	return (double)(diffY + diffX);
	if (HEURISTIC == EUCLIDEAN) return (double)sqrt((diffY*diffY)+(diffX*diffX));
	if (HEURISTIC == CHEBYSHEV) return (double)maxValue(diffY, diffX);
	std::cout << "Unrecognised heuristic, check calc_h in Dstar.cpp" << std::endl;
	return 0; 
}

void Dstar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].h = calc_H(j, i);
		}
	}
}

void Dstar::calcKey(int x, int y){
	double key1, key2;
	
	key2 = minValue(maze[y][x].g, maze[y][x].rhs);
	key1 = key2 + maze[y][x].h + km; //Dstar add km
}


void Dstar::calcKey(DstarCell *cell){
	double key1, key2;
	
	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + getHeuristic(cell, start, HEURISTIC) + km; //Dstar add km
	
	cell->key[0] = key1;
	cell->key[1] = key2;
}

void Dstar::updateAllKeyValues(){	
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   calcKey(&maze[i][j]);
		}
	}
	
	calcKey(start);
	calcKey(goal);
}
double Dstar::getHeuristic(DstarCell * sLast, DstarCell * start, unsigned int heuristic){
	int diffY =	abs(sLast->y - start->y);
	int diffX = abs(sLast->x - start->x);
	if (heuristic == MANHATTAN) return (double)(diffY + diffX);
	if (HEURISTIC == EUCLIDEAN) return (double)sqrt((diffY*diffY)+(diffX*diffX));
	else if (heuristic == CHEBYSHEV) return (double)maxValue(diffY, diffX);
	else std::cout << "Error getting heurisic, can't find heuristic type" << std::endl;
	return 0.0f;
}
bool Dstar::detectChanges(DstarCell * item){
	for (int i = 0; i < DIRECTIONS; i++){
		if (item->move[i]->type == '8') return true;
		if (item->move[i]->type == '9') return true;
	}
	return false;
}

int Dstar::getComputedPathLength(){
	int pathLength = 0;
	DstarCell *current = start;
	while (!( (current->x == goal->x) && (current->y == goal->y))){
		current = getMinGPlusC(current);
		// current->type = '6';
		pathLength++;
	}
	return pathLength;
}
int Dstar::getComputedPathLength2(){
	int pathLength = 0;
	DstarCell *current = start;
	while (!( (current->x == goal->x) && (current->y == goal->y))){
		current = getMinGPlusC(current);
		// current->type = '6';
		pathLength++;
	}
	return pathLength;
}
void Dstar::computeShortestPath(){
	bool printouts = false;
	computedPathLength = 0;
	int stepNumber = 0;
	if (printouts){
		std::cout << "------------------------" << std::endl;
		std::cout << "Step Number: " << stepNumber << std::endl;
		std::cout << "--- DeQueue ---" << std::endl;
		std::cout << std::endl;
		printPriorityQueue();
		std::cout << "------------------------" << std::endl;
	}
	stepNumber++;
	while ((topKeyLessThan(start)) || (start->rhs != start->g)){ //Dstar compare to start
		if (printouts){
			std::cout << "------------------------" << std::endl;
			std::cout << "Step Number: " << stepNumber << std::endl;
		}
		//Dstar---------
		vector<double> topkeys = TopKey();
	 	double key1old = topkeys.at(0);
		double key2old = topkeys.at(1);

		DstarCell *deQueue = Pop();
		// deQueue->type = '6';
		numStateExpansions++;
		vertexAccesses++;
		if (printouts){
		std::cout << "--- DeQueue ---" << std::endl;
		printDstarCell(deQueue);
		}


		if(oldTopKeyLessThan(key1old, key2old, deQueue)){
			Insert(deQueue);
		}
		else if (deQueue->g > deQueue->rhs){
			deQueue->g = deQueue->rhs;
			for (DstarCell *cell : successors(deQueue)){
				updateVertex(cell);
			}
		}
		else {
			deQueue->g = INF;
			for (DstarCell *cell : successors(deQueue)){
				updateVertex(cell);
			}
			updateVertex(deQueue);
		}
		if (printouts){
			printPriorityQueue();
			std::cout << "------------------------" << std::endl;
		}
		if (printouts) std::cout << "Kold: (" << key1old << ", " << key2old << ")" << std::endl;
		stepNumber++;
	}
	if (printouts) std::cout << "The algorithm completed after step " << stepNumber << std::endl;
}

void Dstar::solveDstar(GridWorld gridworld){
	
}
DstarCell * Dstar::getGoal(){
	return goal;
}
DstarCell * Dstar::getStart(){
	return start;
}
void Dstar::testPriorityQueueMethods(){
	// 	printPriorityQueue();
	// std::cout << "Adding fake data" << std::endl;
	// // indexing goes [y][x]
	// Insert(&maze[0][3], 2 ,2);
	// Insert(&maze[1][3], 1 ,2);

	// printPriorityQueue();

	// std::cout << "Testing function: TopKey()" << std::endl;
	// int key1 = *TopKey();
	// int key2 = *(TopKey()+1);
	// std::cout<< "TopKey returned: " << key1<< ", " <<key2<< std::endl;
	// std::cout << "Congratulations you have solved LPA*" << std::endl;
	// std::cout << "Testing function: Update(), chaning y =1, x = 3" << std::endl;
	// Update(&maze[1][3], 99, 91);
	// printPriorityQueue();
	// std::cout << "Testing function: Pop" << std::endl;
	// DstarCell *pop = Pop();
	// printPriorityQueue();
	// std::cout << "Testing function: remove" << std::endl;
	// Remove(&maze[1][3]);
	// printPriorityQueue();
}

vector<double> Dstar::TopKey(){
	int pQSize = U.size();
	vector<double> keys(2);
	keys[0] = INF;
	keys[1] = INF;
	if (U.size() == 0) {
		// std::cout << "U size is zero, but topkey called, could be an error" << std::endl;
		return keys;
	}
	keys[0] = U.front()->key[0];
	keys[1] = U.front()->key[1];
	return keys;

	double minKey1 = INF;
	double minKey2 = INF;
	keys[0] = INF;
	keys[1] = INF;
	for (int i = 0; i < pQSize; i++){
		DstarCell* item = U.at(i);
		if (item->key[0] < minKey1){
			minKey1 = item->key[0];
		}
	}
	for (int i = 0; i < pQSize; i++){
		DstarCell* item = U.at(i);
		if ((item->key[0] == minKey1) && (item->key[1] < minKey2)){
			minKey2 = item->key[1];
		}
	}
	keys[0] = minKey1;
	keys[1] = minKey2;
	// std::cout << "TopKey Called, returning: (key1, key2) = (" << keys[0] << ", " << keys[1] << ")" << std::endl;
	return keys;
}

DstarCell * Dstar::Pop(){
	int pQSize = U.size();

	if (pQSize == 0){
		std::cout << "Updating with a pQSize of 0. This is an error" << std::endl;
	}

	double minKey1 = INF;
	double minKey2 = INF;

	DstarCell * result;

	for (int i = 0; i < pQSize; i++){
		DstarCell* item = U.at(i);
		if (item->key[0] < minKey1){
			minKey1 = item->key[0];
		}
	}
	for (int i = 0; i < pQSize; i++){
		DstarCell* item = U.at(i);
		if ((item->key[0] == minKey1) && (item->key[1] < minKey2)){
			minKey2 = item->key[1];
		}
	}
	// std::cout << "Pop min key 1: "  << minKey1 << std::endl;
	// std::cout << "Pop min key 2: " << minKey2 << std::endl;
	int indexToDelete;
	for (int i = 0; i < pQSize; i++){
		DstarCell* item = U.at(i);
		if ((item->key[0] == minKey1) && (item->key[1] == minKey2)) {
			result = U.at(i);
			U.erase(U.begin() + i);
			return result;
		}
	}
	std::cout << "An error occured calling Pop, check minKey values";// << std::endl;
	std::cout << "  with a queue size of: " << pQSize << std::endl;
	// exit(02);
	return result;
}

void Dstar::Insert(DstarCell * item, int key1, int key2){
	item->key[0] = key1;
	item->key[1] = key2;
	U.push_back(item);
	return;
}

void Dstar::Insert(DstarCell *item)
{
	calcKey(item);
	U.push_back(item);	
	if (maxQLength < U.size()) maxQLength = U.size();
	vertexAccesses++;
	return;
}

void Dstar::Update(DstarCell * item, int key1, int key2){
	int pQSize = U.size();
	if (pQSize == 0){
		std::cout << "Updating with a pQSize of 0. This is an error" << std::endl;
	}
	DstarCell * current;
	for (int i = 0; i < pQSize; i++){
		current = U.at(i);
		if ((current->x == item->x) && (current->y == item->y)){
			current->key[0] = key1;
			current->key[1] = key2;
		}
	}
}
void Dstar::Remove(DstarCell * item){
	int pQSize = U.size();
	if (pQSize == 0){
		std::cout << "Removing with a pQSize of 0. This is an error" << std::endl;
	}
	vertexAccesses++;
	DstarCell * current;
	for (int i = 0; i < pQSize; i++){
		current = U.at(i);
		if ((current->x == item->x) && (current->y == item->y)){
			U.erase(U.begin() + i);
			return;
		}
	}
}

void Dstar::printPriorityQueue(){
	std::cout << "--- EnQueue ---" << std::endl;
	for (int i = 0; i < U.size(); i++) printDstarCell(U.at(i));
	std::cout << "---------------" << std::endl;
}


void Dstar::printDstarCell(DstarCell * currentItem){
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

double Dstar::getCost(DstarCell *item1, DstarCell *item2){
	if (item1->x == item2->x){
		return 1;//same column
	}
	if (item1->y == item2->y){
		return 1; //same row
	}
	return SQRT_2;
	// return 1;
}
string Dstar::getPath(){
	string output = "";
	DstarCell * current;
	current = start;
	while (!( (current->x == goal->x) && (current->y == goal->y))){
		output += cellName(current) + " -> ";
		current = getMinGPlusC(current);
	}
	output += cellName(current);
	return output;
}
DstarCell * Dstar::getMinGPlusC(DstarCell *item){
	// vector<DstarCell*> succ = knownSuccessors(item);
	vector<DstarCell*> succ = successors(item);
	double minGPlusC = INF;

	DstarCell *minCell; 

	for (int i = 0; i < succ.size(); i++){
		double gPlusC = (succ.at(i)->g + getCost(item, succ[i]));
		// double gPlusC = (succ.at(i)->g + 1); // Change this later on to sqrt(2) when needed
		if (minGPlusC > gPlusC) {
			minGPlusC = gPlusC;
			minCell = succ[i];
		}
	}
	// std::cout << "Update Vertex of item (x, y): (" << item->x-1 << ", " << item->y-1 << ")" << std::endl;  
	return minCell;
}

double Dstar::minGPlusCOfPredessessors(DstarCell *item){
	vector<DstarCell*> predecessors = successors(item);
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
void Dstar::updateVertex(DstarCell *item){ //Dstar change start to goal
	if ( !( (item->x == goal->x) && (item->y == goal->y) ) )	item->rhs = minGPlusCOfPredessessors(item);
	if (itemInPriorityQueue(item)) Remove(item);
	if (item->g != item->rhs) Insert(item);
}

bool Dstar::itemInPriorityQueue(DstarCell *item){
	for (DstarCell *node : U){
		if ((item->x == node->x) && (item->y == node->y)){
			return true;
			cout << "remove" << endl;
		}
	}
	return false;
}

bool Dstar::topKeyLessThan(DstarCell *item){
	bool result;
	vector<double> topkeys = TopKey();
	double TopKey1 =  topkeys.at(0);
	double TopKey2 =  topkeys.at(1);

	// std::cout << "Topkeys: " << TopKey1 << " " << TopKey2 << std::endl;
	calcKey(item); //calculate goal key before accessing

	if (TopKey1 < item->key[0]) return true;
	if (TopKey1 == item->key[0]) {
		if (TopKey2 < item->key[1]){
			return true;
		}
	}
	// std::cout << "Topkey is bigger than goal" << std::endl;
	// std::cout << "TopKey1: " << TopKey1 << " TopKey2:" << TopKey2 << std::endl;
	// std::cout << "Goal: " << goal->key[0] << " Goal:" << goal->key[1] << std::endl;
	return false;

}

bool Dstar::oldTopKeyLessThan(double key1old, double key2old, DstarCell *item){
	bool result; 

	calcKey(item); //calculate goal key before accessing

	if (key1old < item->key[0]) return true;
	if (key1old == item->key[0]) {
		if (key2old < item->key[1]){
			return true;
		}
	}
	// std::cout << "Topkey is bigger than goal" << std::endl;
	// std::cout << "TopKey1: " << TopKey1 << " TopKey2:" << TopKey2 << std::endl;
	// std::cout << "Goal: " << goal->key[0] << " Goal:" << goal->key[1] << std::endl;
	return false;

}

vector<DstarCell *> Dstar::successors(DstarCell *item){
	vector<DstarCell *> succ;

	int currX = item->x;
	int currY = item->y;

	for (int j = -1; j < 2; j++){
		for (int i = -1; i < 2; i++){
			if ((i == 0) && (j == 0)) continue;
			if (   (maze[currY + j][currX + i].type == '0')
			    || (maze[currY + j][currX + i].type == '6')
				|| (maze[currY + j][currX + i].type == '7')
				|| (maze[currY + j][currX + i].type == '9'))
				succ.push_back(&maze[currY + j][currX + i]);
		}
	}
	return succ;
}
vector<DstarCell *> Dstar::knownSuccessors(DstarCell *item){
	vector<DstarCell *> succ;

	int currX = item->x;
	int currY = item->y;

	for (int j = -1; j < 2; j++){
		for (int i = -1; i < 2; i++){
			if ((i == 0) && (j == 0)) continue;
			if (   (maze[currY + j][currX + i].type == '0')
			    || (maze[currY + j][currX + i].type == '6')
				|| (maze[currY + j][currX + i].type == '7')
				|| (maze[currY + j][currX + i].type == '0'))
				succ.push_back(&maze[currY + j][currX + i]);
		}
	}
	return succ;
}

char Dstar::letterY(DstarCell *item){
	int y = item->y - 1;
	char letters[6] = {'A', 'B', 'C' ,'D','E','F'};
	return letters[y];
}

string Dstar::cellName(DstarCell * item){
	// Enable for letter y
	char ystring = letterY(item); 
	std::string xstring = to_string(item->x);
	// std::string xstring = to_string(item->x);
	return ystring + xstring;

	// // Enable for integer y
	// string ystring = to_string(item->y);
	

	// // X
	// std::string xstring = to_string(item->x);

	// string result = "";
	// result += xstring + ",";
	// result += ystring;
	
	// return result;
}
