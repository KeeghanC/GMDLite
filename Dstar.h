#ifndef __Dstar_H__
#define __Dstar_H__

#include <vector> 
#include <algorithm>
#include "globalVariables.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class Dstar{

public:
    Dstar(int rows, int cols); //constructor

    void initialise(int startX, int startY, int goalX, int goalY);
	 double minValue(double g_, double rhs_);
     void solveDstar(GridWorld gridworld);
    //double maxValue(double v1, double v2);
    int maxValue(int v1, int v2);
	 void calcKey(int x, int y);
    void calcKey(DstarCell *cell);
    //void calc_H(int x, int y);
    double calc_H(int x, int y);
    void updateHValues();
    void updateAllKeyValues();
    string getPath();
    double getHeuristic(DstarCell * sLast, DstarCell * start, unsigned int heuristic);

    

    //void copyMazeToDisplayMap(GridWorld gWorld);
    friend void copyMazeToDisplayMap(GridWorld &gWorld, Dstar* d);
    friend void copyDisplayMapToMaze(GridWorld &gWorld, Dstar* d);
    // --- Keeghan's Priority Queue Functions ---
        // This returns the lowest key 1 with the corresponding key 2 value
    vector<double> TopKey();
        // Deletes and Returns the element with the lowest key1
    DstarCell * Pop();
    
    void Insert(DstarCell * item, int key1, int key2);
     void Insert(DstarCell *item);

    void Update(DstarCell * item, int key1, int key2);

    void Remove(DstarCell * item);
        // Prints the priority Queue
    void printPriorityQueue();
    void testPriorityQueueMethods();
    void printDstarCell(DstarCell * currentItem);
    // ------------------------------------------

    void computeShortestPath();
    void updateVertex(DstarCell *item);
    vector<DstarCell *> knownSuccessors(DstarCell *item);
    vector<DstarCell *> successors(DstarCell *item);
    bool topKeyLessThan(DstarCell *item);
    double minGPlusCOfPredessessors(DstarCell *item);
    DstarCell * getMinGPlusC(DstarCell *item);
    bool itemInPriorityQueue(DstarCell *item);
    double getCost(DstarCell *item1, DstarCell *item2);
    bool oldTopKeyLessThan(double key1old, double key2old, DstarCell *item);
    bool detectChanges(DstarCell * item);
    char letterY(DstarCell *item);
    string cellName(DstarCell * item);
    DstarCell * getGoal();
    DstarCell * getStart();
    int getComputedPathLength();
    int getComputedPathLength2();
    DstarCell* start;
    DstarCell* goal;
    double km;
    // Testing numbers
    
    
    int maxQLength;
    int pathLength;
    int vertexAccesses;
    int computedPathLength;
    int numStateExpansions;
private:
	
    vector<vector<DstarCell> > maze;   
    DstarCell l;
    struct Comp {
        bool operator()(DstarCell *&lhs, DstarCell *&rhs)
        {
            if (lhs->key[0] > rhs->key[0])
                return true;
            if ((lhs->key[0] == rhs->key[0]) && (lhs->key[1] > rhs->key[1]))
                return true;
            return false;
        }
    };
    vector<DstarCell* > U; //Priority Queue
    

    int rows;
    int cols;

};

#endif
