#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector> 
#include "globalVariables.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class LpaStar{

public:
    LpaStar(int rows, int cols); //constructor

    void initialise(int startX, int startY, int goalX, int goalY);
	 double minValue(double g_, double rhs_);
     void solveLPAStar(GridWorld gridworld);
    //double maxValue(double v1, double v2);
    int maxValue(int v1, int v2);
	 void calcKey(int x, int y);
    void calcKey(LpaStarCell *cell);
    //void calc_H(int x, int y);
    double calc_H(int x, int y);
    void updateHValues();
    void updateAllKeyValues();

    

    //void copyMazeToDisplayMap(GridWorld gWorld);
    friend void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa);
    friend void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa);

    // --- Keeghan's Priority Queue Functions ---
        // This returns the lowest key 1 with the corresponding key 2 value
    double * TopKey();
        // Deletes and Returns the element with the lowest key1
    LpaStarCell * Pop();
    
    void Insert(LpaStarCell * item, int key1, int key2);
     void Insert(LpaStarCell *item);

    void Update(LpaStarCell * item, int key1, int key2);

    void Remove(LpaStarCell * item);
        // Prints the priority Queue
    void printPriorityQueue();
    void testPriorityQueueMethods();
    void printLpaStarCell(LpaStarCell * currentItem);
    // ------------------------------------------

    void computeShortestPath();
    void updateVertex(LpaStarCell *item);
     vector<LpaStarCell *> successors(LpaStarCell *item);
    bool topKeyLessThanGoal();
    double minGPlusCOfPredessessors(LpaStarCell *item);
    bool itemInPriorityQueue(LpaStarCell *item);
    double getCost(LpaStarCell *item1, LpaStarCell *item2);

    char letterY(LpaStarCell *item);
    string cellName(LpaStarCell * item);
    LpaStarCell * getGoal();
    LpaStarCell * getStart();

    int maxQLength;
    int stateExpansions;
    int accessVertex;

    bool replanning;
    
private:

    struct Comp
    {
        bool operator()(LpaStarCell *&lhs, LpaStarCell *&rhs)
        {
            if (lhs->key[0] > rhs->key[0])
                return true;
            if ((lhs->key[0] == rhs->key[0]) && (lhs->key[1] > rhs->key[1]))
                return true;
            return false;
        }
    };
	
    vector<vector<LpaStarCell> > maze;   
    LpaStarCell l;
    vector<LpaStarCell* > U; //Priority Queue
    LpaStarCell* start;
    LpaStarCell* goal;

    int rows;
    int cols;

};

#endif
