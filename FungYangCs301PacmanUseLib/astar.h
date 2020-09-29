#ifndef _ASTAR_H
#define _ASTAR_H 

// A C++ Program to implement A* Search Algorithm
// Adapted from https://www.geeksforgeeks.org/a-search-algorithm/

#include <utility>
#include <stack>
#include <set>
#include <iostream>
using namespace std;

#define ROW 15
#define COL 19

// Creating a shortcut for int, int pair type 
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type 
typedef pair<double, pair<int, int>> pPair;

struct cell

// A Utility Function to check whether given cell (row, col) 
// is a valid cell or not. 
bool isValid(int row, int col)

// A Utility Function to check whether the given cell is 
// blocked or not 
bool isUnBlocked(int grid[][COL], int row, int col)


// A Utility Function to check whether destination cell has 
// been reached or not 
bool isDestination(int row, int col, Pair dest)


// A Utility Function to calculate the 'h' heuristics. 
double calculateHValue(int row, int col, Pair dest)


// A Utility Function to trace the path from the source 
// to destination 
void tracePath(cell cellDetails[][COL], Pair dest)


// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
void aStarSearch(int grid[][COL], Pair src, Pair dest)


#endif