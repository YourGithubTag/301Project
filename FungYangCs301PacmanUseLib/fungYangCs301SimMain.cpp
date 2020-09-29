//======================================================================
//Author: 
//Mr. Fung Yang
//Senior Technician Engineer Research and Design,
//Robotics and Control system signal processing Labs,
//Department of Electrical, Computer and Software Engineering,
//The University of Auckland.
//
//Written for teaching design course Compsys301 in ECSE department of UOA.
//
//This example program uses the pacman robot simulation library written by Mr. Fung Yang.
//
//Date 2012~2020
//=======================================================================

#include <utility>
#include <stack>
#include <set>
#include <iostream>
#include <vector>

#include "mainFungGLAppEngin.h" //a must
#include "data.h" //a must
#include "highPerformanceTimer.h"//just to include if timer function is required by user.

using namespace std;

#define ROW 15
#define COL 19


/*********************** YUNG FANG DEFINED GLOBAL VARIABLES ***********************/

// These global variables must be defined here with no modification.
float virtualCarLinearSpeed;//can get ands set
float virtualCarAngularSpeed;//can get and set
float currentCarAngle;//can get and set
float currentCarPosCoord_X, currentCarPosCoord_Y;//can get and set

int sensorPopulationAlgorithmID;//can set
float sensorSeparation;//can set
float num_sensors;//can set

vector<int> virtualCarSensorStates; //can get

vector<ghostInfoPack> ghostInfoPackList;// can get

highPerformanceTimer myTimer;
highPerformanceTimer turnTimer;


/*********************** YUNG FANG FUNCTIONS ***********************/

//just a helper function
void setVirtualCarSpeed(float linearSpeed, float angularSpeed)
{
	virtualCarLinearSpeed = linearSpeed;
	virtualCarAngularSpeed = angularSpeed;
}

//The Only TWO functions Students need to modify to add their own sensor guided
//path following control and Map path search calculations.
//{=================================================
float virtualCarLinearSpeed_seed;
float virtualCarAngularSpeed_seed;


/*********************** A* STAR ALGORITHM FUNCTION AND DATATYPE DECLARATIONS ***********************/
// Adapted from https://www.geeksforgeeks.org/a-search-algorithm/

// Creating a shortcut for int, int pair type 
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type 
typedef pair<double, pair<int, int>> pPair;

// Global list of coordinates for algoOut
vector<Pair> algoOut;

// A structure to hold the neccesary parameters 
struct cell
{
	// Row and Column index of its parent 
	// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1 
	int parent_i, parent_j;
	// f = g + h 
	double f, g, h;
};

// A Utility Function to check whether given cell (row, col) 
// is a valid cell or not. 
bool isValid(int row, int col)
{
	// Returns true if row number and column number 
	// is in range 
	return (row >= 0) && (row < ROW) &&
		(col >= 0) && (col < COL);
}

// A Utility Function to check whether the given cell is 
// blocked or not 
bool isUnBlocked(int grid[][COL], int row, int col)
{
	// Returns true if the cell is not blocked else false 
	if (grid[row][col] == 1)
		return (true);
	else
		return (false);
}

// A Utility Function to check whether destination cell has 
// been reached or not 
bool isDestination(int row, int col, Pair dest)
{
	if (row == dest.first && col == dest.second)
		return (true);
	else
		return (false);
}

// A Utility Function to calculate the 'h' heuristics. 
double calculateHValue(int row, int col, Pair dest)
{
	// Return using the distance formula 
	return ((double)sqrt((row - dest.first) * (row - dest.first)
		+ (col - dest.second) * (col - dest.second)));
}

// A Utility Function to trace the path from the source 
// to destination 
void tracePath(cell cellDetails[][COL], Pair dest)
{
	printf("\nThe Path is ");
	int row = dest.first;
	int col = dest.second;

	stack<Pair> Path;

	while (!(cellDetails[row][col].parent_i == row
		&& cellDetails[row][col].parent_j == col))
	{
		Path.push(make_pair(row, col));
		int temp_row = cellDetails[row][col].parent_i;
		int temp_col = cellDetails[row][col].parent_j;
		row = temp_row;
		col = temp_col;
	}

	Path.push(make_pair(row, col));
	while (!Path.empty())
	{
		//pair<int, int> p = Path.top();
		Pair p = Path.top();
		algoOut.push_back(p);
		Path.pop();
		printf("-> (%d,%d) ", p.first, p.second);
	}


	return;
}

// A Function to find the shortest path between 
// a given source cell to a destination cell according 
// to A* Search Algorithm 
void aStarSearch(int grid[][COL], Pair src, Pair dest)
{
	// If the source is out of range 
	if (isValid(src.first, src.second) == false)
	{
		printf("Source is invalid\n");
		return;
	}

	// If the destination is out of range 
	if (isValid(dest.first, dest.second) == false)
	{
		printf("Destination is invalid\n");
		return;
	}

	// Either the source or the destination is blocked 
	if (isUnBlocked(grid, src.first, src.second) == false ||
		isUnBlocked(grid, dest.first, dest.second) == false)
	{
		printf("Source or the destination is blocked\n");
		return;
	}

	// If the destination cell is the same as source cell 
	if (isDestination(src.first, src.second, dest) == true)
	{
		printf("We are already at the destination\n");
		return;
	}

	// Create a closed list and initialise it to false which means 
	// that no cell has been included yet 
	// This closed list is implemented as a boolean 2D array 
	bool closedList[ROW][COL];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details 
	//of that cell 
	cell cellDetails[ROW][COL];

	int i, j;

	for (i = 0; i < ROW; i++)
	{
		for (j = 0; j < COL; j++)
		{
			cellDetails[i][j].f = FLT_MAX;
			cellDetails[i][j].g = FLT_MAX;
			cellDetails[i][j].h = FLT_MAX;
			cellDetails[i][j].parent_i = -1;
			cellDetails[i][j].parent_j = -1;
		}
	}

	// Initialising the parameters of the starting node 
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
	cellDetails[i][j].parent_i = i;
	cellDetails[i][j].parent_j = j;

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implenented as a set of pair of pair.*/
	set<pPair> openList;

	// Put the starting cell on the open list and set its 
	// 'f' as 0 
	openList.insert(make_pair(0.0, make_pair(i, j)));

	// We set this boolean value as false as initially 
	// the destination is not reached. 
	bool foundDest = false;

	while (!openList.empty())
	{
		pPair p = *openList.begin();

		// Remove this vertex from the open list 
		openList.erase(openList.begin());

		// Add this vertex to the closed list 
		i = p.second.first;
		j = p.second.second;
		closedList[i][j] = true;

		/*
			Generating all the 4 successor of this cell

			Cell-->Popped Cell (i, j)
			N --> North	 (i-1, j)
			S --> South	 (i+1, j)
			E --> East	 (i, j+1)
			W --> West		 (i, j-1)
			*/

			// To store the 'g', 'h' and 'f' of the 8 successors 
		double gNew, hNew, fNew;

		//----------- 1st Successor (North) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i - 1, j) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i - 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i - 1][j].parent_i = i;
				cellDetails[i - 1][j].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i - 1][j] == false &&
				isUnBlocked(grid, i - 1, j) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i - 1, j, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i - 1][j].f == FLT_MAX ||
					cellDetails[i - 1][j].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i - 1, j)));

					// Update the details of this cell 
					cellDetails[i - 1][j].f = fNew;
					cellDetails[i - 1][j].g = gNew;
					cellDetails[i - 1][j].h = hNew;
					cellDetails[i - 1][j].parent_i = i;
					cellDetails[i - 1][j].parent_j = j;
				}
			}
		}

		//----------- 2nd Successor (South) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i + 1, j) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i + 1, j, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i + 1][j].parent_i = i;
				cellDetails[i + 1][j].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}
			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i + 1][j] == false &&
				isUnBlocked(grid, i + 1, j) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i + 1, j, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i + 1][j].f == FLT_MAX ||
					cellDetails[i + 1][j].f > fNew)
				{
					openList.insert(make_pair(fNew, make_pair(i + 1, j)));
					// Update the details of this cell 
					cellDetails[i + 1][j].f = fNew;
					cellDetails[i + 1][j].g = gNew;
					cellDetails[i + 1][j].h = hNew;
					cellDetails[i + 1][j].parent_i = i;
					cellDetails[i + 1][j].parent_j = j;
				}
			}
		}

		//----------- 3rd Successor (East) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j + 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j + 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j + 1].parent_i = i;
				cellDetails[i][j + 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j + 1] == false &&
				isUnBlocked(grid, i, j + 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j + 1, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j + 1].f == FLT_MAX ||
					cellDetails[i][j + 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i, j + 1)));

					// Update the details of this cell 
					cellDetails[i][j + 1].f = fNew;
					cellDetails[i][j + 1].g = gNew;
					cellDetails[i][j + 1].h = hNew;
					cellDetails[i][j + 1].parent_i = i;
					cellDetails[i][j + 1].parent_j = j;
				}
			}
		}

		//----------- 4th Successor (West) ------------ 

		// Only process this cell if this is a valid one 
		if (isValid(i, j - 1) == true)
		{
			// If the destination cell is the same as the 
			// current successor 
			if (isDestination(i, j - 1, dest) == true)
			{
				// Set the Parent of the destination cell 
				cellDetails[i][j - 1].parent_i = i;
				cellDetails[i][j - 1].parent_j = j;
				printf("The destination cell is found\n");
				tracePath(cellDetails, dest);
				foundDest = true;
				return;
			}

			// If the successor is already on the closed 
			// list or if it is blocked, then ignore it. 
			// Else do the following 
			else if (closedList[i][j - 1] == false &&
				isUnBlocked(grid, i, j - 1) == true)
			{
				gNew = cellDetails[i][j].g + 1.0;
				hNew = calculateHValue(i, j - 1, dest);
				fNew = gNew + hNew;

				// If it isn’t on the open list, add it to 
				// the open list. Make the current square 
				// the parent of this square. Record the 
				// f, g, and h costs of the square cell 
				//			 OR 
				// If it is on the open list already, check 
				// to see if this path to that square is better, 
				// using 'f' cost as the measure. 
				if (cellDetails[i][j - 1].f == FLT_MAX ||
					cellDetails[i][j - 1].f > fNew)
				{
					openList.insert(make_pair(fNew,
						make_pair(i, j - 1)));

					// Update the details of this cell 
					cellDetails[i][j - 1].f = fNew;
					cellDetails[i][j - 1].g = gNew;
					cellDetails[i][j - 1].h = hNew;
					cellDetails[i][j - 1].parent_i = i;
					cellDetails[i][j - 1].parent_j = j;
				}
			}
		}

	}

	// When the destination cell is not found and the open 
	// list is empty, then we conclude that we failed to 
	// reach the destiantion cell. This may happen when the 
	// there is no way to destination cell (due to blockages) 
	if (foundDest == false)
		printf("Failed to find the Destination Cell\n");

	return;
}



/*********************** OUR GLOBAL VARIABLES & DATATYPES ***********************/

//// Custom Pair data structure
//typedef struct {
//	int x;
//	int y;
//} Pair;

// Custom command data typedef
enum Command { TurnLeft, TurnRight, GoStraight, Turn180, Halt };

//List of commands for robot
vector<Command> CommandList;
int CommandListIndex;

//vector<Pair> CoordinateList;

// Stores the algorithm output of a vector of Pairs
//vector<Pair> algoOut;

// Map contains information on each node and its type
int intersectionmap[15][19];

// Inverted map for shortest path algo
int invertedMap[15][19];

//A NEW Map (15x19) recording the positions that the Car has been to
int visited[15][19];

// Keeps track of which level was requested
int level;

// Bool for if the robot is currently dealing with an intersection
bool intDetected = false;

// Bool value to keep track of whether the centre sensor is active
bool followingLine = true;

// Target angle if the car is turning
int targetAngle = 0;

// Bool flag to check if the car is currently turning
bool turning = false;

// Variables to store the corners of the map
Pair topLeft, topRight, botLeft, botRight;

// Integer for what type of intersection has been detected
// 0=no inter, 1=T from bottom, 2=T from left, 3=T from right, 4=Left Turn, 5=Right Turn, 6=Dead end
int typeOfInt = 0;


/*********************** OUR FUNCTIONS ***********************/

// Logic Functions
void detectIntersection()
{
	// Value of either 0, 1, 10, or 111 representing the bias to a side
	int rightStrength = 0;
	int leftStrength = 0;
	// Simply keeps track if the center sensor is active
	bool centered = false;

	// String to represent the sensor states
	string sensors = "0000000";

	// Check all sensors
	for (int i = 0; i < 7; i++)
	{
		// Store sensor states in a string
		if (!virtualCarSensorStates[i])
		{
			sensors[i] = '1';
		}
	}

	// Get a strength for each side of the robot
	if (sensors[0] == '1')
	{
		rightStrength += 100;
	}
	if (sensors[1] == '1')
	{
		rightStrength += 10;
	}
	if (sensors[2] == '1')
	{
		rightStrength += 1;
	}
	if (sensors[3] == '1')
	{
		centered = TRUE;
	}
	else
	{
		centered = FALSE;
	}
	if (sensors[4] == '1')
	{
		leftStrength += 1;
	}
	if (sensors[5] == '1')
	{
		leftStrength += 10;
	}
	if (sensors[6] == '1')
	{
		leftStrength += 100;
	}

	// Looking for any type of intersection
	if (intDetected == false)
	{
		// Check to see if there is an intersection (either side is greater than 1) that hasn't been found
		if (((rightStrength > 1) || (leftStrength > 1)) && (centered == true))
		{
			intDetected = true;
			cout << "Intersection Detected." << endl;
		}

		// Check for a left turn
		if ((leftStrength > 10) && (rightStrength > 0) && (centered == TRUE))
		{
			cout << "Intersection discovered: Left Turn" << endl;
			typeOfInt = 4;
		}
		// Check for a right turn
		else if ((leftStrength > 0) && (rightStrength > 10) && (centered == TRUE))
		{
			cout << "Intersection discovered: Right Turn" << endl;
			typeOfInt = 5;
		}
	}
	// If intDetected then check if we have left it
	else if (intDetected == true)
	{
		// Check if we left intersection
		if ((leftStrength == 0) && (rightStrength == 0) && (centered == TRUE))
		{
			cout << "Left intersection" << endl;
			typeOfInt = 0;
			intDetected = false;
		}
	}

	// DEBUGGING: for printing some values to console
	if (myTimer.getTimer() > 0.5)
	{
		myTimer.resetTimer();
		cout << sensors << endl;
		cout << "Detected = " << intDetected << endl;
	}

	/* OLD CODE, is able to detect a different type of intersection
	// Inside intersection, redetecting type
	else if ((intDetected == true) & (intDecided == true))
	{
		// Check if we left intersection
		if ((leftStrength == 0) && (rightStrength == 0) && (centered == TRUE))
		{
			cout << "Left intersection" << endl;
			typeOfInt = 0;
			intDetected = false;
			intDecided = false;
		}
		// Recheck for entering the bottom of a T intersection
		else if ((leftStrength == 111) && (rightStrength == 111) && (centered == TRUE))
		{
			cout << "Intersection changed: Bottom of T intersection" << endl;
			typeOfInt = 1;
		}
		// Recheck for entering the left of a T intersection
		else if ((leftStrength == 10) && (rightStrength > 10))
		{
			cout << "Intersection changed: Left of T intersection" << endl;
			typeOfInt = 1;
		}
		// Recheck for entering the right of a T intersection
		else if ((leftStrength > 10) && (rightStrength == 10))
		{
			cout << "Intersection changed: Right of T intersection" << endl;
			typeOfInt = 1;
		}
	}
	// Should never be reached
	else
	{
		cout << "You shouldn't be here." << endl;
	}
	
	


	
	// Check for a dead end
	else if ((rightStrength == 0) && (leftStrength == 0) && (centered == FALSE))
	{
		intDetected = TRUE;
		intDecided = TRUE;
		cout << "Dead End Detected" << endl;
		//TODO: Add dead end functionality
	}
	*/

	return;
}

void FollowInstructions() {
	//Assumptions about functionality of other functions are made
	// Declaring array of instructions 
	/*vector<Pair> algoOut;
	Pair one;
	Pair two;
	one.first = 10;
	one.second = 5;
	two.first = 4;
	two.second = 5;

	algoOut.push_back(one);
	algoOut.push_back(two);*/
	
	// CURRENT DIRECTION, NOTE THAT 1 = going right, 2 = going up, 3 = going left, 4 = going down
	int direction = 0;
	Pair currlocation;
	Pair nextlocation;
	Command nextcommand;
	nextcommand = GoStraight;

	nextlocation.first = 1;
	nextlocation.second = 1;

	direction = currentCarAngle / 90;
	if (direction == 0) {
		direction = 4;
	}

	// Loop through every set of Pairs (every instruction)
	for (int i = 0; i < algoOut.size() - 1; i++) {
		nextcommand = GoStraight;
		if (direction == 1) {
			while ((algoOut[i].first == ((algoOut[i + 1].first) + 1)) && (algoOut[i + 1].second == algoOut[i].second)) {
				//Add go straight commands if it is possible to turn but command says go straight
				if (intersectionmap[(algoOut[i].first)][(algoOut[i].second) - 1] != 5 || intersectionmap[(algoOut[i].first)][(algoOut[i].second) + 1] != 5) {
					CommandList.push_back(nextcommand);
				}
				i++;
			}
			// Instructions for turning right
			if ((algoOut[i].first) == (algoOut[i + 1].first) && algoOut[i].second == (algoOut[i + 1].second + 1)) {
				nextcommand = TurnRight;
				CommandList.push_back(nextcommand);
				direction = 4;
				i++;
			}
			// Instructions for turning left
			if ((algoOut[i].first) == (algoOut[i + 1].first) && algoOut[i].second == (algoOut[i + 1].second - 1)) {
				nextcommand = TurnLeft;
				CommandList.push_back(nextcommand);
				direction = 2;
				i++;
			}
			// Instructions for turning 180
			if ((algoOut[i].first == ((algoOut[i + 1].first) - 1)) && (algoOut[i + 1].second == algoOut[i].second)) {
				nextcommand = Turn180;
				CommandList.push_back(nextcommand);
				direction = 3;
				i++;
			}
		}
		else if (direction == 2) {
			while ((algoOut[i].first == algoOut[i + 1].first) && (algoOut[i].second == algoOut[i + 1].second + 1)) {
				//Add go straight commands if it is possible to turn but command says go straight
				if (intersectionmap[(algoOut[i].first - 1)][(algoOut[i].second)] != 5 || intersectionmap[(algoOut[i].first + 1)][(algoOut[i].second)] != 5) {
					CommandList.push_back(nextcommand);
				}
				i++;
			}
			// Instructions for turning right
			if ((algoOut[i].first) == (algoOut[i + 1].first + 1) && algoOut[i].second == (algoOut[i + 1].second)) {
				nextcommand = TurnRight;
				CommandList.push_back(nextcommand);
				direction = 1;
				i++;
			}
			// Instructions for turning left
			if ((algoOut[i].first) == (algoOut[i + 1].first - 1) && algoOut[i].second == (algoOut[i + 1].second)) {
				nextcommand = TurnLeft;
				CommandList.push_back(nextcommand);
				direction = 3;
				i++;
			}
			// Instructions for turning 180
			if ((algoOut[i].first == algoOut[i + 1].first) && (algoOut[i].second == algoOut[i + 1].second - 1)) {
				nextcommand = Turn180;
				CommandList.push_back(nextcommand);
				direction = 4;
				i++;
			}
		}
		else if (direction == 3) {
			while ((algoOut[i].first == ((algoOut[i + 1].first) - 1)) && (algoOut[i + 1].second == algoOut[i].second)) {
				//Add go straight commands if it is possible to turn but command says go straight
				if (intersectionmap[(algoOut[i].first)][(algoOut[i].second) - 1] != 5 || intersectionmap[(algoOut[i].first)][(algoOut[i].second) + 1] != 5) {
					CommandList.push_back(nextcommand);
				}
				i++;
			}
			// Instructions for turning right
			if ((algoOut[i].first) == (algoOut[i + 1].first) && algoOut[i].second == (algoOut[i + 1].second + 1)) {
				nextcommand = TurnRight;
				CommandList.push_back(nextcommand);
				direction = 2;
				i++;
			}
			// Instructions for turning left
			if ((algoOut[i].first) == (algoOut[i + 1].first) && algoOut[i].second == (algoOut[i + 1].second - 1)) {
				nextcommand = TurnLeft;
				CommandList.push_back(nextcommand);
				direction = 4;
				i++;
			}
			// Instructions for turning 180
			if ((algoOut[i].first == ((algoOut[i + 1].first) + 1)) && (algoOut[i + 1].second == algoOut[i].second)) {
				nextcommand = Turn180;
				CommandList.push_back(nextcommand);
				direction = 1;
				i++;
			}
		}
		else if (direction == 4) {
			while ((algoOut[i].first == algoOut[i + 1].first) && (algoOut[i + 1].second - 1 == algoOut[i].second)) {
				//Add go straight commands if it is possible to turn but command says go straight
				if (intersectionmap[(algoOut[i].first - 1)][(algoOut[i].second)] != 5 || intersectionmap[(algoOut[i].first + 1)][(algoOut[i].second)] != 5) {
					CommandList.push_back(nextcommand);
				}
				i++;
			}
			// Instructions for turning right
			if ((algoOut[i].first) == (algoOut[i + 1].first - 1) && algoOut[i].second == (algoOut[i + 1].second)) {
				nextcommand = TurnRight;
				CommandList.push_back(nextcommand);
				direction = 3;
				i++;
			}
			// Instructions for turning left
			if ((algoOut[i].first) == (algoOut[i + 1].first + 1) && algoOut[i].second == (algoOut[i + 1].second)) {
				nextcommand = TurnLeft;
				CommandList.push_back(nextcommand);
				direction = 1;
				i++;
			}
			// Instructions for turning 180
			if ((algoOut[i].first == algoOut[i + 1].first) && (algoOut[i].second == algoOut[i + 1].second + 1)) {
				nextcommand = Turn180;
				CommandList.push_back(nextcommand);
				direction = 2;
				i++;
			}
		}
	}
	nextcommand = Halt;
	CommandList.push_back(nextcommand);
		
}


// Data Functions
void ConvertToIntersectionMap() {
	// Dead ends = 1
	// Corners = 2
	// T intersections = 3
	// + intersections = 4|
	// Normal path = 0
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 19; j++) {
			int paths = 0;
			if ((i - 1) >= 0) {
				if (map[i - 1][j] == 0) {
					paths++;
				}
			}
			if ((j - 1) >= 0) {
				if (map[i][j - 1] == 0) {
					paths++;
				}
			}
			if ((i + 1) <= 14) {
				if (map[i + 1][j] == 0) {
					paths++;
				}
			}
			if ((j + 1 <= 18)) {
				if (map[i][j + 1] == 0) {
					paths++;
				}
			}
			if (map[i][j] == 1) {
				intersectionmap[i][j] = 5;
			}
			else if ((paths == 1) && map[i][j] == 0) {
				intersectionmap[i][j] = 1;
			}
			else if ((paths == 3) && map[i][j] == 0) {
				intersectionmap[i][j] = 3;
			}
			else if ((paths == 4) && map[i][j] == 0) {
				intersectionmap[i][j] = 4;
			}
			else {
				intersectionmap[i][j] = 0;
			}
		}
	}
	for (int i = 0; i < 15; i++) {
		for (int j = 0; j < 19; j++) {
			int paths = 0;
			if ((i - 1) >= 0) {
				if (map[i - 1][j] == 0) {
					if ((j - 1) >= 0) {
						if (map[i][j - 1] == 0) {
							paths++;
						}
					}
					if ((j + 1) <= 18) {
						if (map[i][j + 1] == 0) {
							paths++;
						}
					}
				}
			}
			if ((i + 1) <= 14) {
				if (map[i + 1][j] == 0) {
					if ((j - 1) >= 0) {
						if (map[i][j - 1] == 0) {
							paths++;
						}
					}
					if ((j + 1) <= 18) {
						if (map[i][j + 1] == 0) {
							paths++;
						}
					}
				}
			}
			if ((paths == 1) && map[i][j] == 0) {
				intersectionmap[i][j] = 2;
			}
		}
	}
}

void findMapCorners()
{
	// Finds the most top left, top right, bottom left, and bottom right corners of the map
	topLeft.first = 99;
	topLeft.second = 99;
	topRight.first = 99;
	topRight.second = 99;
	botLeft.first = 99;
	botLeft.second = 99;
	botRight.first = 99;
	botRight.second = 99;


	// Start with top left
	int row, col;
	bool found = false;
	for (row = 0; row < 7; row++)
	{
		for (col = 0; col < 10; col++)
		{
			if (map[row][col] == 0)
			{
				topLeft.first = row;
				topLeft.second = col;
				found = true;
				break;
			}
		}
		// If no corner found then default to 6,9
		if ((row == 6) && (topLeft.first == 99) && (topLeft.second == 99))
		{
			topLeft.first = 6;
			topLeft.second = 9;
		}

		// Break out if it has been found
		if (found == true)
		{
			break;
		}
	}

	found = false;
	// Top right
	for (row = 0; row < 7; row++)
	{
		for (col = 18; col > 9; col--)
		{
			if (map[row][col] == 0)
			{
				topRight.first = row;
				topRight.second = col;
				found = true;
				break;
			}
		}

		// If no corner found then default to 6,10
		if ((row == 6) && (topRight.first == 99) && (topRight.second == 99))
		{
			topRight.first = 6;
			topRight.second = 10;
		}

		// Break out if it has been found
		if (found == true)
		{
			break;
		}
	}

	found = false;
	// Bottom Left
	for (row = 14; row > 6; row--)
	{
		for (col = 0; col < 10; col++)
		{
			if (map[row][col] == 0)
			{
				botLeft.first = row;
				botLeft.second = col;
				found = true;
				break;
			}
		}
		// If no corner found then default to 7,9
		if ((row == 7) && (botLeft.first == 99) && (botLeft.second == 99))
		{
			botLeft.first = 7;
			botLeft.second = 9;
		}

		// Break out if it has been found
		if (found == true)
		{
			break;
		}
	}

	found = false;
	// Bottom right
	for (row = 14; row > 6; row--)
	{
		for (col = 18; col > 9; col--)
		{
			if (map[row][col] == 0)
			{
				botRight.first = row;
				botRight.second = col;
				found = true;
				break;
			}
		}

		// If no corner found then default to 7,10
		if ((row == 7) && (botRight.first == 99) && (botRight.second == 99))
		{
			botRight.first = 7;
			botRight.second = 10;
		}

		// Break out if it has been found
		if (found == true)
		{
			break;
		}
	}

	cout << "Top left is " << topLeft.first << ", " << topLeft.second << endl;
	cout << "Top right is " << topRight.first << ", " << topRight.second << endl;
	cout << "Bottom left is " << botLeft.first << ", " << botLeft.second << endl;
	cout << "Bottom right is " << botRight.first << ", " << botRight.second << endl;
	return;
}

void statusReport()
{
	// Prints a status report to the console every tick, taken from the default code provided
	//below is optional. just to provid some status report and function test result .
	//You can try to use "printf()" to reimplemet this "cout" c++ section in a c style instead.
	//{--------------------------------------------------------------	
	if (myTimer.getTimer() > 1)
	{
		myTimer.resetTimer();

		cout << "=====================================" << endl;
		cout << "current car X, Y, theta = " << currentCarPosCoord_X << " , " << currentCarPosCoord_Y << " , " << currentCarAngle << endl;
		cout << "current Cell X, Y = " << coordToCellX(currentCarPosCoord_X) << " , " << coordToCellY(currentCarPosCoord_Y) << endl;
		cout << "-----------------------------------------" << endl;
		cout << " ghost list info:" << endl;
		for (int i = 0; i < ghostInfoPackList.size(); i++)
		{
			cout << "g[" << i << "]: (" << ghostInfoPackList[i].coord_x << ", " << ghostInfoPackList[i].coord_y << "); [s=" <<
				ghostInfoPackList[i].speed << "; [d=" << ghostInfoPackList[i].direction << "]; [T=" << ghostInfoPackList[i].ghostType << "]" << endl;
		}
		cout << "-----------------------------------------" << endl;
		int randNumber = rand_nextInt(10);
		cout << " a rand number between 0 ~ 10 = " << randNumber << endl;
		randNumber = rand_nextInt(10, 20);
		cout << " a rand number between 10 ~ 20 = " << randNumber << endl;
		cout << "-----------------------------------------" << endl;
		cout << "map[0][9] = " << map[0][9] << endl;
		cout << "food_list[5][0] = " << food_list[5][0] << endl;
	}
	//}---------------------------------------------------------------
}

void invertMap()
{
	for (int i = 0; i < 15; i++)
	{
		for (int j = 0; j < 19; j++)
		{
			if (map[i][j] == 0)
			{
				invertedMap[i][j] = 1;
			}
			else
			{
				invertedMap[i][j] = 0;
			}
		}
	}
}

// Movement Functions
void dumbLineFollow()
{
	// Follows the line blindly, taken from the code given to us
	// Process sensor state information
	float halfTiltRange = (num_sensors - 1.0) / 2.0;
	float tiltSum = 0.0;
	float blackSensorCount = 0.0;
	for (int i = 0; i < num_sensors; i++)
	{
		if (virtualCarSensorStates[i] == 0)
		{
			float tilt = (float)i - halfTiltRange;
			tiltSum += tilt;
			blackSensorCount += 1.0;
		}
	}

	// Update linear and rotational speed based on sensor information
	if (blackSensorCount > 0.0)
	{
		setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
	}
	else
	{
		setVirtualCarSpeed(0.0, virtualCarAngularSpeed_seed);
	}
}

void turnLeft()
{
	setVirtualCarSpeed(virtualCarLinearSpeed_seed * .1, virtualCarAngularSpeed_seed);
}

void turnRight() {
	setVirtualCarSpeed(virtualCarLinearSpeed_seed * .1, -virtualCarAngularSpeed_seed);
}

void goStraight()
{
	setVirtualCarSpeed(virtualCarLinearSpeed_seed, 0);
}



void RobotControl() {
	Command current = CommandList.at(CommandListIndex);

	switch (current) {
		case TurnLeft:
			break;

		case TurnRight:
			break;

		case GoStraight:
			break;

		case Turn180:
			break;

			//TODO: HALT STUFF
		case Halt:
			break;

	}




}




/******************** CORE FUNCTIONS ********************/

int virtualCarInit()
{
	//sensorPopulationAlgorithmID = PLACE_SENSORS_AUTO_SEP;
	sensorPopulationAlgorithmID = PLACE_SENSORS_SEP_USER_DEFINED;
	num_sensors = 7;
	sensorSeparation = 0.08;

	virtualCarLinearSpeed_seed = 0.4;
	virtualCarAngularSpeed_seed = 50;
	currentCarPosCoord_X = 6;
	currentCarPosCoord_Y = -3;
	currentCarAngle = 90;


	// All our initialisation code is here on out

	CommandListIndex = 0;

	// Get the level from the user
	cout << "Enter desired level\n0 -> Debug Mode\n1 -> Level 1 Logic\n2 -> Level 2 Logic" << endl;
	cin >> level;

	// Find corners of the given map
	findMapCorners();

	// Find all intersections, dead ends, turns, and crosses in the given map
	ConvertToIntersectionMap();

	// Invert the given map
	invertMap();

	// Call the shortest path algorithm between the four corners of the map
	Pair src = { 1, 1 };
	Pair dest = { 13, 17 };
	aStarSearch(invertedMap, src, dest);

	// Convert algorithm output to robot instructions
	// FollowInstructions();

	// Print out the map containing intersections
	for (int i = 0; i < 15; ++i)
	{
		for (int j = 0; j < 19; ++j)
		{
			cout << intersectionmap[i][j] << ' ';
		}
		cout << endl;
	}


	for (int i = 0; i < algoOut.size(); i++)
	{
		cout << "(" << algoOut[i].first << "," << algoOut[i].second << ") -> ";
	}
	cout << endl;

	cout << "===============================================" << endl;

	return 1;
}

int virtualCarUpdate()
{
	float linspeed = virtualCarLinearSpeed_seed;
	float angspeed = virtualCarAngularSpeed_seed;

	// DEBUG
	if (level == 0)
	{
		//dumbLineFollow();
		//detectIntersection();
		statusReport();

		/*
		if (myTimer.getTimer() > 0.5)
		{
			myTimer.resetTimer();

			turnLeft90OnSpot();
		}
		*/
	}

	// Level 1
	else if (level == 1)
	{


	}

	// Level 2
	else if (level == 2)
	{

	}

	else
	{
		cout << "Invalid level number." << endl;
		return 1;
	}
	
	return 1;
}


//This function runs every tick, gets the currentCarCoord and then converts to Cell.
void ConvertToVisitedMap() 
{
	//visited map (15x19) is initialised with all Zeros
	for (int i = 0; i < 15; i++)
	{
		for (int j = 0; j < 19; j++)
		{
			visited[i][j] = 0;
		}
	}

	// Runs through every tick?????
	if (myTimer.getTimer() > 1)
	{
		myTimer.resetTimer();

		//Convert X-coord into Cell-X
		int Cell_X = coordToCellX(currentCarPosCoord_X);

		//Convert Y-coord into Cell-Y
		int Cell_Y = coordToCellY(currentCarPosCoord_Y);

		//Set visited positions to be 1
		visited[Cell_X][Cell_Y] = 1;

	}
}


int main(int argc, char** argv)
{
	FungGlAppMainFuction(argc, argv);

	return 0;
}