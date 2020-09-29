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

#include "mainFungGLAppEngin.h" //a must
#include "data.h" //a must
#include "highPerformanceTimer.h"//just to include if timer function is required by user.
#include <vector>
#include <iostream>

#include "astar.h"

using namespace std;


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


/*********************** OUR GLOBAL VARIABLES & DATATYPES ***********************/

//// Custom coordinate data structure
//typedef struct {
//	int x;
//	int y;
//} Coordinate;

// Custom command data typedef
enum Command { TurnLeft, TurnRight, GoStraight, Turn180, Halt };

// Stores the algorithm output of a vector of coordinates
vector<Pair> algoOut;

// Map contains information on each node and its type
int intersectionmap[15][19];

// Inverted map for shortest path algo
int invertedMap[15][19];

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
// 0 - no inter, 1 - T from bottom, 2 - T from left, 3 - T from right, 4 - Left Turn, 5 - Right Turn, 6 - Dead end
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

//void FollowInstructions() {
//	//Assumptions about functionality of other functions are made
//	// Declaring array of instructions 
//	int startingCarAngle = 0;
//	vector<Coordinate> instructions;
//	Coordinate one;
//	Coordinate two;
//	one.x = 10;
//	one.y = 5;
//	two.x = 4;
//	two.y = 5;
//
//	instructions.push_back(one);
//	instructions.push_back(two);
//	
//	// CURRENT DIRECTION, NOTE THAT 1 = going right, 2 = going up, 3 = going left, 4 = going down
//	int direction = 0;
//	Coordinate currlocation;
//	Coordinate nextlocation;
//
//	nextlocation.x = 1;
//	nextlocation.y = 1;
//
//	direction = startingCarAngle / 90;
//	if (direction == 0) {
//		direction = 4;
//	}
//
//	// Loop through every set of coordinates (every instruction)
//	for (int i = 0; i < instructions.size(); i++) {
//		if (direction == 1) {
//			// Instructions if the first coordinate is being checked
//			while ((instructions[i].x == (instructions[i + 1].x) - 1) && (instructions[i + 1].y == instructions[i].y)) {
//
//				}
//		}
//		if (direction == 2) {
//			cout << "Hello, I am here 2" << endl;
//		}
//		if (direction == 3) {
//			cout << "Hello, I am here 3" << endl;
//		}
//		if (direction == 4) {
//			cout << "Hello, I am here 4" << endl;
//		}
//	}
//	
//}


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
	//FollowInstructions();

	// Print out the map containing intersections
	for (int i = 0; i < 15; ++i)
	{
		for (int j = 0; j < 19; ++j)
		{
			std::cout << intersectionmap[i][j] << ' ';
		}
		std::cout << std::endl;
	}

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



int main(int argc, char** argv)
{
	FungGlAppMainFuction(argc, argv);

	return 0;
}