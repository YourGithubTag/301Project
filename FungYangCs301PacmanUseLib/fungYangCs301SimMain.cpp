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

using namespace std;

//{=================================================
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//these global variables must be defined here with no modification.
float virtualCarLinearSpeed;//can get ands set
float virtualCarAngularSpeed;//can get and set
float currentCarAngle;//can get and set
float currentCarPosCoord_X, currentCarPosCoord_Y;//can get and set

int sensorPopulationAlgorithmID;//can set
float sensorSeparation;//can set
float num_sensors;//can set

int intersectionmap[15][19];

vector<int> virtualCarSensorStates; //can get

vector<ghostInfoPack> ghostInfoPackList;// can get
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//}=================================================

highPerformanceTimer myTimer;
highPerformanceTimer turnTimer;

//just a helper function
void setVirtualCarSpeed(float linearSpeed, float angularSpeed)
{
	virtualCarLinearSpeed = linearSpeed;
	virtualCarAngularSpeed = angularSpeed;
}

//The Only TWO unctions Students need to modify to add their own sensor guided
//path following control and Map path search calculations.
//{=================================================
float virtualCarLinearSpeed_seed;
float virtualCarAngularSpeed_seed;

/******************** USER DEFINED GLOBAL VARIABLES ********************/

// Bool for if the robot is currently dealing with an intersection
bool intDetected = false;
bool intStatusPrinted = false;
// Integer for what type of intersection has been detected
// 0 - no inter, 1 - T from bottom, 2 - T from left, 3 - T from right, 4 - Left Turn, 5 - Right Turn, 6 - Dead end
int typeOfInt = 0;
// Bool value to keep track of whether the centre sensor is active
bool followingLine = true;
// Target angle if the car is turning
int targetAngle = 0;
// Bool flag to check if the car is currently turning
bool turning = false;

// Variables to store the corners of the map
int topLeftRow = 99;
int topLeftCol = 99;
int topRightRow = 99;
int topRightCol = 99;
int botLeftRow = 99;
int botLeftCol = 99;
int botRightRow = 99;
int botRightCol = 99;

/******************** USER DEFINED FUNCTIONS ********************/

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

	/*
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

	// Start with top left
	int row, col;
	bool found = false;
	for (row = 0; row < 7; row++)
	{
		for (col = 0; col < 10; col++)
		{
			if (map[row][col] == 0)
			{
				topLeftRow = row;
				topLeftCol = col;
				found = true;
				break;
			}
		}
		// If no corner found then default to 6,9
		if ((row == 6) && (topLeftRow == 99) && (topLeftCol == 99))
		{
			topLeftRow = 6;
			topLeftCol = 9;
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
				topRightRow = row;
				topRightCol = col;
				found = true;
				break;
			}
		}

		// If no corner found then default to 6,10
		if ((row == 6) && (topRightRow == 99) && (topRightCol == 99))
		{
			topRightRow = 6;
			topRightCol = 10;
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
				botLeftRow = row;
				botLeftCol = col;
				found = true;
				break;
			}
		}
		// If no corner found then default to 7,9
		if ((row == 7) && (botLeftRow == 99) && (botLeftCol == 99))
		{
			botLeftRow = 7;
			botLeftCol = 9;
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
				botRightRow = row;
				botRightCol = col;
				found = true;
				break;
			}
		}

		// If no corner found then default to 7,10
		if ((row == 7) && (botRightRow == 99) && (botRightCol == 99))
		{
			botRightRow = 7;
			botRightCol = 10;
		}

		// Break out if it has been found
		if (found == true)
		{
			break;
		}
	}

	cout << "Top left is " << topLeftRow << ", " << topLeftCol << endl;
	cout << "Top right is " << topRightRow << ", " << topRightCol << endl;
	cout << "Bottom left is " << botLeftRow << ", " << botLeftCol << endl;
	cout << "Bottom right is " << botRightRow << ", " << botRightCol << endl;
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

void turnLeftAtSpeed(int speedInput) {
	speedInput = -speedInput;
	setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * speedInput);
}

void turnRightAtSpeed(int speedInput) {
	setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * speedInput);
}

void turnLeft90()
{
	currentCarAngle = 0;
	
	//setVirtualCarSpeed(virtualCarLinearSpeed_seed * 0, virtualCarAngularSpeed_seed);
}

void turnRight90() {
	bool pathDetected = 0;
	for (int j = 0; j < num_sensors; j++) {
		if (virtualCarSensorStates[j] == 0) {
			pathDetected = 1;
		}
	}

	turnTimer.resetTimer();
	while (turnTimer.getTimer() < 0.5) {
		float halfTiltRange = (num_sensors - 1.0) / 2.0;
		float tiltSum = 0.0;
		for (int i = halfTiltRange + 2; i < num_sensors; i++) {
			if (virtualCarSensorStates[i] == 0) {
				float tilt = (float)i - halfTiltRange;
				tiltSum += tilt;
			}
		}
		setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
	}
	if (pathDetected == 0) {
		turnRightAtSpeed(2);
	}
}

void turnLeft90OnSpot() {
	// Let other code know the car is currently turning
	if (turning == false)
	{
		turning = true;
		if (currentCarAngle > 270)
		{
			targetAngle = (currentCarAngle - 270);
		}
		else
		{
			targetAngle = currentCarAngle + 90;
		}
	}
	else
	{
		if ((currentCarAngle < (targetAngle + 5)) && (currentCarAngle > (targetAngle - 5)))
		{
			turning = false;
			targetAngle = 0;
		}
		else
		{
			setVirtualCarSpeed(virtualCarLinearSpeed_seed * 0, virtualCarAngularSpeed_seed);
		}
		cout << "Current car angle is " << currentCarAngle << endl;
		cout << "Target car angle is " << targetAngle << endl;
	}
}

void turnRightOnSpot(int speedInput) {
	setVirtualCarSpeed(0, virtualCarAngularSpeed_seed * speedInput);
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

	ConvertToIntersectionMap();

	for (int i = 0; i < 15; ++i)
	{
		for (int j = 0; j < 19; ++j)
		{
			std::cout << intersectionmap[i][j] << ' ';
		}
		std::cout << std::endl;
	}

	// Find corners of the given map
	findMapCorners();

	return 1;
}

int virtualCarUpdate()
{
	float linspeed = virtualCarLinearSpeed_seed;
	float angspeed = virtualCarAngularSpeed_seed;

	/*TODO
	* 1. Read sensor
	* - Can just use YungFang's code
	* - Move to another function maybe
	* - Determine which are active, e.g. -ve is left, +ve right
	* - Value determines srength, e.g. -1 is slight left turn, 3 is strong right turn
	* 2. Detect intersection
	* - E.g. are the far left/far right sensors active? Is it a T or a pure turn? No 4 intersections luckily
	* - Can we move the sensors around?
	* 3. What do we do at a dead end?
	* 4. What do we do off track?
	* 5. When do we call director and algo?
	*/

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
	
	
	
	

	return 1;
}



int main(int argc, char** argv)
{
	FungGlAppMainFuction(argc, argv);

	return 0;
}