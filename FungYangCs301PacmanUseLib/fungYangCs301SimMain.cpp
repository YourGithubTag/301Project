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
// Bool for when the robot has decided what the type of intersection is but is still in the intersection
bool intDecided = false;
bool intStatusPrinted = false;
// Integer for what type of intersection has been detected
// 0 - no inter, 1 - T from bottom, 2 - T from left, 3 - T from right, 4 - Left Turn, 5 - Right Turn, 6 - Dead end
int typeOfInt = 0;
// Bool value to keep track of whether the centre sensor is active
bool followingLine = true;

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

	// Not inside intersection, looking for intersection
	if ((intDetected == false) && (intDecided == false))
	{
		// Check to see if there is an intersection (either side is greater than 1) that hasn't been found
		if (((rightStrength > 1) || (leftStrength > 1)) && (centered == true))
		{
			intDetected = true;
			cout << "Intersection Detected." << endl;
		}
	}
	// Inside intersection, type NOT detected
	else if ((intDetected == true) && (intDecided == false))
	{
		// Check for a left turn
		if ((leftStrength > 10) && (rightStrength > 0) && (centered == TRUE))
		{
			cout << "Intersection discovered: Left Turn" << endl;
			typeOfInt = 4;
			intDecided = TRUE;
		}
		// Check for a right turn
		else if ((leftStrength > 0) && (rightStrength > 10) && (centered == TRUE))
		{
			cout << "Intersection discovered: Right Turn" << endl;
			typeOfInt = 5;
			intDecided = TRUE;
		}
	}
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
	
	// DEBUGGING: for printing some values to console
	if (myTimer.getTimer() > 0.5)
	{
		myTimer.resetTimer();
		cout << sensors << endl;
		cout << "Detected = " << intDetected << endl;
		cout << "Dealt with = " << intDecided << endl;
	}


	/*
	// Check for a dead end
	else if ((rightStrength == 0) && (leftStrength == 0) && (centered == FALSE))
	{
		intDetected = TRUE;
		intDecided = TRUE;
		cout << "Dead End Detected" << endl;
		//TODO: Add dead end functionality
	}
	// If an intersection has been detected but not decided upon then keep checking
	if ((intDetected == TRUE) && (intDecided == FALSE) && (intStatusPrinted == FALSE))
	{
		

		intStatusPrinted == TRUE;
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

void turnLeft() {
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
		for (int i = 0; i < halfTiltRange; i++) {
			if (virtualCarSensorStates[i] == 0) {
				float tilt = (float)i - halfTiltRange;
				tiltSum += tilt;
			}
		}
		setVirtualCarSpeed(virtualCarLinearSpeed_seed, virtualCarAngularSpeed_seed * tiltSum);
	}
	if (pathDetected == 0) {
		turnLeftAtSpeed(2);
	}
}

void turnRight() {
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

void turnLeftOnSpot(int speedInput) {
	speedInput = -speedInput;
	setVirtualCarSpeed(0, virtualCarAngularSpeed_seed * speedInput);
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

	virtualCarLinearSpeed_seed = 0.5;
	virtualCarAngularSpeed_seed = 40;
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

	dumbLineFollow();
	detectIntersection();
	statusReport();


	return 1;
}



int main(int argc, char** argv)
{
	FungGlAppMainFuction(argc, argv);

	return 0;
}