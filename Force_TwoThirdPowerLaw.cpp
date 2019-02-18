// Force_TwoThirdPowerLaw.cpp : Defines the entry point for the console application.
//

#include "stdlib.h" // needed otherwise conflict with glut
#include <glut.h>
#include "HapticAPI.h"
#include "HapticMaster.h"
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>
#include <windows.h>
#include "Mmsystem.h" 

#define IPADDRESS "10.30.203.37"
#define  ERROR_MSG "--- ERROR:" 
#define PI 3.141592654

#define posX 0
#define posY 1
#define posZ 2

#define ERROR -1
#define WAIT_TO_START 0
#define MOVE_TO_INITIAL 1
#define SIGNAL_START 2
#define MOVE_TRAJECTORY 3
#define WRITE_DATA 4
#define SHUTDOWN 5
#define INIT_TRAJECTORY 6
#define END_TRAJECTORY 7



// TODO make sure HM velocity limit is not exceeded
int LoadFile(std::string inputFileName);
void InitOpenGl();
void createGlutWindow();
void Display();
void Reshape(int iWidth, int iHeight);
void Keyboard(unsigned char ucKey, int iX, int iY);
void TimerCB (int iTimer);
void writeData();

void SetScaleOffset(int experimentType);
void DrawDesiredLine();
void DrawCurrentLine(double currentValue, double desiredValue);

int windowSizeX, windowSizeY, windowPosX, windowPosY;
double gNear;
double gFar;
double gOrtLeft;
double gOrtRight;
double gOrtBottom;
double gOrtTop;

double screenDistance, screenWidth, screenHeight;
double physicalScreenWidth; 
double eyeX, eyeY, eyeZ; // position of the eye point
double centerX, centerY, centerZ; // position of the reference point
double upX, upY, upZ; // direction of the up vector

double scale, offset, currentValue, desiredValue, currentValueFilter[50];

long hapticMaster;
char response[400]; // must be long enough to store everything (pos, vel, force vectors)
char str_pos[100], str_vel[100], str_acc[100], str_force[100];

double inertia;
double currentPosition[3], currentVelocity[3], currentAcceleration[3], currentForce[3];
double springPosition_X[3], springPosition_Y[3], springPosition_Z[3];
double springVelocity_X[3], springVelocity_Y[3], springVelocity_Z[3];
double springDirection_X[3], springDirection_Y[3], springDirection_Z[3];
double springStiffness, springDamping, springDeadband, springMaxForce;
double forceDirectionMotion, forceNormal;

double startPosition[3], stopPosition[3];
double velInit, distInit, timeInit;

double heightHM;
int keyboardInput;

std::string subjectName;
int experimentType, trialNb, maxTrialNb, forceDirection;
std::string inputFileName; 

// Trajectory features
double beta, period, axisX, axisY;
std::vector<double> timeSampling;
std::vector<double> phi;
int currentIndex;

// record
std::queue<double> recordTime;
std::queue<double> recordPosition[3];
std::queue<double> recordVelocity[3];
std::queue<double> recordAcceleration[3];
std::queue<double> recordForce[3];


unsigned __int64 currentTimeStamp, timerFrequency;
double currentTime, startTime, circlePeriod;
const char* bipSound;
const char* demoSound;
const char* endSound;
int nbBip;
int countBip;
double dwellTime;
double reactionTime;
double initialDiplacementTime;
double maxVel;
int statehm;

void Display ()
{
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(gOrtLeft, gOrtRight, gOrtBottom, gOrtTop);
	gluLookAt(eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ);
	DrawDesiredLine();
	DrawCurrentLine(currentValue, desiredValue);
	glutSwapBuffers();
	glutPostRedisplay();
}

void DrawDesiredLine()
{
	glLineWidth(10);
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_LINES); 

	glVertex3f(0.0f, -0.5f, 0.0f);
	glVertex3f(0.0f, 0.5f, 0.0f);

	glEnd();
}

void DrawCurrentLine(double currentValue, double desiredValue)
{
	glLineWidth(10);
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_LINES); 

	glVertex3f(0.0f, -0.25f, (currentValue-desiredValue)*scale);
	glVertex3f(0.0f, 0.25f,  (currentValue-desiredValue)*scale);

	glEnd();
}

void SetScaleOffset(int experimentType)
{
	/* Experimental Types
	0 = 5 desired force in direction of motion;
	1 = -5 desired force in direction of motion;
	*/
	switch(experimentType)
	{
	case 0:
		scale = .1;
		desiredValue = 5.; //5;
		break;
	case 1:
		desiredValue = -5;
		scale = -.1;
		break;
	}
}




//---------------------------------------------------------------------
//                              M A I N
//---------------------------------------------------------------------
int main(int argc, char** argv)
{
	hapticMaster = 0;
	inertia = 2.0;

	// Spring features 
	double springStiffness = 10000;//5000; // 20000.;  // TODO gain peut eter a regler en fonction de la vitesse/etendue du mouvement pour que l'ellipse desiree soit environ effectuee
	double springDamping=  2; //2; //2*sqrt(inertia * springStiffness); //10. // Critical damping
	double springDeadband = 0.;
	double springMaxForce = 5000.;
	for (int i = 0; i<3; i++)
	{
		springPosition_X[i] = 0.;
		springPosition_Y[i] = 0.;
		springPosition_Z[i] = 0.;
		springVelocity_X[i] = 0.;
		springVelocity_Y[i] = 0.;
		springVelocity_Z[i] = 0.;
		springDirection_X[i] = 0.;
		springDirection_Y[i] = 0.;
		springDirection_Z[i] = 0.;

		startPosition[i] = 0.;
		stopPosition[i] = 0.;
	}
	springDirection_X[posX] = 1.0;
	springDirection_Y[posY] = 1.0;
	springDirection_Z[posZ] = 1.0;

	heightHM = -0.1;

	keyboardInput = 0;
	statehm = 0;
	currentValue = 0;
	// Timing

	bipSound = "Sounds\\bip.wav";
	demoSound = "Sounds\\success.wav";
	endSound  = "Sounds\\go.wav";
	nbBip = 3;
	countBip = 0;
	dwellTime = 3.;
	reactionTime = 0.8;
	maxVel = 0.005;

	// Smooth start and stop
	distInit = 0.10;
	timeInit = 1.;

	/* experiment details */
	subjectName = "DS";
	circlePeriod = 8.;
	trialNb = 1;
	maxTrialNb = 3;
	experimentType = 0;
	forceDirection = 1; // 0: radial; 1: tangential
	if (circlePeriod == 2.)
		inputFileName = "circle_2sec.csv"; // "ellipse_standard.csv", "ellipse_constant", "ellipse_exaggerated", "ellipse_reverse"
	else if (circlePeriod == 8.)
		inputFileName = "circle_8sec.csv";
	else
	{
		printf( "ERROR wrong period");
		return -1;
	}
	SetScaleOffset(experimentType);

	// record
	while(!recordTime.empty())
		recordTime.pop();
	for(int i=0; i<3; i++)
	{
		while(!recordPosition[i].empty())
			recordPosition[i].pop();
		while(!recordVelocity[i].empty())
			recordVelocity[i].pop();
		while(!recordAcceleration[i].empty())
			recordAcceleration[i].pop();
		while(!recordForce[i].empty())
			recordForce[i].pop();
	}
	
	//initialize currentValueFilter
	for(int i=0; i<50; i++){
		currentValueFilter[i] = 0;
	}

	// Initialize time
	QueryPerformanceFrequency((LARGE_INTEGER*)&timerFrequency);

	// Initialize random
	srand(time(NULL));

	// Initialize Haptic Master and Graphics
	
	hapticMaster = haDeviceOpen( IPADDRESS );

	if( hapticMaster == HARET_ERROR ) 
	{
	printf( "--- ERROR: Unable to connect to device: %s\n", IPADDRESS );
	return HARET_ERROR;
	}

	else 
	{
		InitializeDevice( hapticMaster );

		// set inertia
		if (haSendCommand(hapticMaster, "set inertia", inertia, response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master inertia");
			return HARET_ERROR;
		}
		
		// set state
		if (haSendCommand(hapticMaster, "set state home", response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master state");
			return HARET_ERROR;
		}

		Sleep(1000);

		// set state
		if (haSendCommand(hapticMaster, "set state position", response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master state");
			return HARET_ERROR;
		}

		// set friction
		if (haSendCommand(hapticMaster, "set coulombfriction", 0.00001, response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master Coulomb friction");
			return HARET_ERROR;
		}
	
		// create active spring
		if (haSendCommand(hapticMaster, "create spring spring_X", response) || strstr(response, ERROR_MSG))
 		{
 			printf("ERROR on spring_active creation");
 			return HARET_ERROR;
 		}
		else
		{
			if(	haSendCommand(hapticMaster, "set spring_X stiffness", springStiffness, response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X dampfactor", springDamping, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X deadband", springDeadband, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X maxforce", springMaxForce, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X direction", springDirection_X[posX], springDirection_X[posY], springDirection_X[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X enable", response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_active initialization");
 					return HARET_ERROR;
				}
		}

		// create passive spring
		if (haSendCommand(hapticMaster, "create spring spring_Y", response) || strstr(response, ERROR_MSG))
 		{
 			printf("ERROR on spring_passive creation");
 			return HARET_ERROR;
 		}
		else
		{
			if(	haSendCommand(hapticMaster, "set spring_Y stiffness", springStiffness, response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y dampfactor", springDamping, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y deadband", springDeadband, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y maxforce", springMaxForce, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y direction", springDirection_Y[posX], springDirection_Y[posY], springDirection_Y[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Y enable", response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_passive initialization");
 					return HARET_ERROR;
				}
		}

 		if (haSendCommand(hapticMaster, "create spring spring_Z", response) || strstr(response, ERROR_MSG)) 
 		{
 			printf("ERROR on Z spring creation");
 			return HARET_ERROR;
 		}
 		else
 		{
			if(	haSendCommand(hapticMaster, "set spring_Z stiffness", springStiffness, response) || strstr(response, ERROR_MSG) ||
				haSendCommand(hapticMaster, "set spring_Z dampfactor", springDamping, response)	|| strstr(response, ERROR_MSG) ||
				haSendCommand(hapticMaster, "set spring_Z deadband", springDeadband, response)	|| strstr(response, ERROR_MSG) ||
				haSendCommand(hapticMaster, "set spring_Z maxforce", springMaxForce, response)	|| strstr(response, ERROR_MSG) ||
				haSendCommand(hapticMaster, "set spring_Z direction", springDirection_Z[posX], springDirection_Z[posY], springDirection_Z[posZ], response) || strstr(response, ERROR_MSG) ||
				haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG) ||
				haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_Z enable", response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_Z initialization");
 					return HARET_ERROR;
				}	
		}
   }
	std::cout << "PRESS ENTER TO START" << std::endl;
	if (hapticMaster != 0 && LoadFile(inputFileName) == 0)
	{
		// OpenGL Initialization Calls
		glutInit(&argc, argv);
		InitOpenGl();
		// More OpenGL Initialization Calls
		glutReshapeFunc(Reshape);
		glutDisplayFunc(Display);
		glutKeyboardFunc(Keyboard);
		glutTimerFunc(10, TimerCB, 1);
		glutMainLoop();
	}
	return 0;
}

int LoadFile(std::string inputFileName)
{

	std::ifstream inputFile("Input/"+inputFileName, std::ios::in);
	if (inputFile)
	{
		std::string content;
		int ind;
				
		// Read period
		getline(inputFile, content);
		ind = content.find(',');
		period = std::stof(content.substr(ind+1));
		// Read beta
		getline(inputFile, content);
		ind = content.find(',');
		beta = std::stof(content.substr(ind+1));
		// Read X axis
		getline(inputFile, content);
		ind = content.find(',');
		axisX = std::stof(content.substr(ind+1));
		// Read Y axis
		getline(inputFile, content);
		ind = content.find(',');
		axisY = std::stof(content.substr(ind+1));
		// Read time and phi
		while(getline(inputFile, content))
		{
			ind = content.find(',');
			timeSampling.push_back(std::stof(content.substr(0, ind)));
			phi.push_back(std::stof(content.substr(ind+1)));
		}			
	}
	else
	{
		std::cout << "Cannot open input trajectory file" << std::endl;
		while(true){}
		return -1;
	}

	startPosition[posX] = axisX;
	startPosition[posY] = -distInit;
	startPosition[posZ] = heightHM;
	stopPosition[posX] = axisX;
	stopPosition[posY] = distInit;
	stopPosition[posZ] = heightHM;

	// compute initial velocity (at beginning of elliptical movement)
	velInit = axisY * (phi[1] - phi[0])/(timeSampling[1] - timeSampling[0]);
	return 0;
}

		

void TimerCB(int iTimer) {

	switch(statehm) 
	{ 
		
		case WAIT_TO_START:
			
			if (keyboardInput != 10)
			{
				keyboardInput = getchar();
			}
			else
			{
			statehm = MOVE_TO_INITIAL;
			std::cout << "MOVING TO INITIAL POSITION" << std::endl;
			}
			// Initialize move to initial setup
			QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
			currentTime = (1. * currentTimeStamp) / timerFrequency; 
			startTime = currentTime;		
			break;

		case MOVE_TO_INITIAL:

			// Go smoothly to initial position
			if(haSendCommand(hapticMaster, "get modelpos;", response) || strstr(response, ERROR_MSG)) 
			{
				printf("Error on reading HM position");
				statehm = ERROR;
			}
			else	
			{
				BreakResponse(str_pos, response, 1);
				ParseFloatVec(str_pos, currentPosition[posX], currentPosition[posY], currentPosition[posZ]);
				double distance = sqrt(pow(currentPosition[posX] - startPosition[posX], 2) + pow(currentPosition[posY] - startPosition[posY], 2) + pow(currentPosition[posZ] - startPosition[posZ],2));
				initialDiplacementTime = distance / maxVel;

				if (currentTime - startTime < initialDiplacementTime)
				{
					QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
					currentTime = (1. * currentTimeStamp) / timerFrequency; 
					springPosition_X[posX] = currentPosition[posX] + (startPosition[posX] - currentPosition[posX]) * (currentTime - startTime) / initialDiplacementTime;
					springPosition_Y[posY] = currentPosition[posY] + (startPosition[posY] - currentPosition[posY]) * (currentTime - startTime) / initialDiplacementTime;
					springPosition_Z[posZ] = currentPosition[posZ] + (startPosition[posZ] - currentPosition[posZ]) * (currentTime - startTime) / initialDiplacementTime;
					springVelocity_X[posX] = 0;
					springVelocity_Y[posY] = 0;
					springVelocity_Z[posZ] = 0;

					if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_X update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Y update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Z update");
		 				statehm = ERROR;
					}
					
					if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_X update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Y update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Z update");
		 				statehm = ERROR;
					}				
				}
				else
				{
					statehm = SIGNAL_START;
					startTime = currentTime;
					springPosition_X[posX] = startPosition[posX]; 
					springPosition_Y[posY] = startPosition[posY]; 
					springPosition_Z[posZ] = startPosition[posZ];

					//initialize currentValueFilter
					for(int i=0; i<50; i++){
						currentValueFilter[i] = 0;
					}

					std::cout << "SIGNALLING TRIAL TO START WITH SOUND" << std::endl;
				}
			}
			break;	

		case SIGNAL_START:
			// pause before start
			while (currentTime - startTime < dwellTime)
			{
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency; 
			}
			PlaySoundA(bipSound, NULL, SND_ASYNC);

	//		if countBip < 
	//			//		// Initialize
	////		currentIndex = 0;
	////		springPosition_active[activeAxis] = position[0];
	////		startTime = currentTime;
	////		
	////		if (trialNb == -1)
	////		{
	//		PlaySoundA(demoSound, NULL, SND_ASYNC);
	////			std::cout << "Demo" << std::endl;
	////		}
	////			
	////		// Dwell time
	////		while (currentTime - startTime < dwellTime)
	////		{
	////			QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
	////			currentTime = (1. * currentTimeStamp) / timerFrequency; 
	////		}
	////		while (countBip < nbBip)
	////		{
	////			startTime = currentTime;
	//			PlaySoundA(bipSound, NULL, SND_ASYNC);
	////			countBip++;
	////			while (currentTime - startTime < reactionTime)
	////			{
	////				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
	////				currentTime = (1. * currentTimeStamp) / timerFrequency; 
	////			}
	////		}		
			startTime = currentTime;			
			std::cout << "INITIATING TRAJECTORY" << std::endl;
			statehm = INIT_TRAJECTORY;	
			break;

		case INIT_TRAJECTORY:	
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency; 
				if (currentTime - startTime < timeInit)
				{											
					springPosition_X[posX] = startPosition[posX];
					springPosition_Y[posY] = startPosition[posY] - (4*velInit*timeInit + 10*startPosition[posY])*pow((currentTime - startTime)/timeInit,3) + (7*velInit*timeInit + 15*startPosition[posY])*pow((currentTime - startTime)/timeInit,4) - (3*velInit*timeInit + 6*startPosition[posY])*pow((currentTime - startTime)/timeInit,5);
					springPosition_Z[posZ] = startPosition[posZ];
					springVelocity_X[posX] = 0; 
					springVelocity_Y[posY] = 0;
					springVelocity_Z[posZ] = 0;
					
					if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_X update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Y update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Z update");
		 				statehm = ERROR;
					}
					
					if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_X update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Y update");
		 				statehm = ERROR;
					}
					if(	haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Z update");
		 				statehm = ERROR;
					}
				}
				else
				{
					statehm = MOVE_TRAJECTORY;
					startTime = currentTime;
					std::cout << "MOVING ARM ALONG TRAJECTORY" << std::endl;
					currentIndex = 0;
				}
			break;


		case MOVE_TRAJECTORY:
			if(currentIndex < timeSampling.size()-2) {
				// Update time
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency;

				// Get HM position, force and velocity
				if(haSendCommand(hapticMaster, "get modelpos; get modelvel; get modelacc; get measforce", response) || strstr(response, ERROR_MSG)) 
					printf("Error on reading HM position/velocity/force");
				else	
				{
					BreakResponse(str_pos, response, 1);
					ParseFloatVec(str_pos, currentPosition[posX], currentPosition[posY], currentPosition[posZ]);

					BreakResponse(str_vel, response, 2);
					ParseFloatVec(str_vel, currentVelocity[posX], currentVelocity[posY], currentVelocity[posZ]);
		
					BreakResponse(str_acc, response, 3);
					ParseFloatVec(str_acc, currentAcceleration[posX], currentAcceleration[posY], currentAcceleration[posZ]);

					BreakResponse(str_force, response, 4);
					ParseFloatVec(str_force, currentForce[posX], currentForce[posY], currentForce[posZ]);
				}


				currentValue = 0;
				for (int i = 1; i<50; i++){
					currentValue += currentValueFilter[i]/50;
					currentValueFilter[i-1] = currentValueFilter[i];
				}
				// Compute force along direction of motion
				forceDirectionMotion = 1./sqrt(axisX*axisX*sin(phi[currentIndex])*sin(phi[currentIndex]) + axisY*axisY*cos(phi[currentIndex])*cos(phi[currentIndex])) * (-currentForce[posX]*axisX*sin(phi[currentIndex]) + currentForce[posY]*axisY*cos(phi[currentIndex]));
				forceNormal = 1./sqrt(axisX*axisX*sin(phi[currentIndex])*sin(phi[currentIndex]) + axisY*axisY*cos(phi[currentIndex])*cos(phi[currentIndex])) * (currentForce[posY]*axisX*sin(phi[currentIndex]) + currentForce[posX]*axisY*cos(phi[currentIndex]));

				if (forceDirection == 0) // radial
				{
					currentValue += forceNormal/50;
					currentValueFilter[49] = forceNormal;
				}
				else // tangential
				{
					currentValue += forceDirectionMotion/50;
					currentValueFilter[49] = forceDirectionMotion;
				}

				// Store data
				recordTime.push(currentTime-startTime);
				for(int i=0; i<3; i++)
				{
					recordPosition[i].push(currentPosition[i]);
					recordVelocity[i].push(currentVelocity[i]);
					recordAcceleration[i].push(currentAcceleration[i]);
					recordForce[i].push(currentForce[i]);
				}

				// Update desired position (springs)
				while (currentIndex < timeSampling.size()-2 && timeSampling[currentIndex] < currentTime - startTime)
					currentIndex++;
				springPosition_X[posX] = axisX * cos(phi[currentIndex]);
				springPosition_Y[posY] = axisY * sin(phi[currentIndex]);	
				springVelocity_X[posX] = - axisX * sin(phi[currentIndex]) * (phi[currentIndex+1] - phi[currentIndex])/(timeSampling[currentIndex]+1 - timeSampling[currentIndex]);
				springVelocity_Y[posY] = axisY * cos(phi[currentIndex]) * (phi[currentIndex+1] - phi[currentIndex])/(timeSampling[currentIndex]+1 - timeSampling[currentIndex]);	

				if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_X update");
		 				statehm = ERROR;
					}
				if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Y update");
		 				statehm = ERROR;
					}
				if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_X update");
		 				statehm = ERROR;
					}
				if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
					{
			 			printf("ERROR on spring_Y update");
		 				statehm = ERROR;
					}						
			}
			else { 
				statehm = END_TRAJECTORY;
				startTime = currentTime;
				std::cout << "ENDING TRAJECTORY" << std::endl;
			}		
			break;

		case END_TRAJECTORY:
			QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
			currentTime = (1. * currentTimeStamp) / timerFrequency; 
			if (currentTime - startTime < timeInit)
			{											
				springPosition_X[posX] = stopPosition[posX];
				springPosition_Y[posY] = velInit*(currentTime - startTime)/timeInit + (-6*velInit*timeInit + 10*stopPosition[posY])*pow((currentTime - startTime)/timeInit,3) + (8*velInit*timeInit - 15*stopPosition[posY])*pow((currentTime - startTime)/timeInit,4) + (-3*velInit*timeInit + 6*stopPosition[posY])*pow((currentTime - startTime)/timeInit,5);
				springPosition_Z[posZ] = stopPosition[posZ];
				springVelocity_X[posX] = 0; 
				springVelocity_Y[posY] = 0;
				springVelocity_Z[posZ] = 0;

				if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
				{
			 		printf("ERROR on spring_X update");
		 			statehm = ERROR;
				}
				if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
				{
			 		printf("ERROR on spring_Y update");
		 			statehm = ERROR;
				}
				if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
				{
			 		printf("ERROR on spring_Z update");
		 			statehm = ERROR;
				}
					
				if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
				{
			 		printf("ERROR on spring_X update");
		 			statehm = ERROR;
				}
				if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
				{
			 		printf("ERROR on spring_Y update");
		 			statehm = ERROR;
				}
				if(	haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG))	
				{
			 		printf("ERROR on spring_Z update");
		 			statehm = ERROR;
				}
			}
			else
			{
				statehm = WRITE_DATA;
				std::cout << "WRITING DATA TO FILE" << std::endl;
			}
			break;

		case WRITE_DATA:
			writeData();
			if (trialNb < maxTrialNb)
			{
				trialNb++;
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency; 
				startTime = currentTime;
				statehm = MOVE_TO_INITIAL;
			}
			else
			{
				// set state to home
				if (haSendCommand(hapticMaster, "set state home", response) || strstr(response, ERROR_MSG))
				{
					printf( "ERROR can't set Haptic Master state");
				}
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency; 
				startTime = currentTime;
				statehm = SHUTDOWN;
			}
			break;

		case ERROR:
			statehm = SHUTDOWN;
			break;
		case SHUTDOWN:
			// leave time to HM to go back home
			while (currentTime - startTime < dwellTime)
			{
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency; 
			}

			haSendCommand(hapticMaster, "remove all", response);
			printf("remove all ==> %s\n", response);
		      
			haSendCommand(hapticMaster, "set state stop", response);
			printf("set state stop ==> %s\n", response);
		      
			if ( haDeviceClose(hapticMaster) ) {
				printf("---ERROR: closing device\n");
			}
			exit(0);
			break;

	}
	glutTimerFunc(0, TimerCB, 1);
}

void writeData()
{
	std::string filename;
	if (experimentType == 0)
		filename = "Output/" + subjectName + "_RobotLeads_Circle_TangentialForceAssist" + "_Period_" + std::to_string(circlePeriod) + "_Trial_" + std::to_string(trialNb) + ".csv"; 
	else
		filename = "Output/" + subjectName + "_RobotLeads_Circle_TangentialForceResist" + "_Period_" + std::to_string(circlePeriod) + "_Trial_" + std::to_string(trialNb) + ".csv"; 
	std::ofstream data_file(filename, std::ios::out);
	//std::ofstream data_file("Output/" + subjectName + "_RobotLeads_Circle_TangentialForceResist" + "_Period_" + std::to_string(circlePeriod) + "_Trial_" + std::to_string(trialNb) + ".csv", std::ios::out);
	//std::ofstream data_file("Output/" + subjectName + "_beta_" + std::to_string(beta)  + "_forceDirection_" + std::to_string(experimentType) + "_trial_" + std::to_string(trialNb) + "_" + inputFileName, std::ios::out);
	if (data_file)
	{
		data_file << "Time" << "," << "PositionX" << "," << "PositionY" << "," << "PositionZ" << "," << "VelocityX" << "," << "VelocityY" << "," << "VelocityZ" << "," << "AccelerationX" << "," << "AccelerationY" << "," << "AccelerationZ" <<"," << "ForceX" << "," << "ForceY" << "," << "ForceZ" << std::endl;
		while(!recordTime.empty())
		{
			data_file	<< recordTime.front() << "," << recordPosition[posX].front() << ","  << recordPosition[posY].front() << ","  << recordPosition[posZ].front() << "," 
						<< recordVelocity[posX].front() << "," << recordVelocity[posY].front() << "," << recordVelocity[posZ].front() << "," 
						<< recordAcceleration[posX].front() << "," << recordAcceleration[posY].front() << "," << recordAcceleration[posZ].front() << "," 
						<< recordForce[posX].front() << "," << recordForce[posY].front() << "," << recordForce[posZ].front() << "," << std::endl;

			recordTime.pop();
			for(int i=0; i<3; i++)
			{
				recordPosition[i].pop();
				recordVelocity[i].pop();
				recordAcceleration[i].pop();
				recordForce[i].pop();
			}
		}
		std::cout << "Recording Done" << std::endl;
	}
	else
	{
		std::cout << "Recording Failed" << std::endl;
	}
}
	
void InitOpenGl()
{

   glShadeModel(GL_SMOOTH);

   glLoadIdentity();
   
   GLfloat GrayLight[] = {0.75, 0.75, 0.75, 1.0};
   GLfloat LightPosition[] = {1.0, 2.0, 1.0, 0.0};
   GLfloat LightDirection[] = {0.0, 0.0, -1.0, 0.0};

   glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
   glLightfv(GL_LIGHT0, GL_AMBIENT, GrayLight);
   glLightfv(GL_LIGHT0, GL_DIFFUSE, GrayLight);
   glLightfv(GL_LIGHT0, GL_SPECULAR, GrayLight);
   
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glEnable(GL_DEPTH_TEST);
   glEnable(GL_NORMALIZE);
   
   glEnable (GL_BLEND);
   glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   
   createGlutWindow();

   glClearColor(0.0, 0.0, 0.0, 0.0);

}

void createGlutWindow()
{
	windowSizeX = 1024;
	windowSizeY = 768;
	windowPosX = 1920 + 0.5 * (1024 - windowSizeX);
	windowPosY = 0.5 * (768 - windowSizeY);
	physicalScreenWidth = 2.46;
	double ratio = (double)glutGet(GLUT_SCREEN_WIDTH)/(double)glutGet(GLUT_SCREEN_HEIGHT);
	screenDistance = 1.0; 
	screenWidth = physicalScreenWidth; 
	screenHeight = screenWidth / ratio; //m	
	gOrtLeft = -screenWidth / 2.0;
	gOrtRight = screenWidth / 2.0;
	gOrtBottom = -screenHeight / 2.0;
	gOrtTop = screenHeight / 2.0;
	// Define position of observer wrt screen 
	eyeX = screenDistance;
	eyeY = 0.;
	eyeZ = -.3 + 0.15 * screenHeight;
	centerX = 0.0;
	centerY =  eyeY;
	centerZ = eyeZ;
	upX = 0.0;
	upY = 0.0;
	upZ = 1.0;
	
	glutInitDisplayMode (GLUT_DOUBLE| GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(windowSizeX, windowSizeY);
	glutInitWindowPosition(windowPosX, windowPosY);
	glutCreateWindow("ForceExperiment");
}

void Reshape(int iWidth, int iHeight)
{
	float fAspect = (float)iWidth / iHeight;

	double ViewportWidth = ((GLsizei)glutGet(GLUT_WINDOW_WIDTH));
	double ViewportHeight = ((GLsizei)glutGet(GLUT_WINDOW_HEIGHT));

	// Set viewport
	glViewport(0, 0, (int)ViewportWidth, (int)ViewportHeight);
}

void Keyboard(unsigned char ucKey, int iX, int iY)
{

}
