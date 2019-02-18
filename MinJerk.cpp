// MinJerk.cpp : Defines the entry point for the console application.
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
#include "Mmsystem.h" // required for PlaySound and must be placed here and not in header file (don't know why but otherwise compilation errors)

#define IPADDRESS "10.30.203.37"
#define  ERROR_MSG "--- ERROR:" 
#define PI 3.141592654

#define posX 0
#define posY 1
#define posZ 2

// TODO verifier pour tous les cas de l'experience qu'on n'est jamais en limite de vitesse par rapport aux capacites du HM

//---------------------------------------------------------------------
//                              M A I N
//---------------------------------------------------------------------
int main(int argc, char** argv)
{
	long hapticMaster = 0;
	char response[400]; // must be long enough to store everything (pos, vel, force vectors)
	char str_pos[100], str_vel[100], str_acc[100], str_force[100];
	double inertia = 2.0;
	double currentPosition[3];
	double currentVelocity[3];
	double currentAcceleration[3];
	double currentForce[3];

	// Spring features 
	double springStiffness = 20000; // 10000.;  // TODO gain peut eter a regler en fonction de la vitesse/etendue du mouvement pour que l'ellipse desiree soit environ effectuee
	double springDamping=  2; //2*sqrt(inertia * springStiffness); //10. // Critical damping
	double springDeadband = 0.;
	double springMaxForce = 5000.;
	double springPosition_active[3];
	double springPosition_passive[3];
	double springPosition_Z[3];
	double springVelocity_active[3];
	double springVelocity_passive[3];
	double springVelocity_Z[3];
	double springDirection_active[3];
	double springDirection_passive[3];
	double springDirection_Z[3];
	for (int i = 0; i<3; i++)
	{
		springPosition_active[i] = 0.;
		springPosition_passive[i] = 0.;
		springPosition_Z[i] = 0.;
		springVelocity_active[i] = 0.;
		springVelocity_passive[i] = 0.;
		springVelocity_Z[i] = 0.;
		springDirection_active[i] = 0.;
		springDirection_passive[i] = 0.;
		springDirection_Z[i] = 0.;
	}
	springDirection_Z[posZ] = 1.0;
	double heightHM = -0.12;

	// Timing
	unsigned __int64 currentTimeStamp, timerFrequency;
	double currentTime, startTime;
	const char* bipSound = "Sounds\\bip.wav";
	const char* demoSound = "Sounds\\success.wav";
	const char* endSound  = "Sounds\\go.wav";
	int nbBip = 3;
	int countBip = 0;
	double dwellTime = 3.;
	double reactionTime = 0.8;
	double initialDiplacementTime;
	double maxVel = 0.1;

	int trialNb = -1;
	int maxTrialNb = 5;
	int caseNb = 0;
	int whichCase;
	int maxNbCases = 9;
	std::vector<int> totalNbCases;
	for (int i=0; i<maxNbCases; i++)   // TODO change for real number of cases
		totalNbCases.push_back(i+9);
	int keyboardInput = 0;

	// Trajectory features
	std::string axis;
	unsigned int activeAxis;
	unsigned int passiveAxis;
	double period;
	std::string law;
	std::vector<double> timeSampling;
	std::vector<double> position;
	int currentIndex;

	// record
	std::string subjectName = "PM";
	std::queue<double> recordTime;
	std::queue<double> recordPosition[3];
	std::queue<double> recordVelocity[3];
	std::queue<double> recordAcceleration[3];
	std::queue<double> recordForce[3];
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
	
	// Initialize time
	QueryPerformanceFrequency((LARGE_INTEGER*)&timerFrequency);
	// Initialize random
	srand(time(NULL));
	

   // Open the HapticMASTER device
   hapticMaster = haDeviceOpen( IPADDRESS );

   if( hapticMaster == HARET_ERROR ) {
      printf( "--- ERROR: Unable to connect to device: %s\n", IPADDRESS );
      return HARET_ERROR;
   }

   else {
	   // Initialization
		InitializeDevice( hapticMaster );

		if (haSendCommand(hapticMaster, "set inertia", inertia, response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master inertia");
			return HARET_ERROR;
		}
		
		if (haSendCommand(hapticMaster, "set state position", response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master state");
			return HARET_ERROR;
		}
		if (haSendCommand(hapticMaster, "set coulombfriction", 0.00001, response) || strstr(response, ERROR_MSG))
		{
			printf( "ERROR can't set Haptic Master Coulomb friction");
			return HARET_ERROR;
		}
	
		// Create springs to constrain motion
		if (haSendCommand(hapticMaster, "create spring spring_active", response) || strstr(response, ERROR_MSG))
 		{
 			printf("ERROR on spring_active creation");
 			return HARET_ERROR;
 		}
		else
		{
			if(	haSendCommand(hapticMaster, "set spring_active stiffness", springStiffness, response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active dampfactor", springDamping, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active deadband", springDeadband, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active maxforce", springMaxForce, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active direction", springDirection_active[posX], springDirection_active[posY], springDirection_active[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active pos", springPosition_active[posX], springPosition_active[posY], springPosition_active[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active vel", springVelocity_active[posX], springPosition_active[posY], springPosition_active[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_active enable", response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_active initialization");
 					return HARET_ERROR;
				}
		}

		if (haSendCommand(hapticMaster, "create spring spring_passive", response) || strstr(response, ERROR_MSG))
 		{
 			printf("ERROR on spring_passive creation");
 			return HARET_ERROR;
 		}
		else
		{
			if(	haSendCommand(hapticMaster, "set spring_passive stiffness", springStiffness, response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive dampfactor", springDamping, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive deadband", springDeadband, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive maxforce", springMaxForce, response)	|| strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive direction", springDirection_passive[posX], springDirection_passive[posY], springDirection_passive[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive pos", springPosition_passive[posX], springPosition_passive[posY], springPosition_passive[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive vel", springVelocity_passive[posX], springVelocity_passive[posY], springVelocity_passive[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_passive enable", response) || strstr(response, ERROR_MSG))	
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

		// Launch 
		while (totalNbCases.size() > 0)
		{
			std::cout << "Press Enter to start next block" << std::endl;
						
			// Initialize
			trialNb = -1;
			keyboardInput = 0;
			period = 0.;
			law = '0';
			axis = '0';
			timeSampling.clear();
			position.clear();

			// Wait for user to trigger beginning of block
			while (keyboardInput != 10)
				keyboardInput = getchar();
	
			// Pick a random case and read trajectory and parameters in file
			for (int i=0; i<totalNbCases.size(); i++)
			whichCase = rand() % totalNbCases.size();
			std::string inputFileName = "Input/trajectory_minjerk" + std::to_string(totalNbCases[whichCase]+1) + ".csv";  // TODO +1 is for if traj files start at 1 instead of 0
			std::cout << inputFileName << std::endl;
			std::ifstream inputFile(inputFileName, std::ios::in);
			if (inputFile)
			{
				std::string content;
				int ind;
				
				// Read period
				getline(inputFile, content);
				ind = content.find(',');
				period = std::stof(content.substr(ind+1));
				// Read law
				getline(inputFile, content);
				ind = content.find(',');
				law = content.substr(ind+1);
				// Read axis
				getline(inputFile, content);
				ind = content.find(',');
				axis = content.substr(ind+1);
				if (!axis.compare("X"))
				{
					activeAxis = posX;
					passiveAxis = posY;
				}
				else
				{
					activeAxis = posY;
					passiveAxis = posX;
				}
				// Read time and position
				while(getline(inputFile, content))
				{
					ind = content.find(',');
					timeSampling.push_back(std::stof(content.substr(0, ind)));
					position.push_back(std::stof(content.substr(ind+1)));
				}
				totalNbCases.erase(totalNbCases.begin() + whichCase);				
			}
			else
			{
				std::cout << "Cannot open input trajectory file" << std::endl;
				while(true){}
				return -1;
			}

			// Define active axisX
			for (int i=0; i<3; i++)
			{
				if (activeAxis == i)		
					springDirection_active[i] = 1.0;	
				else
					springDirection_active[i] = 0.0;
				if (passiveAxis == i)		
					springDirection_passive[i] = 1.0;	
				else
					springDirection_passive[i] = 0.0;
			}
			if(	haSendCommand(hapticMaster, "set spring_active direction", springDirection_active[posX], springDirection_active[posY], springDirection_active[posZ], response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_active set direction");
 					return HARET_ERROR;
				}	
			if(	haSendCommand(hapticMaster, "set spring_passive direction", springDirection_passive[posX], springDirection_passive[posY], springDirection_passive[posZ], response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_passive set direction");
 					return HARET_ERROR;
				}	
						
			while (trialNb < maxTrialNb)
			{
				std::cout << "Next Trial " << std::endl;
				countBip = 0;

					
				QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
				currentTime = (1. * currentTimeStamp) / timerFrequency; 
				startTime = currentTime;

				// Go smoothly to initial position
				if(haSendCommand(hapticMaster, "get modelpos;", response) || strstr(response, ERROR_MSG)) 
					printf("Error on reading HM position");
				else	
				{
					BreakResponse(str_pos, response, 1);
					ParseFloatVec(str_pos, currentPosition[posX], currentPosition[posY], currentPosition[posZ]);
					double distance = sqrt(pow(currentPosition[activeAxis] - position[0], 2) + pow(currentPosition[passiveAxis] - 0., 2) + pow(currentPosition[posZ] - heightHM,2));
					initialDiplacementTime = distance / maxVel;

					while (currentTime - startTime < initialDiplacementTime)
					{
						QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
						currentTime = (1. * currentTimeStamp) / timerFrequency; 
											
						springPosition_active[activeAxis] = currentPosition[activeAxis] + (position[0] - currentPosition[activeAxis]) * (currentTime - startTime) / initialDiplacementTime;
						springPosition_passive[passiveAxis] = currentPosition[passiveAxis] + (0. - currentPosition[passiveAxis]) * (currentTime - startTime) / initialDiplacementTime;
						springPosition_Z[posZ] = currentPosition[posZ] + (heightHM - currentPosition[posZ]) * (currentTime - startTime) / initialDiplacementTime;
						springVelocity_active[activeAxis] = 0;
						springVelocity_passive[passiveAxis] = 0;
						springVelocity_Z[posZ] = 0;
						if(	haSendCommand(hapticMaster, "set spring_active pos", springPosition_active[posX], springPosition_active[posY], springPosition_active[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_active update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_passive pos", springPosition_passive[posX], springPosition_passive[posY], springPosition_passive[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_passive update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Z update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_active vel", springVelocity_active[posX], springVelocity_active[posY], springVelocity_active[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_active update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_passive vel", springVelocity_passive[posX], springVelocity_passive[posY], springVelocity_passive[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_passive update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Z update");
		 					return HARET_ERROR;
						}
					}
				}

				// Initialize
				currentIndex = 0;
				springPosition_active[activeAxis] = position[0];
				startTime = currentTime;
			
				if (trialNb == -1)
				{
					PlaySoundA(demoSound, NULL, SND_ASYNC);
					std::cout << "Demo" << std::endl;
				}
				
				// Dwell time
				while (currentTime - startTime < dwellTime)
				{
					QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
					currentTime = (1. * currentTimeStamp) / timerFrequency; 
				}
				while (countBip < nbBip)
				{
					startTime = currentTime;
					PlaySoundA(bipSound, NULL, SND_ASYNC);
					countBip++;
					while (currentTime - startTime < reactionTime)
					{
						QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
						currentTime = (1. * currentTimeStamp) / timerFrequency; 
					}
				}

				startTime = currentTime;
				// Start ellipse  
				while (currentIndex < timeSampling.size()-2) {
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
					springPosition_active[activeAxis] = position[currentIndex];	
					springVelocity_active[activeAxis] = (position[currentIndex+1] - position[currentIndex])/(timeSampling[currentIndex]+1 - timeSampling[currentIndex]);	

					if(	haSendCommand(hapticMaster, "set spring_active pos", springPosition_active[posX], springPosition_active[posY], springPosition_active[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_active update");
		 					return HARET_ERROR;
						}
					if(	haSendCommand(hapticMaster, "set spring_active vel", springVelocity_active[posX], springVelocity_active[posY], springVelocity_active[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_active update");
		 					return HARET_ERROR;
						}						
				}
		
				// Signal end of movement
				PlaySoundA(endSound, NULL, SND_ASYNC);

				// Write data in file
				std::string nbTrialStr; // should be enough, less that 1000 trials + end character
				if (trialNb == -1)
					nbTrialStr = "demo";
				else
					nbTrialStr = std::to_string(trialNb);
				std::string nbCaseStr = std::to_string(caseNb);
				std::string ellipse_period = std::to_string(period);
				std::string filename = "Output/ellipse_" + subjectName + "_period_" + ellipse_period+ "_law_" + law + "_axis_" + axis + "_block_" + nbCaseStr + "_trial_" + nbTrialStr + ".csv";
				std::ofstream data_file(filename, std::ios::out);
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
			
				trialNb++;
			}
			caseNb++;

		}
   }

   	// Go smoothly back to to initial position
	if(haSendCommand(hapticMaster, "get modelpos;", response) || strstr(response, ERROR_MSG)) 
		printf("Error on reading HM position");
	else	
	{
		QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
		currentTime = (1. * currentTimeStamp) / timerFrequency; 
		startTime = currentTime;

		BreakResponse(str_pos, response, 1);
		ParseFloatVec(str_pos, currentPosition[posX], currentPosition[posY], currentPosition[posZ]);
		double distance = sqrt(pow(currentPosition[posX] - 0., 2) + pow(currentPosition[posY] - 0., 2));
		initialDiplacementTime = distance / maxVel;

		while (currentTime - startTime < initialDiplacementTime)
		{
			QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
			currentTime = (1. * currentTimeStamp) / timerFrequency; 
											
			springPosition_active[activeAxis] = currentPosition[activeAxis] + (0. - currentPosition[activeAxis]) * (currentTime - startTime) / initialDiplacementTime;
			springPosition_passive[passiveAxis] = currentPosition[passiveAxis] + (0. - currentPosition[passiveAxis]) * (currentTime - startTime) / initialDiplacementTime;
			springPosition_Z[posZ] = currentPosition[posZ] + (0. - currentPosition[posZ]) * (currentTime - startTime) / initialDiplacementTime;
			springVelocity_active[activeAxis] = 0;
			springVelocity_passive[passiveAxis] = 0;
			springVelocity_Z[posZ] = 0;
			if(	haSendCommand(hapticMaster, "set spring_active pos", springPosition_active[posX], springPosition_active[posY], springPosition_active[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_active update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_passive pos", springPosition_passive[posX], springPosition_passive[posY], springPosition_passive[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_passive update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_Z update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_active vel", springVelocity_active[posX], springVelocity_active[posY], springVelocity_active[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_active update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_passive vel", springVelocity_passive[posX], springVelocity_passive[posY], springVelocity_passive[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_passive update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_Z update");
		 		return HARET_ERROR;
			}
		}
	}
	//while(true){}

	haSendCommand(hapticMaster, "remove all", response);
	printf("remove all ==> %s\n", response);
      
	haSendCommand(hapticMaster, "set state stop", response);
	printf("set state stop ==> %s\n", response);
      
	if ( haDeviceClose(hapticMaster) ) {
		printf("---ERROR: closing device\n");
	}
   return 0;

}
