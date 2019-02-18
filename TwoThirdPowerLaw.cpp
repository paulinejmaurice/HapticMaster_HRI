// TwoThirdPowerLaw.cpp : Defines the entry point for the console application.
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

	double maxFX = 0.;
	double maxFY = 0.;
	double maxFZ = 0.;

	// Spring features 
	double springStiffness = 20000; // 10000.;  // TODO gain peut eter a regler en fonction de la vitesse/etendue du mouvement pour que l'ellipse desiree soit environ effectuee
	double springDamping=  2; //2*sqrt(inertia * springStiffness); //10. // Critical damping
	double springDeadband = 0.;
	double springMaxForce = 5000.;
	double springPosition_X[3];
	double springPosition_Y[3];
	double springPosition_Z[3];
	double springVelocity_X[3];
	double springVelocity_Y[3];
	double springVelocity_Z[3];
	double springDirection_X[3];
	double springDirection_Y[3];
	double springDirection_Z[3];
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
	}
	springDirection_X[posX] = 1.0;
	springDirection_Y[posY] = 1.0;
	springDirection_Z[posZ] = 1.0;
	double heightHM = -0.;

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
	int maxTrialNb = 10;
	int caseNb = 0;
	int whichCase;
	int maxNbCases = 18;
	std::vector<int> totalNbCases;
	for (int i=0; i<maxNbCases; i++)   // TODO change for real number of cases
		totalNbCases.push_back(i);
	int keyboardInput = 0;
	int nbEllipse = 4; // nb times ellipse is drawn within one trial

	// Ellipse features
	double axisX;
	double axisY;
	double period;
	double beta;
	std::vector<double> timeSampling;
	std::vector<double> phi;
	int currentIndex;

	// record
	std::string subjectName = "Eric";
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
		if (haSendCommand(hapticMaster, "create spring spring_X", response) || strstr(response, ERROR_MSG))
 		{
 			printf("ERROR on_X spring creation");
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
				haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG)||
				haSendCommand(hapticMaster, "set spring_X enable", response) || strstr(response, ERROR_MSG))	
				{
	 				printf("ERROR on spring_X initialization");
 					return HARET_ERROR;
				}
		}
		if (haSendCommand(hapticMaster, "create spring spring_Y", response) || strstr(response, ERROR_MSG))
 		{
 			printf("ERROR on Y spring creation");
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
	 				printf("ERROR on spring_Y initialization");
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
			beta = 0.;
			axisX = 0.;
			axisY = 0.;
			timeSampling.clear();
			phi.clear();

			// Wait for user to trigger beginning of block
			while (keyboardInput != 10)
				keyboardInput = getchar();
	
			// Pick a random case and read trajectory and parameters in file
			for (int i=0; i<totalNbCases.size(); i++)
			whichCase = rand() % totalNbCases.size();
			std::string inputFileName = "Input/trajectory_" + std::to_string(totalNbCases[whichCase]+1) + ".csv";  // TODO +1 is for if traj files start at 1 instead of 0
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
				totalNbCases.erase(totalNbCases.begin() + whichCase);				
			}
			else
			{
				std::cout << "Cannot open input trajectory file" << std::endl;
				while(true){}
				return -1;
			}

			std::cout << "period : " << period << std::endl;
			
			while (trialNb < maxTrialNb)
			{
				std::cout << "Next Trial " << std::endl;
				maxFX = 0.;
				maxFY = 0.;
				maxFZ = 0.;
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
					double distance = sqrt(pow(currentPosition[posX] - axisX * cos(phi[0]), 2) + pow(currentPosition[posY] - axisY * sin(phi[0]), 2) + pow(currentPosition[posZ] - heightHM,2));
					initialDiplacementTime = distance / maxVel;

					while (currentTime - startTime < initialDiplacementTime)
					{
						QueryPerformanceCounter((LARGE_INTEGER *)&currentTimeStamp);
						currentTime = (1. * currentTimeStamp) / timerFrequency; 
											
						springPosition_X[posX] = currentPosition[posX] + (axisX * cos(phi[0]) - currentPosition[posX]) * (currentTime - startTime) / initialDiplacementTime;
						springPosition_Y[posY] = currentPosition[posY] + (axisY * sin(phi[0]) - currentPosition[posY]) * (currentTime - startTime) / initialDiplacementTime;
						springPosition_Z[posZ] = currentPosition[posZ] + (heightHM - currentPosition[posZ]) * (currentTime - startTime) / initialDiplacementTime;
						springVelocity_X[posX] = 0;
						springVelocity_Y[posY] = 0;
						springVelocity_Z[posZ] = 0;
						if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_X update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Y update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Z update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_X update");
		 					return HARET_ERROR;
						}
						if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Y update");
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
				springPosition_X[posX] = axisX * cos(phi[0]);
				springPosition_Y[posY] = axisY * sin(phi[0]);
				springPosition_Z[posZ] = heightHM;
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
					springPosition_X[posX] = axisX * cos(phi[currentIndex]);
					springPosition_Y[posY] = axisY * sin(phi[currentIndex]);	
					springVelocity_X[posX] = - axisX * sin(phi[currentIndex]) * (phi[currentIndex+1] - phi[currentIndex])/(timeSampling[currentIndex]+1 - timeSampling[currentIndex]);
					springVelocity_Y[posY] = axisY * cos(phi[currentIndex]) * (phi[currentIndex+1] - phi[currentIndex])/(timeSampling[currentIndex]+1 - timeSampling[currentIndex]);	

					if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_X update");
		 					return HARET_ERROR;
						}
					if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Y update");
		 					return HARET_ERROR;
						}
					if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_X update");
		 					return HARET_ERROR;
						}
					if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
						{
			 				printf("ERROR on spring_Y update");
		 					return HARET_ERROR;
						}							
					if (springStiffness * abs(springPosition_X[posX] - currentPosition[posX]) + springDeadband * abs(springVelocity_X[posX] - currentVelocity[posX]) > maxFX)
						maxFX = springStiffness * abs(springPosition_X[posX] - currentPosition[posX])  + springDeadband * abs(springVelocity_X[posX] - currentVelocity[posX]);
					if (springStiffness * abs(springPosition_Y[posY] - currentPosition[posY])  + springDeadband * abs(springVelocity_Y[posY] - currentVelocity[posY]) > maxFY)
						maxFY = springStiffness * abs(springPosition_X[posY] - currentPosition[posY])  + springDeadband * abs(springVelocity_Y[posY] - currentVelocity[posY]);
					if (springStiffness * abs(springPosition_Z[posZ] - currentPosition[posZ]) + springDeadband * abs(springVelocity_Z[posZ] - currentVelocity[posZ]) > maxFZ)
						maxFZ = springStiffness * abs(springPosition_Z[posZ] - currentPosition[posZ]) + springDeadband * abs(springVelocity_Z[posZ] - currentVelocity[posZ]);
					//std::cout << springStiffness * (springPosition_X[posX] - currentPosition[posX]) << "-----" << springStiffness * (springPosition_Y[posY] - currentPosition[posY]) <<  "-----" << springStiffness * (springPosition_Z[posZ] - currentPosition[posZ]) << std::endl;
				}

				std::cout << "Fx max : " << maxFX << std::endl;
				std::cout << "Fy max : " << maxFY << std::endl;
				std::cout << "Fz max : " << maxFZ << std::endl;
		
				// Signal end of movement
				PlaySoundA(endSound, NULL, SND_ASYNC);

				// Write data in file
				std::string nbTrialStr; // should be enough, less that 1000 trials + end character
				if (trialNb == -1)
					nbTrialStr = "demo";
				else
					nbTrialStr = std::to_string(trialNb);
				std::string nbCaseStr = std::to_string(caseNb);
				std::string axisX_size = std::to_string(axisX);
				std::string axisY_size = std::to_string(axisY);
				std::string ellipse_period = std::to_string(period);
				std::string beta_law = std::to_string(beta);
				std::string filename = "Output/ellipse_" + subjectName + "_period_" + ellipse_period+ "_beta_" + beta_law + "_axisX_" + axisX_size + "_axisY_" + axisY_size + "_block_" + nbCaseStr + "_trial_" + nbTrialStr + ".csv";
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
											
			springPosition_X[posX] = currentPosition[posX] + (0. - currentPosition[posX]) * (currentTime - startTime) / initialDiplacementTime;
			springPosition_Y[posY] = currentPosition[posY] + (0. - currentPosition[posY]) * (currentTime - startTime) / initialDiplacementTime;
			springPosition_Z[posZ] = currentPosition[posZ] + (0. - currentPosition[posZ]) * (currentTime - startTime) / initialDiplacementTime;
			springVelocity_X[posX] = 0;
			springVelocity_Y[posY] = 0;
			springVelocity_Z[posZ] = 0;
			if(	haSendCommand(hapticMaster, "set spring_X pos", springPosition_X[posX], springPosition_X[posY], springPosition_X[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_X update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_Y pos", springPosition_Y[posX], springPosition_Y[posY], springPosition_Y[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_Y update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_Z pos", springPosition_Z[posX], springPosition_Z[posY], springPosition_Z[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_Z update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_X vel", springVelocity_X[posX], springVelocity_X[posY], springVelocity_X[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_X update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_Y vel", springVelocity_Y[posX], springVelocity_Y[posY], springVelocity_Y[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_Y update");
		 		return HARET_ERROR;
			}
			if(	haSendCommand(hapticMaster, "set spring_Z vel", springVelocity_Z[posX], springVelocity_Z[posY], springVelocity_Z[posZ], response) || strstr(response, ERROR_MSG))	
			{
			 	printf("ERROR on spring_Z update");
		 		return HARET_ERROR;
			}
		}
	}

	while(true){}

	haSendCommand(hapticMaster, "remove all", response);
	printf("remove all ==> %s\n", response);
      
	haSendCommand(hapticMaster, "set state stop", response);
	printf("set state stop ==> %s\n", response);
      
	if ( haDeviceClose(hapticMaster) ) {
		printf("---ERROR: closing device\n");
	}
   return 0; 
}
