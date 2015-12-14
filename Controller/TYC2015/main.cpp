// Includes
#include <iostream>
#include <stdio.h>
#include <LocoKitInterface.h>

// Defines
#define MOTOR_FL		12  // Motor 1
#define MOTOR_FR		4   // Motor 2
#define MOTOR_BL		31  // Motor 3
#define MOTOR_BR		6   // Motor 4

// Global variables
LocoKitInterface *LKI;

// Functions

/********************************************************************************/
/* High level commands - can be designed according to the needed task           */
/********************************************************************************/
void stopActuators()
{
    // Stop motion
	LKI->setActuatorStopped(MOTOR_FL);
	LKI->setActuatorStopped(MOTOR_FR);
	LKI->setActuatorStopped(MOTOR_BL);
	LKI->setActuatorStopped(MOTOR_BR);
}

void goForwardPWM(float pmw)
{
    // Stop old motion
    stopActuators();

    // Start actuation
	LKI->setActuatorPWM(pmw, MOTOR_FL);
	LKI->setActuatorPWM(pmw, MOTOR_BR);
	LKI->setActuatorPWM(-pmw, MOTOR_FR);
	LKI->setActuatorPWM(-pmw, MOTOR_BL);
}

void goForwardConstantSpeedInterpolating(float period, float phaseoffset)
{
    // Stop old motion
    stopActuators();

    // Start actuation
    LKI->setConstantSpeedInterpolatingFunction(MOTOR_FL, period, phaseoffset, 0);
    LKI->setConstantSpeedInterpolatingFunction(MOTOR_BR, period, phaseoffset, 1);
    LKI->setConstantSpeedInterpolatingFunction(MOTOR_BL, period, phaseoffset, 0);
    LKI->setConstantSpeedInterpolatingFunction(MOTOR_FR, period, phaseoffset, 1);
}

void goBackwardPWM(float pmw)
{
    // Stop old motion
    stopActuators();

    // Start actuation
	LKI->setActuatorPWM(-pmw, MOTOR_FL);
	LKI->setActuatorPWM(-pmw, MOTOR_BR);
	LKI->setActuatorPWM(pmw, MOTOR_FR);
	LKI->setActuatorPWM(pmw, MOTOR_BL);
}

void goRightPWM(float pmw)
{
    // Stop old motion
    stopActuators();

    // Start actuation
	LKI->setActuatorPWM(pmw, MOTOR_FL);
	LKI->setActuatorPWM(pmw, MOTOR_BR);
	LKI->setActuatorPWM(-pmw/4.0, MOTOR_FR);
	LKI->setActuatorPWM(-pmw/4.0, MOTOR_BL);
}

void goLeftPWM(float pmw)
{
    // Stop old motion
    stopActuators();

    // Start actuation
	LKI->setActuatorPWM(pmw/4.0, MOTOR_FL);
	LKI->setActuatorPWM(pmw/4.0, MOTOR_BR);
	LKI->setActuatorPWM(-pmw, MOTOR_FR);
	LKI->setActuatorPWM(-pmw, MOTOR_BL);
}

/********************************************************************************/
/* Gait commands                                                                */
/********************************************************************************/
void forwardWalkGait(float period, float phaseoffset)
{
    // Stop old motion
    stopActuators();

    // Front legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(31, 0.75, 90, 1); // Right
    LKI->setConstantSpeedInterpolatingFunction(6, 0.75, 270, 0); // Left

    // Back legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(4, 0.75, 180, 1); // Right
    LKI->setConstantSpeedInterpolatingFunction(12, 0.75, 0, 0);  // Left
}

void forwardBoundGait(float period, float phaseoffset)
{
    // Stop old motion
    stopActuators();

    // Front legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(31, 0.35, 5, 0);  // Right
    LKI->setConstantSpeedInterpolatingFunction(6, 0.35, 185, 1); // Left

    // Back legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(4, 0.35, 180, 0); // Right
    LKI->setConstantSpeedInterpolatingFunction(12, 0.35, 0, 1);  // Left
}

void forwardTrotGait(float period, float phaseoffset)
{
    // Stop old motion
    stopActuators();

    // Front legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(31, 0.75, 180, 1); // Right
    LKI->setConstantSpeedInterpolatingFunction(6, 0.75, 180, 0);  // Left

    // Back legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(4, 0.75, 0, 1);    // Right
    LKI->setConstantSpeedInterpolatingFunction(12, 0.75, 0, 0);   // Left
}

void forwardPaceGait(float period, float phaseoffset)
{
    // Stop old motion
    stopActuators();

    // Front legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(31, 0.75, 90, 0);  // Right
    LKI->setConstantSpeedInterpolatingFunction(6, 0.75, 90, 1);   // Left

    // Back legs, opposite direction
    LKI->setConstantSpeedInterpolatingFunction(4, 0.75, 270, 0);  // Right
    LKI->setConstantSpeedInterpolatingFunction(12, 0.75, 270, 1); // Left
}

/********************************************************************************/
/* Print sensor info commands                                                   */
/********************************************************************************/
void printMotorVelocity()
{
    // Reset terminal
    system("stty cooked");

    std::cout << std::endl << "M1Vel:  M2Vel:  M3Vel:  M4Vel:" << std::endl;

    float f1, f2, f3, f4;
    LKI->getActuatorVelocity(MOTOR_FL, f1);
    LKI->getActuatorVelocity(MOTOR_FR, f2);
    LKI->getActuatorVelocity(MOTOR_BL, f3);
    LKI->getActuatorVelocity(MOTOR_BR, f4);

    std::cout << f1 << ", " << f2 << ", " << f3 << ", " << f4 << std::endl;
}

void printMotorPWM()
{
    // Reset terminal
    system("stty cooked");

    std::cout << std::endl << "M1PWM:  M2PWM:  M3PWM:  M4PWM:" << std::endl;

    float f1, f2, f3, f4;
    LKI->getActuatorPWM(MOTOR_FL, f1);
    LKI->getActuatorPWM(MOTOR_FR, f2);
    LKI->getActuatorPWM(MOTOR_BL, f3);
    LKI->getActuatorPWM(MOTOR_BR, f4);

    std::cout << f1 << ", " << f2 << ", " << f3 << ", " << f4 << std::endl;
}

void printMotorPosition()
{
    // Reset terminal
    system("stty cooked");

    std::cout << std::endl << "M1Pos:  M2Pos:  M3Pos:  M4Pos:" << std::endl;

    float f1, f2, f3, f4;
    LKI->getActuatorPosition(MOTOR_FL, f1);
    LKI->getActuatorPosition(MOTOR_FR, f2);
    LKI->getActuatorPosition(MOTOR_BL, f3);
    LKI->getActuatorPosition(MOTOR_BR, f4);

    std::cout << f1 << ", " << f2 << ", " << f3 << ", " << f4 << std::endl;
}

void printIMU()
{
    // Reset terminal
    system("stty cooked");

    std::cout << std::endl << "ACC_X:  ACC_Y:  ACC_Z:  GYO_X:  GYO_Y:  GYO_Z:" << std::endl;

    LKI->updateSensorValueRawFloat_array();
    for(int i=0; i<6; i++)
    {
        std::cout << LKI->sensory_inputs[i];

        if(i<5)
            std::cout << ", ";
    }

    std::cout << std::endl;
}

void controller()
{
    // Set PWM
    float mainPWM = 500;

	// Wait for single character
    while(true)
    {
        // Set terminal to raw mode
        system("stty raw");

        // Get input
        char input = getchar();

        // Gaits (using constant speed interpolation)
        if(input == '1')
        {
            forwardWalkGait(0, 0);
        }
        else if(input == '2')
        {
            forwardTrotGait(0, 0);
        }
        else if(input == '4')
        {
            forwardBoundGait(0, 0);
        }
        else if(input == '3')
        {
            forwardPaceGait(0, 0);
        }
        // Standard PWM
        else if(input == 'w')
        {
            goForwardPWM(mainPWM);
        }
        else if(input == 's')
        {
            goBackwardPWM(mainPWM);
        }
        else if(input == 'd')
        {
            goRightPWM(mainPWM);
        }
        else if(input == 'a')
        {
            goLeftPWM(mainPWM);
        }
        // Motor stop
        else if(input == 'p')
        {
            stopActuators();
        }
        else if(input == 'm')
        {
            goForwardConstantSpeedInterpolating(2, 180);
        }
        // Fetching sensor inputs
        else if(input == 'v')
        {
            printMotorVelocity();
        }
        else if(input == 'x')
        {
            printMotorPWM();
        }
        else if(input == 'z')
        {
            printIMU();
        }
        else if(input == 'c')
        {
            printMotorPosition();
        }
        // Program control
        else if(input == 'q')
        {
			break;
        }
        else if(input == 't')
        {
			LKI->terminate_connection_with_server();
		}
	}
}

int main()
{
    // Create locokit instance
    LKI = new LocoKitInterface;

    // Check connection
    if(LKI->establish_connection() == -1)
    {
        std::cerr << "Error from LocoKitInterface: a connection couldn't be established..." << std::endl;
		return -1;
	}
    else
    {
        std::cout << "connected to robot..." << std::endl;
    }

    // Start controller loop
    controller();

    // Reset terminal
    system("stty cooked");

    // Return
	return 0;
}
