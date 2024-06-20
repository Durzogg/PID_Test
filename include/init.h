#ifndef _PROS_INIT_H_
#define _PROS_INIT_H_

#include "main.h"

pros::Controller Master(pros::E_CONTROLLER_MASTER);

pros::Motor backLeft(16, 1);
pros::Motor frontLeft(17, 1);
pros::Motor_Group leftWheels({backLeft, frontLeft});

pros::Motor backRight(14, 0);
pros::Motor frontRight(15, 0);
pros::Motor_Group rightWheels({backRight, frontRight});

pros::Motor_Group allWheels({backLeft, backRight, frontLeft, frontRight});

pros::IMU Inertial(20);


int drvtrDZ = 10;
int drvtrFB;
int drvtrLR;



#endif