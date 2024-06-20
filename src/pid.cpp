#include "main.h"

void PIDMover(
		int setPoint // how far you want to move in inches
		)
{
// controller and motor declarations
	pros::Controller Master(pros::E_CONTROLLER_MASTER);

	pros::Motor backLeft(16, 1);
	pros::Motor frontLeft(17, 1);
	pros::Motor_Group leftWheels({backLeft, frontLeft});

	pros::Motor backRight(14, 0);
	pros::Motor frontRight(15, 0);
	pros::Motor_Group rightWheels({backRight, frontRight});

	pros::Motor_Group allWheels({backLeft, backRight, frontLeft, frontRight});

// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	int tolerance = 10;
	bool actionCompleted = false;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral;
	int integralLimiter = 512; // customizable
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError;

// Constants -- tuning depends on whether the robot is moving or turning
	double kP = 1; // customizable
	double kI = 0.25; // customizable
	double kD = 0; // customizable

// Checks if the movement is positive or negative
	bool isPositive = setPoint > 0;

// PID LOOPING VARIABLES
	setPoint = setPoint * 2.54; // converts from inches to cm

	double wheelCircumference = 3.14 * 2.75; // 4 is the wheel diameter in inches
	double gearRatio = 1;
	double wheelRevolution = wheelCircumference * 2.54; // in cm
	long double singleDegree = wheelRevolution / 360;

	backRight.tare_position();
	backLeft.tare_position();
	frontRight.tare_position();
	frontLeft.tare_position();

	double br;
	double bl;
	double fr;
	double fl;

	double currentMotorReading = ((br + bl + fr + fl) / 4);
	double currentWheelReading = currentMotorReading * gearRatio;

	double currentDistanceMovedByWheel = 0;

	error = (int) (setPoint - currentDistanceMovedByWheel);
	prevError = error;

	int timeout = 0;

	while (actionCompleted != true) {
	// PID CALCULATION CODE
		
	// P: Proportional -- slows down as we reach our target for more accuracy

		// error = goal reading - current reading
		error = int (setPoint - currentDistanceMovedByWheel);
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
		// prevents the integral variable from causing the robot to overshoot
		if ((isPositive && (error <= tolerance)) || (!isPositive && (error >= -tolerance))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		if (integral >= integralLimiter) {
            integral /= 2;
        }
		// kI (integral constant) brings integral down to a reasonable/useful output number
		integralOut = integral * kI;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
        derivative = int (error - prevError);
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;

		power = proportionalOut + integralOut + derivativeOut;


		allWheels.move(power);






		pros::delay(15);

		br = backRight.get_position();
		bl = backLeft.get_position();
		fr = frontRight.get_position();
		fl = frontLeft.get_position();

		currentMotorReading = ((br + bl + fr + fl) / 4); // degrees
		currentWheelReading = currentMotorReading / gearRatio; // degrees = degrees * multiplier
		currentDistanceMovedByWheel = currentWheelReading * singleDegree; // centimeters

		if ((((currentDistanceMovedByWheel <= setPoint + tolerance) && (currentDistanceMovedByWheel >= setPoint - tolerance))) || 
			((power <= 10) && (power >= -10))) {
				actionCompleted = true;
				allWheels.brake();
		}
	}
}

void PIDTurner(
		int setPoint, // how far you want to move in inches
		int direction // 1 for left and 2 for right
		)
{
// controller, motor, and sensor declarations
	pros::Controller Master(pros::E_CONTROLLER_MASTER);

	pros::Motor backLeft(16, 1);
	pros::Motor frontLeft(17, 1);
	pros::Motor_Group leftWheels({backLeft, frontLeft});

	pros::Motor backRight(14, 0);
	pros::Motor frontRight(15, 0);
	pros::Motor_Group rightWheels({backRight, frontRight});
	
	pros::Motor_Group allWheels({backLeft, backRight, frontLeft, frontRight});

	pros::IMU Inertial(20);



// PID CALCULATION VARIABLES
// General Variables
	int error;
	int power;
	bool actionCompleted = false;

// Proportional Variables
	int proportionalOut;

// Integral Variables
	int integral;
	int integralLimiter;
	int integralOut;

// Derivative Variables
    int derivative;
    int derivativeOut;
	int prevError = error;

// Constants -- tuning depends on whether the robot is moving or turning
	double kP = 0.9;
	double kI = 0;
	double kD = 0.3;

// Checks if the movement is positive or negative
	bool isPositive = setPoint > 0;

// PID LOOPING VARIABLES
	int negativePower;

	int inertialReadingInit = Inertial.get_heading();
	int distanceToMove;

	if (direction == 1) {
		// standard left turn is negative, so the calculation makes it positive if it is a normal turn
		// ex: current = 90, goal = 45 -> -45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn left
		distanceToMove = inertialReadingInit - setPoint;
	}
	else if (direction == 2) {
		// standard right turn is positive, so the calculation keeps it positive if it is a normal turn
		// ex: current = 45, goal = 90 -> 45 degree turn -> positive 45 degree turn by calculation
		// 90 - 45 = 45 degree turn right
		distanceToMove = setPoint - inertialReadingInit;
	}

	// if the error is positive, then the calculation is fine and is left
	if (distanceToMove >= 0) {
		// do nothing
	}
	// otherwise, the turn takes the "long way" around the circle, and the calculation has provided the
	// value of the negative short way - adding 360 to this value gives the long way around the circle,
	// which is what is needed
	// ex: current = 90, goal = 45, direction = right -> calculated -45 degree turn -> + 360 -> 315 (length of long way)
	// 45 - 90 = -45 (short way, negative) + 360 = 315 (long way, positive)
	else {
		distanceToMove += 360;
	}
	// the calculation has now yielded a positive value that is the error needed of our turn in the proper
	// direction, making it similar to how a forward/backward movement is coded

	// finally, the code sets a new value that will be set to the distance moved to zero to finalize this similarity
	// distanceToMove is analogous to setPoint on PIDMover, and changeInReading is analogous to currentDistanceMovedByWheel
	int changeInReading = 0;

	prevError = (int) (distanceToMove - changeInReading);

	Master.print(0, 0, "DTM: %d", distanceToMove);

	int timeout = 0;

	while (!actionCompleted) {
	// PID CALCULATION CODE
		
	// P: Proportional -- slows down as we reach our target for more accuracy
	
		// error = goal reading - current reading
		error = distanceToMove - changeInReading;
		// kP (proportional constant) determines how fast we want to go overall while still keeping accuracy
		proportionalOut = error * kP;




	// I: Integral -- starts slow and speeds up as time goes on to prevent undershooting

		// starts the integral at the error, then compounds it with the new current error every loop
		integral = int (integral + error);
		// prevents the integral variable from causing the robot to overshoot
		if ((isPositive && (error <= 0)) || (!isPositive && (error >= 0))) {
			integral = 0;
		}
		// prevents the integral from winding up too much, causing the number to be beyond the control of
        // even kI
		// if we want to make this better, see Solution #3 for 3.3.2 in the packet
		if (error >= integralLimiter) {
            integral = 0;
        }
		// kI (integral constant) brings integral down to a reasonable/useful output number
		integralOut = integral * kI;



	// D: Derivative -- slows the robot more and more as it goes faster

        // starts the derivative by making it the rate of change from the previous cycle to this one
		// the error from the previous cycle should be taken as a parameter
        derivative = int (error - prevError);
		// sets the previous error to the previous error for use in the next cycle
		prevError = error;

        // kD (derivative constant) prevents derivative from over- or under-scaling
        derivativeOut = derivative * kD;

		power = proportionalOut + integralOut + derivativeOut;



	// PID LOOPING CODE

		negativePower = power * -1;

		// the power will never be negative and invert the turns because distanceToMove is always positive
		if (direction == 1) {
			leftWheels.move(negativePower);
			rightWheels.move(power);
		}
		else if (direction == 2) {
			leftWheels.move(power);
			rightWheels.move(negativePower);
		}

		pros::delay(15);

		// the change in reading is set to the absolute value of the change in reading due to everything being positive
		int changeInDistance = direction == 1 
			? inertialReadingInit - Inertial.get_heading() 
			: Inertial.get_heading() - inertialReadingInit;
		changeInReading = changeInDistance < 0
		    ? changeInDistance + 360
			: changeInDistance;

		// int exampleVar = Inertial.get_heading() - inertialReadingInit;
		// changeInReading = std::abs(Inertial.get_heading() - inertialReadingInit);

		if ((changeInReading >= distanceToMove) ||
			((changeInReading <= (distanceToMove + 10)) && (changeInReading >= (distanceToMove - 10)))) {
				actionCompleted = true;
				allWheels.brake();
		}
	}
}