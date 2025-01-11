//setup is the first file in the conga line, and sets up all of the hardware related functionality
#include "main.h"

//assigning the master controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Creating the drivetrain motors and assigning their groups
pros::Motor driveLF(5,pros::E_MOTOR_GEARSET_06, true);
pros::Motor driveLB(6,pros::E_MOTOR_GEARSET_06, true);
pros::Motor driveLT(7,pros::E_MOTOR_GEARSET_06, false);

pros::Motor_Group driveLeft ({driveLF,driveLB,driveLT});

pros::Motor driveRF(2,pros::E_MOTOR_GEARSET_06, false);
pros::Motor driveRB(3,pros::E_MOTOR_GEARSET_06, false);
pros::Motor driveRT(4,pros::E_MOTOR_GEARSET_06, true);

pros::Motor_Group driveRight ({driveRF,driveRB,driveRT});

//assigning other motors
pros::Motor intake(8,pros::E_MOTOR_GEARSET_06,true);
pros::Motor wallstake(10,pros::E_MOTOR_GEARSET_18,false);

//assigning pneumatics
pros::ADIDigitalOut mogoMech ('A');

//assigning sensors
pros::IMU inertial (1);
pros::GPS gps (11);

//tuneable values for certain systems
const int drivetrainTurnGoverner = 2;
const double ENCadjustment = -1.57;

//double values for storing the final absolute position on the robot
double FAPedX = 0;
double FAPedY = 0;
double FAPedTheta = 0;

/** PID SYSTEM
 * PID, or Proportional, Integral, Derivative, is a feedback controller that uses
 * past, present, and future error to determine how much and how quickly to correct
 * a process, the controller uses three times, P, I, and D, that are equivilant to
 * magnitude, duration, and rate of change of the error. 
 */
class PIDSystem {																				//PID system class and tunable values

	private:

		//TURNING
		//variables stored for the loop
		double Tprevious_error = 0;
		double Tintegral = 0;
		double Tintegral_limit = 50;

		//PID values for turning
		double TkP = 0.5;
		double TkI = 0.04;
		double TkD = 0.2;

		//DISTANCE PID
		//variables stored for the loop
		double Dprevious_error = 0;
		double Dintegral = 0;
		double Dintegral_limit = 50;

		//PID values for distance
		double DkP = 0.25;
		double DkI = 0.03;
		double DkD = 0.2;

	//functions for the system are stored here
	public:
 		double distancePID(double target, double current) {										//PID code for forward/backward velocity
    		double error = target - current;

    		//proportional equation
    		double proportional = error * DkP;
    
    		//integral equation
    		Dintegral += error;
			//if integral error gets too large, change it to the max (prevents windup)
			if (Dintegral > Dintegral_limit) Dintegral = Dintegral_limit;
    		if (Dintegral < -Dintegral_limit) Dintegral = -Dintegral_limit;
    		double integral_part = Dintegral * DkI;
    
    		//derivative equation
    		double derivative = (error - Dprevious_error) * DkD;
    		Dprevious_error = error;
    
    		//combine the numbers and return the output
			
			/*take together the proportional and the derivative then smash together those
			 *two different expressions with inertial to create and push out an added number
			 * 想像上のテクニック: 出力
			 */
    		double output = proportional + integral_part + derivative;
    
    		return output;
		}

		double turnPID(double target, double current) {											//PID code for turning velocity
    		double error = target - current;
			//if statements for fixing zero turns (for turning)
			if (error > 180) error -= 360;
			if (error < -180) error += 360;
    		//proportional equation
    		double proportional = error * TkP;
    		//integral equation
    		Tintegral += error;
			//if integral error gets too large, change it to the max (prevents windup)
			if (Tintegral > Tintegral_limit) Tintegral = Tintegral_limit;
    		if (Tintegral < -Tintegral_limit) Tintegral = -Tintegral_limit;
    		double integral_part = Tintegral * TkI;
    		//derivative equation
    		double derivative = (error - Tprevious_error) * TkD;
    		Tprevious_error = error; 
    		//combine the numbers and return the output
			
			/*take together the proportional and the derivative then smash together those
			 *two different expressions with inertial to create and push out an added number
			 * 想像上のテクニック: 出力
			 */
    		double output = proportional + integral_part + derivative;
    
    		return output;
		}

		//small function for reseting the loop for reusability
		void resetvariables() {																	//function resets values in the PID system
			Tprevious_error = 0;
			Tintegral = 0;
			Dprevious_error = 0;
			Dintegral = 0;
		}
};
PIDSystem PID;