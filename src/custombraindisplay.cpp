//custombraindisplay is 3rd in the conga line, and holds all functionality for the brain
#include "positiontracking.cpp"
//extra included files for the brain
#include "pros/colors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <string>


//class for placing a robot on a 2d plane for 
class positiondisplay {                                                                                     //position display system
    private:
        //draws another display so it overlaps the old display
        void drawfield() {
            //overlaps the old display
            pros::screen::set_pen(COLOR_GRAY);
            pros::screen::fill_rect(220, 20, 420, 220);
            //draws the new display
            pros::screen::set_pen(COLOR_WHITE);
            for (int y = 0; y <= 6; y++) {
            pros::screen::draw_line(220 + (y*33.3333), 20, 220 + (y*33.3333), 220);
            pros::screen::draw_line(220, 20 + (y*33.3333), 420, 20 + (y*33.3333)); }
        }

        //draws a square that represents the robot so we know where the robot believes its positioned
        void drawrobot(double X, double Y, double theta) {
            double drawX = (X/18) + 320;
            double drawY = (Y/-18) + 120;
            double truetheta = (theta-90) * (PI/180);
            pros::screen::set_pen(COLOR_YELLOW);
            pros::screen::fill_circle(drawX, drawY, 10);
            pros::screen::set_pen(COLOR_YELLOW);
            pros::screen::draw_line(drawX, drawY, (cos(truetheta)*30)+drawX, (sin(truetheta)*30)+drawY);
        }
    public:
        //updates the robots current position
        void updateposition(double X, double Y, double theta) {
            //overlay the old field
            drawfield();
            //draw the robot in its new position
            drawrobot(X,Y,theta);
        }
};

positiondisplay posDisplay;

class autonomousSelector {};                                                                                 //auton selector


//sets the pen color relating to where a value lies on a low-high bar
void setColorStatus(double value, double low, double high, bool inverted) {                                  //function for setting status based on color
    double ygreenpoint =  high*0.2;
    double yellowpoint =  high*0.4;
    double orangepoint =  high*0.6;
    double redpoint =  high*0.8;
    if (value <= ygreenpoint) {pros::screen::set_pen(COLOR_GREEN);}
    if (value >= ygreenpoint) {pros::screen::set_pen(COLOR_YELLOW_GREEN);}
    if (value >= yellowpoint) {pros::screen::set_pen(COLOR_YELLOW);}
    if (value >= orangepoint) {pros::screen::set_pen(COLOR_ORANGE);}
    if (value >= redpoint) {pros::screen::set_pen(COLOR_RED);}
}

//function to print the motors value
void printMotorTemp(pros::Motor motor,std::string name,int line) {                                          //prints motor temps
    int motortemp = motor.get_temperature();
    setColorStatus(motortemp, 0, 100, false);
    pros::screen::print(TEXT_SMALL, line,"%s",name + ": " + (std::to_string(motortemp)));
}

//task function that constantly sets all of the temps every .5 seconds
void taskTempDisplay () {                                                                                   //also prints motor temps
    pros::delay(20);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(TEXT_SMALL, 1,"%s","Motor Temps");
    while (true) {
        printMotorTemp(driveLB, "driveLeft", 2);
        printMotorTemp(driveRB, "driveRight", 3);
        printMotorTemp(intake, "intakeMotor", 4);
        pros::delay(500);
    }
}

//creating a task function for running the position system
postracking posTracking;
void activatePositionTracking() {																//function to activate all position tracking functionality
	while (inertial.is_calibrating()) {
		pros::delay(20);
	}
	// posTracking.driftOffSet = inertial.get_accel().x;
	while (true) {
		posTracking.getAbsolutePosition();
        //update position display
		posDisplay.updateposition(FAPedX, FAPedY, FAPedTheta);
        pros::screen::print(TEXT_SMALL, 6,"%s","X: " + (std::to_string(FAPedX)));
        pros::screen::print(TEXT_SMALL, 7,"%s","Y: " + (std::to_string(FAPedY)));
		pros::delay(20);
	}
}