#include "squiggles/geometry/controlvector.hpp"// IWYU pragma: keep
#include "squiggles/geometry/pose.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/timer.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "squiggles/math/quinticpolynomial.hpp"// IWYU pragma: keep
#include "squiggles/physicalmodel/physicalmodel.hpp"// IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp" // IWYU pragma: keep
#include <cstdint> // IWYU pragma: keep
#include <numbers>
#include <sys/_intsup.h>
#include "lemlib/pose.hpp"
#include "lemlib-tarball/api.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "squiggles/squiggles.hpp"// IWYU pragma: keep

//ASSET(skills_txt);

//lemlib_tarball::Decoder skillsFollow(skills_txt);

void ramsete(std::vector<squiggles::Pose> points, float timeout, bool async = false, bool forwards = true) {
    // Convert inches to meters and degrees to radians
    for (size_t i = 0; i < points.size(); i++) {
        points[i].x *= 0.0254;           // inches -> meters
        points[i].y *= 0.0254;           // inches -> meters
        points[i].yaw *= (std::numbers::pi / 180.0);  // degrees -> radians
    }
    
    chassis.ramsete(points, timeout, async, forwards);
}

// Skills code
void skills(){
	allowedXOff = 0;
	lemlib::setPose({-17, -49.5, 0});
	descore.set_value(HIGH);
	intaking = true;

	chassis.moveToPoint(-20, -30.5, 2000);
	chassis.moveToPoint(-22.5, -22.5, 2000, {}, false);
	intaking = false;
	middleScore.set_value(HIGH);

	chassis.turnToHeading(-134,700);
	chassis.moveToPoint(-10, -13, 2000, {.forwards = false}, false);
	hood.set_value(HIGH);
	intaking = true;
	middleGoal = true;
	pros::delay(3000);
	middleGoal = false;
	hood.set_value(LOW);
	allowedXOff = 4;
	middleScore.set_value(LOW);
	chassis.moveToPoint(-47.6, -41, 2000, {.maxSpeed = 100});
	matchLoader.set_value(HIGH);
	chassis.turnToPoint(-47.5, -58, 700);
	chassis.moveToPoint(-47.5, -55,1000, {} , false);
	chassis.moveToPoint(-47.5, -61,1000, {} , false);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.brake();
	leftMotors.brake();
	pros::delay(1800);
	chassis.moveToPoint(-62.5, -26, 2000, {.forwards = false}, false);
	chassis.moveToPoint(-61.5, 26, 2000, {.forwards = false});
	chassis.moveToPoint(-48, 46, 2000, {.forwards = false});
	chassis.turnToHeading(0, 700);
	chassis.moveToPoint(-47.5, 30, 2000, {.forwards = false}, false);
	hood.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(-47, 54, 2000, {}, false);
	hood.set_value(LOW);
	chassis.moveToPoint(-47, 58, 2000, {}, false);
	rightMotors.brake();
	leftMotors.brake();
	pros::delay(1800);
	chassis.moveToPoint(-47.5, 30, 2000, {.forwards = false}, false);
	hood.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(-47.5, 44, 2000, {}, false);
	hood.set_value(LOW);
	chassis.turnToHeading(90, 700);
	allowedYOff = 10;
	chassis.moveToPoint(47, 44, 4000, {.maxSpeed = 90});	
	chassis.waitUntil(50);
	allowedYOff = 4;
	chassis.waitUntilDone();
	allowedXOff = 10;
	pros::delay(600);
	allowedXOff = 4;
	chassis.turnToPoint(47.1, 54, 700);
	chassis.moveToPoint(47.1, 53.5, 1000);
	chassis.moveToPoint(46.5,61.5, 2000, {}, false);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.brake();
	leftMotors.brake();
	pros::delay(1500);
	chassis.moveToPoint(62, 24, 2000, {.forwards = false});
	chassis.moveToPoint(62.5, -26, 2000, {.forwards = false});
	chassis.moveToPoint(49, -45.5, 1000, {.forwards = false});
	chassis.turnToHeading(180,700);
	chassis.moveToPoint(47, -29, 1000, {.forwards = false}, false);
	hood.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(47.5, -57, 1000, {. maxSpeed = 80}, false);
	hood.set_value(LOW);
	chassis.moveToPoint(44.5, -60, 600, {}, false);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.brake();
	leftMotors.brake();
	pros::delay(1800);
	chassis.moveToPoint(46.5, -29.5, 1000, {.forwards = false}, false);
	hood.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(26, -61, 1000, {}, false);
	hood.set_value(LOW);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(16, -63.5, 1000);
	chassis.turnToHeading(-90, 700, {}, false);
	wheelLift.set_value(HIGH);
	intaking = false;
	outtaking = true;
	leftMotors.move_velocity(160);
	rightMotors.move_velocity(160);
	pros::delay(1400);
	leftMotors.brake();
	rightMotors.brake();


	

	/*
move to 47, -45.5
turn to heading 180
move to 47.5, -30.5, backwards
move to 47.5, -54
move to 47.5, -58
move to 47.5, -30.5, backwards
move to 26, -61
move to 15, -63.5
tracking wheel up
	outtaking = true
*/



	/*
	chassis.moveToPoint(-30, 62, 2000);
	chassis.turnToHeading(70, 700);
	chassis.moveToPoint(-19, 66, 2000, {}, false);
	wheelLift.set_value(HIGH);
	intaking = true;
	leftMotors.move_velocity(120);
	rightMotors.move_velocity(120);
	pros::delay(2000);
	wheelLift.set_value(LOW);
	allowedXOff = 0;
	allowedYOff = 0;
	leftMotors.brake();
	rightMotors.brake();
	chassis.moveToPoint(20, 44, 2000, {}, false);
	allowedXOff = 40;
	pros::delay(1700);
	allowedYOff = 4;
	pros::delay(1000);
	allowedXOff = 4;

	*/


/*



*/






	/*
x, y, direction
-20.5, -30.5, forwards
-33, -22.5, turn to -134 degrees
-11.7, -11.3, backwards
put down intake and score mid goal
-47.5, -41, forwards
matchload down
-47.5, -58, forwards
-63, -26, backwards
matchloader up
-62, 26, backwards
-48, 46, backwards
turn to heading 0
-47.5, 31, backwards
score and matchloader down
-47, 54, forwards
-47, 58, forwards
-47.5, 31, backwards
score and matchloader up
-30, 62, forwards
turn to heading 70
-17, 64, forwards
*/

	/*
	chassis.moveToPoint(-20, -32, 1000);
	chassis.moveToPoint(-18.5, -21, 1000);
	chassis.turnToHeading(-172,700);
	chassis.moveToPoint(-17, -15, 1000, {.forwards= false});
	chassis.moveToPoint(-46, -43, 2000);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(-46.5, -57, 1000);
	chassis.moveToPoint(-60, -22, 1000, {.forwards = false});
	chassis.moveToPoint(-60, 30, 2000, {.forwards = false});
	chassis.moveToPoint(-49.5, 49, 1000, {.forwards = false});
	chassis.turnToHeading(0, 700);
	chassis.moveToPoint(-48.5, 33, 1000, {.forwards = false});
	chassis.moveToPoint(-49.5, 61, 1000);
	chassis.moveToPoint(-48.5, 33, 1000, {.forwards = false});
	chassis.moveToPoint(-48.5, -44, 1000);
	*/

	
	


	/*hood.set_value(HIGH);
    chassis.setPose({-19.7, -50.3, 0});
	intaking = true;
	chassis.moveToPoint(-24, -27, 2000);
	chassis.waitUntil(15);
	matchLoader.set_value(HIGH);
	chassis.turnToPoint(-13, -17, 700, {.forwards = false});
	chassis.moveToPoint(-13, -13, 1000, {.forwards = false}, false);
	hood.set_value(LOW);
	intaking = false;
	outtaking = true;
	pros::delay(400);
	outtaking = false;
	middleGoal = true;
	pros::delay(1000);
	middleGoal = false;
	outtaking = true;
	pros::delay(200);
	outtaking = false;
	middleGoal = true;
	pros::delay(1000);
	middleGoal = false;
	hood.set_value(HIGH);
	matchLoader.set_value(LOW);
	intaking = true;
	chassis.moveToPoint(-48, -48, 2000, {.maxSpeed = 80});
	chassis.turnToHeading(180, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-49, -61.5, 1000, {.maxSpeed = 50}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1700);
	chassis.moveToPoint(-61, -24, 2000, {.forwards = false});
	matchLoader.set_value(LOW);
	descore.set_value(HIGH);
	chassis.moveToPoint(-61, 44, 3000, {.forwards = false, .maxSpeed = 80});
	intaking = false;
	outtaking = true;
	pros::delay(100);
	outtaking = false;
	chassis.turnToPoint(-48, 36, 700, {.forwards = false});
	chassis.moveToPoint(-48, 36, 1000, {.forwards = false}, false);
	chassis.turnToPoint(-47, 27, 700, {.forwards = false});
	chassis.moveToPoint(-47, 24.5, 1000, {.forwards = false, .maxSpeed = 100}, false);
	hood.set_value(LOW);
	intaking = true;
	matchLoader.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(-46.5, 63, 2000, {.maxSpeed = 60}, false);
	hood.set_value(HIGH);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1900);
	intaking = false;
	outtaking = true;
	pros::delay(100);
	outtaking = false;
	chassis.moveToPoint(-47, 25.5, 1000, {.forwards = false, .maxSpeed = 100}, false);
	hood.set_value(LOW);
	intaking = true;
	pros::delay(2200);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-46, 36, 1000, {}, false);
	hood.set_value(HIGH);
	
	chassis.turnToPoint(-24, 65, 700, {.forwards = false});
	chassis.moveToPoint(-32, 62, 1000, {.forwards = false});
	chassis.moveToPoint(-22, 64, 1000, {.forwards = false});
	wheelLift.set_value(HIGH);
	chassis.moveToPoint(10, 67, 5000, {.forwards = false, .minSpeed = 127}, false);
	acceptedMCLError = 50;
	wheelLift.set_value(LOW);
	chassis.moveToPoint(34, 48, 2000, {}, false);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(-20, 4000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 40});
	
	chassis.turnToHeading(90, 700);
	chassis.moveToPoint(48, 48, 4000, {.maxSpeed = 80});
	chassis.turnToHeading(0, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(45.5, 64, 1000, {.maxSpeed = 70}, false);
	//acceptedMCLError = 4;
	hood.set_value(HIGH);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1900);
	intaking = false;
	hood.set_value(LOW);
	chassis.moveToPoint(60, 48, 1000, {.forwards = false});
	chassis.moveToPoint(60, -26, 2000, {.forwards = false});
	chassis.moveToPoint(48, -48, 1000, {.forwards = false});
	chassis.turnToHeading(180, 900);
	chassis.moveToPoint(44.5, -26, 1000, {.forwards = false}, false);
	hood.set_value(LOW);
	intaking = true;
	matchLoader.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(46, -62, 1200, {.maxSpeed = 70}, false);
	hood.set_value(HIGH);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1900);
	intaking = false;
	chassis.moveToPoint(44.5, -26, 1000, {.forwards = false}, false);
	hood.set_value(LOW);
	intaking = true;
	pros::delay(2000);
	intaking = false;
	chassis.moveToPoint(44.5, -44, 1000);
	matchLoader.set_value(LOW);
	chassis.turnToPoint(24, -59, 700, {.forwards = false});
	chassis.moveToPoint(24, -59, 3000, {.forwards = false});
	wheelLift.set_value(HIGH);
	chassis.moveToPoint(-10, -59, 4000, {.forwards = false, .minSpeed = 127});
	*/


}


// Right side long goal only code
void rightRush(){
    //acceptedMCLError = 5;
	middleScore.set_value(HIGH);
	intaking = true;
	chassis.setPose({5.7, -46, -90});
	chassis.moveToPoint(47, -46, 2000, {.forwards = false});
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(46.5, -62, 1000, {}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(300);
	chassis.moveToPoint(47.5, -26, 1000, {.forwards = false, .maxSpeed = 90}, false);
	matchLoader.set_value(LOW);
	hood.set_value(HIGH);
	intaking = false;
	outtaking = true;
	pros::delay(150);
	outtaking = false;
	intaking = true;
	pros::delay(1500);
	chassis.moveToPoint(47.5, -46, 1000);
	chassis.moveToPoint(37, -26, 1000, {.forwards = false}, false);
	chassis.moveToPoint(37, -11, 1000, {.forwards = false}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	/*
	chassis.setPose({17.2, -51.2, 0});
	hood.set_value(LOW);
	intaking = true;
	chassis.moveToPoint(21, -28, 6000, {.maxSpeed = 55});
	chassis.waitUntil(12);
	matchLoader.set_value(HIGH);
	//chassis.turnToHeading(45, 700, {}, false);
	
	chassis.moveToPoint(40, -12, 2000, {});
	chassis.waitUntil(16);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(20, -26, 1000, {.forwards = false});
	
	chassis.moveToPoint(48, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	matchLoader.set_value(LOW);
	chassis.turnToHeading(180, 700);
	
	chassis.moveToPoint(48.5, -25, 1000, {.forwards = false, .maxSpeed = 80}, false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	intaking = false;
	matchLoader.set_value(HIGH);
	
	
	chassis.moveToPoint(47, -62, 1400, {.maxSpeed = 60}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	intaking = true;
	hood.set_value(HIGH);
	pros::delay(200);
	*/
	/*	chassis.moveToPoint(51, -25, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	hood.set_value(HIGH);
	matchLoader.set_value(LOW);
	pros::delay(600);
	intaking=false;
	outtaking = true;
	pros::delay(100);
	outtaking = false;
	intaking = true;
	pros::delay(1500);
	chassis.moveToPoint(48, -44, 1000, {});
	chassis.waitUntil(3);
	hood.set_value(LOW);
	descore.set_value(LOW);
	chassis.moveToPoint(44, -30, 1000, {.forwards = false});
	chassis.moveToPoint(43, -10, 2000, {.forwards = false}, false);
	chassis.turnToHeading(170, 700, {}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	*/


}


// Right side normal code
void rightSplit(){
	allowedXOff = 0;
	lemlib::setPose({17, -49.5, 0});
	intaking = true;

	chassis.moveToPoint(20, -30.5, 2000);
	chassis.moveToPoint(22.5, -22.5, 2000, {}, false);
	intaking = false;

	chassis.turnToPoint(10.5, -13.5, 700);
	chassis.moveToPoint(10.5, -13.5, 2000, {}, false);
	intaking = false;
	outtaking = true;
	pros::delay(2000);
	outtaking = false;
	intaking = true;
	allowedXOff = 4;
	chassis.moveToPoint(47.6, -41, 2000, {.forwards = false, .maxSpeed = 100});
	hood.set_value(LOW);
	chassis.turnToPoint(47.5, -58, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(47.5, -55,1000, {} , false);
	chassis.moveToPoint(47.5, -61.5,600, {} , false);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.brake();
	leftMotors.brake();
	chassis.moveToPoint(47.5, -29.5, 2000, {.forwards = false, .maxSpeed = 80}, false);
	hood.set_value(HIGH);
	pros::delay(1800);
	chassis.moveToPoint(47.5, -46, 2000, {.maxSpeed = 80}, false);
	chassis.moveToPoint(37.5, -26, 1000, {.forwards = false});
	chassis.moveToPoint(38, -11, 1000, {.forwards = false}, false);
	rightMotors.brake();
	leftMotors.brake();

	/*
	//acceptedMCLError = 5;
	intaking = true;
	middleScore.set_value(HIGH);
	chassis.setPose({5.7, -46, -90});
	chassis.moveToPoint(47, -46, 2000, {.forwards = false});
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(46.5, -63, 1000, {.maxSpeed = 90}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(300);
	chassis.moveToPoint(47.5, -26, 1000, {.forwards = false, .maxSpeed = 90}, false);
	matchLoader.set_value(LOW);
	hood.set_value(HIGH);
	intaking = false;
	outtaking = true;
	pros::delay(150);
	outtaking = false;
	intaking = true;
	pros::delay(1500);
	//acceptedMCLError = 1;
	chassis.moveToPoint(47.5, -46, 1000, {}, false);
	hood.set_value(LOW);
	chassis.moveToPoint(28, -20, 2000, {.maxSpeed = 80});
	pros::delay(1250);
	matchLoader.set_value(HIGH);
	pros::delay(1000);
	matchLoader.set_value(LOW);
	chassis.moveToPose(15, -4.5, -45, 1000, {.maxSpeed = 50}, false);
	intaking = false;
	outtaking = true;
	pros::delay(2000);
	//chassis.moveToPoint(36, -40, 1000, {.forwards = false});
	*/
	/*
	chassis.swingToPoint(42, -10, DriveSide::RIGHT, 1000);
	chassis.moveToPoint(42, -15, 2000, {.forwards = false}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	*/
	/*
    chassis.setPose({17.2, -51.2, 0});
	hood.set_value(HIGH);
	intaking = true;
	chassis.moveToPoint(20, -26, 1000, {.maxSpeed = 80});
	chassis.waitUntil(12);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(-45, 700, {}, false);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(15.5, -17.25, 2000, {.maxSpeed = 50}, false);
	intaking = false;
	outtaking = true;
	pros::delay(800); // Change to 1000 later
	outtaking = false;
	intaking = true;
	chassis.moveToPoint(49, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	chassis.turnToHeading(180, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(47.5, -61, 1000);
	pros::delay(1300);
	chassis.moveToPoint(49, -25, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	hood.set_value(LOW);
	pros::delay(2200);
	matchLoader.set_value(LOW);
	intaking = false;
	chassis.moveToPoint(45, -43, 2000);
	hood.set_value(HIGH);
	chassis.moveToPoint(38, -30, 2000, {.forwards = false});
	chassis.turnToPoint(38, -11, 700, {.forwards = false});
	chassis.moveToPoint(37.75, -11, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.turnToHeading(140,700);
	*/

    

}

// Left side code
void leftRush(){
	intaking = true;
	chassis.setPose({-13, -50, -90});
	chassis.moveToPoint(-48, -48, 1000);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(-48, -58.5, 1000, {}, false);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.brake();
	leftMotors.brake();
	chassis.moveToPoint(-47, -30, 1000, {.forwards = false}, false);
	hood.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(-49, -48, 1000);
	chassis.moveToPoint(-57, -26, 1000, {.forwards = false});
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(-57, -13, 1000, {.forwards = false});
	chassis.turnToHeading(195, 700, {}, false);
	rightMotors.brake();
	leftMotors.brake();
	

	/*
	chassis.setPose({-17.2, -51.2, 0});
	hood.set_value(HIGH);
	descore.set_value(HIGH);
	intaking = true;
	chassis.moveToPoint(-20, -26, 1000, {.maxSpeed = 80});
	chassis.waitUntil(11);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(-45, 700, {}, false);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-38, -12, 1200, {}, false);
	matchLoader.set_value(HIGH);
	pros::delay(200);
	chassis.moveToPoint(-20, -26, 1000, {.forwards = false});
	chassis.moveToPoint(-48, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	matchLoader.set_value(LOW);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(-46, -25, 1000, {.forwards = false, .maxSpeed = 80}, false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	intaking = false;
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-46, -62, 1200, {.maxSpeed = 70}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	intaking = true;
	hood.set_value(HIGH);
	pros::delay(1200);
	chassis.moveToPoint(-47, -25, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-47, -36, 1000, {}, false);
	hood.set_value(HIGH);
	chassis.moveToPoint(-47, -25, 500, {.forwards = false, .minSpeed = 90} ,false);
	leftMotors.brake();
	rightMotors.brake();
	wheelLift.set_value(HIGH);
	*/
}

void leftSplit(){

	allowedXOff = 0;
	lemlib::setPose({-17, -49.5, 0});
	descore.set_value(HIGH);
	intaking = true;

	chassis.moveToPoint(-20, -30.5, 2000);
	chassis.moveToPoint(-22.5, -22.5, 2000, {}, false);
	intaking = false;
	middleScore.set_value(HIGH);

	chassis.turnToHeading(-134,700);
	chassis.moveToPoint(-10.5, -13.5, 2000, {.forwards = false}, false);
	hood.set_value(HIGH);
	intaking = true;
	middleGoal = true;
	pros::delay(2000);
	middleGoal = false;
	allowedXOff = 4;
	middleScore.set_value(LOW);
	chassis.moveToPoint(-47.6, -41, 2000, {.maxSpeed = 100});
	hood.set_value(LOW);
	matchLoader.set_value(HIGH);
	chassis.turnToPoint(-47.5, -58, 700);
	chassis.moveToPoint(-47.5, -55,1000, {} , false);
	chassis.moveToPoint(-47.5, -61.5,600, {} , false);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	rightMotors.brake();
	leftMotors.brake();
	chassis.moveToPoint(-47.5, -30.5, 2000, {.forwards = false, .maxSpeed = 80}, false);
	hood.set_value(HIGH);
	pros::delay(1800);
	chassis.moveToPoint(-47.5, -35, 2000, {.maxSpeed = 80}, false);
	chassis.turnToPoint(-15, -16, 1000, {.forwards = false});
	hood.set_value(LOW);
	middleScore.set_value(HIGH);
	chassis.moveToPoint(-15, -15, 2000, {.forwards = false}, false);
	chassis.moveToPoint(-13, -10.5, 2000, {.forwards = false}, false);
	rightMotors.brake();
	leftMotors.brake();

	/*
	chassis.moveToPoint(-47.5, -30.5, 2000, {.forwards = false, .minSpeed = 80}, false);
	rightMotors.brake();
	leftMotors.brake();
	*/

	/*
	//acceptedMCLError = 5;
	intaking = true;
	middleScore.set_value(HIGH);
	chassis.setPose({-5.7, -46, 270});
	chassis.moveToPoint(-46, -46, 2000, {});
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 700, {}, false);
	chassis.moveToPoint(-48.5, -63, 1000, {.maxSpeed = 90}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(300);
	chassis.moveToPoint(-47.5, -26, 1000, {.forwards = false, .maxSpeed = 90}, false);
	matchLoader.set_value(LOW);
	hood.set_value(HIGH);
	intaking = false;
	outtaking = true;
	pros::delay(150);
	outtaking = false;
	intaking = true;
	pros::delay(1500);
	intaking = false;
	outtaking = true;
	//acceptedMCLError = 0;
	chassis.moveToPoint(-47.5, -46, 1000, {}, false);
	hood.set_value(LOW);
	chassis.moveToPoint(-26, -22, 2000, {.maxSpeed = 80});
	chassis.waitUntil(24);
	matchLoader.set_value(HIGH);
	outtaking = false;
	intaking = true;
	pros::delay(1000);
	chassis.turnToPoint(-11, -7, 700, {.forwards = false});
	middleScore.set_value(LOW);
	hood.set_value(HIGH);
	intaking = false;
	chassis.moveToPoint(-15, -14.5, 1000, {.forwards = false, .maxSpeed = 50}, false);
	intaking = true;
	pros::delay(2000);
	*/
	/*
	chassis.setPose({-17.2, -51.2, 0});
	hood.set_value(HIGH);
	descore.set_value(HIGH);
	intaking = true;
	chassis.moveToPoint(-20, -26, 1000, {.maxSpeed = 80});
	chassis.waitUntil(11);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-8.5, -12, 1000, {.forwards = false}, false);
	chassis.turnToHeading(215, 700, {}, false);
	intaking=false;
	outtaking = true;
	pros::delay(200);
	outtaking = false;
	middleGoal = true;
	pros::delay(1000);
	middleGoal = false;
	intaking = true;
	chassis.moveToPoint(-41, -44, 2000);
	chassis.moveToPoint(-48, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	matchLoader.set_value(LOW);
	chassis.turnToHeading(180, 700);
	matchLoader.set_value(LOW);
	intaking = false;
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-48, -63, 1400, {.maxSpeed = 80}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	intaking = true;
	hood.set_value(HIGH);
	pros::delay(1200);
	chassis.moveToPoint(-47, -25, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-47, -36, 1000, {}, false);
	hood.set_value(HIGH);
	chassis.moveToPoint(-47, -25, 500, {.forwards = false, .minSpeed = 90} ,false);
	leftMotors.brake();
	rightMotors.brake();
	*/
}

void leftMiddle(){
	chassis.setPose({-17.2, -51.2, 0});
	hood.set_value(HIGH);
	descore.set_value(HIGH);
	intaking = true;
	chassis.moveToPoint(-20, -26, 1000, {.maxSpeed = 80});
	chassis.waitUntil(11);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(-45, 700, {}, false);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-40, -12, 2000, {}, false);
	matchLoader.set_value(HIGH);
	pros::delay(200);
	chassis.moveToPoint(-20, -26, 1000, {.forwards = false});
	chassis.moveToPoint(-48, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	matchLoader.set_value(LOW);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(-46, -25, 1000, {.forwards = false, .maxSpeed = 80}, false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	intaking = false;
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-46, -62, 1200, {.maxSpeed = 70}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	intaking = true;
	hood.set_value(HIGH);
	pros::delay(1200);
	chassis.moveToPoint(-47, -25, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-47, -36, 1000, {}, false);
	hood.set_value(HIGH);
	chassis.moveToPoint(-47, -25, 500, {.forwards = false, .minSpeed = 90} ,false);
	leftMotors.brake();
	rightMotors.brake();
	wheelLift.set_value(HIGH);
}

// Solo Atonomous Win Point Code
void SAWP(){
	//chassis.setPose({24, -48, 0});
	chassis.moveToPoint(0, 5, 1000);
	/*
	acceptedMCLError = 5;
	intaking = true;
	chassis.setPose({5.7, -46, -90});
	chassis.moveToPoint(47, -46, 2000, {.forwards = false});
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(46.5, -58, 1000, {}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1500);
	chassis.moveToPoint(47.5, -26, 1000, {.forwards = false, .maxSpeed = 90}, false);
	matchLoader.set_value(LOW);
	hood.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(47.5, -46, 1000);
	chassis.moveToPoint(38, -26, 1000, {.forwards = false});
	chassis.moveToPoint(38, -10, 1000, {.forwards = false}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	*/

	//outtaking = true;
	//chassis.moveToPoint(0, 6, 10000, {.minSpeed = 127});
	/*
    chassis.setPose({19.6, -47.8, 89});
	hood.set_value(HIGH);
	intaking = true;
    chassis.moveToPoint(48, -48, 1000);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 400);
	chassis.moveToPoint(48, -57,900, {.minSpeed = 60}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(200);
	chassis.moveToPoint(47, -22, 1000, {.forwards = false}, false);
	hood.set_value(LOW);            
	matchLoader.set_value(LOW);
	pros::delay(800);
	chassis.moveToPoint(48, -38, 1000);
	chassis.moveToPoint(24, -22, 2000);
	hood.set_value(HIGH);
	chassis.waitUntil(28);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-23, -26, 2000);
	chassis.waitUntil(4);
	matchLoader.set_value(LOW);
	chassis.waitUntil(25);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-7.5, -13, 1000, {.forwards = false}, false);
	intaking=false;
	outtaking = true;
	pros::delay(200);
	outtaking = false;
	middleGoal = true;
	pros::delay(900);
	middleGoal = false;
	intaking = true;
	chassis.moveToPoint(-41, -48, 2000);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(-42, -63, 1200, {}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(200);
	chassis.moveToPoint(-43, -26, 1000, {.forwards = false}, false);
	hood.set_value(LOW);
	matchLoader.set_value(LOW);
	pros::delay(1000);
	*/
	



}