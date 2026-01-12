#include "geometry/controlvector.hpp"// IWYU pragma: keep
#include "geometry/pose.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/timer.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "math/quinticpolynomial.hpp"// IWYU pragma: keep
#include "physicalmodel/physicalmodel.hpp"// IWYU pragma: keep
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
#include "squiggles.hpp"// IWYU pragma: keep

//ASSET(skills_txt);

//lemlib_tarball::Decoder skillsFollow(skills_txt);

void ramsete(std::vector<squiggles::Pose> points, float timeout, bool async=false, bool forwards = true){
    for (int i = 0; i < points.size(); i++){
        points[i].x = points[i].x*0.0254;
        points[i].y = points[i].y*0.0254;
        points[i].yaw = points[i].yaw*(std::numbers::pi/180);
    }
    chassis.ramsete(points, timeout, async, forwards);

}

// Skills code
void skills(){
	hood.set_value(HIGH);
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
	/*
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
	*/
	chassis.turnToHeading(90, 700);
	chassis.moveToPoint(48, 48, 4000, {.maxSpeed = 80});
	chassis.turnToHeading(0, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(45.5, 64, 1000, {.maxSpeed = 70}, false);
	acceptedMCLError = 4;
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


}


// Right side long goal only code
void redRightLong(){
    
	chassis.setPose({17.2, -51.2, 0});
	hood.set_value(HIGH);
	intaking = true;
	chassis.moveToPoint(21, -28, 1000, {.maxSpeed = 80});
	chassis.waitUntil(11);
	matchLoader.set_value(HIGH);
	//chassis.turnToHeading(45, 700, {}, false);
	/*
	chassis.moveToPoint(40, -12, 2000, {});
	chassis.waitUntil(16);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(20, -26, 1000, {.forwards = false});
	*/
	chassis.moveToPoint(48, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	chassis.turnToHeading(180, 700);
	/*
	chassis.moveToPoint(48.5, -25, 1000, {.forwards = false, .maxSpeed = 80}, false);
	hood.set_value(LOW);
	pros::delay(1200);
	matchLoader.set_value(LOW);
	intaking = false;
	matchLoader.set_value(HIGH);
	*/
	chassis.moveToPoint(47, -62, 1400, {.maxSpeed = 60}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	intaking = true;
	hood.set_value(HIGH);
	pros::delay(200);
	chassis.moveToPoint(48, -25, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	hood.set_value(LOW);
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
	hood.set_value(HIGH);
	chassis.moveToPoint(41, -30, 1000, {.forwards = false});
	chassis.moveToPoint(40, -8, 2000, {.forwards = false}, false);
	chassis.turnToHeading(170, 700, {}, false);
	leftMotors.brake();
	rightMotors.brake();



}


// Right side normal code
void redRight(){

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

    

}

// Left side code
void redLeft(){
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
}

void leftSplit(){
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
void redAWP(){
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

	



}