#include "geometry/controlvector.hpp"// IWYU pragma: keep
#include "geometry/pose.hpp"
#include "lemlib/chassis/odom.hpp"
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
	isSkills = false;
	redTeam = true;
	intaking = true;
	chassis.moveToPoint(-24, -33, 2000);
	chassis.turnToPoint(-12, -15, 700, {.forwards = false});
	chassis.moveToPoint(-8.5, -11, 1000, {.forwards = false}, false);
	hood.set_value(LOW);
	intaking = false;
	outtaking = true;
	pros::delay(400);
	outtaking = false;
	middleGoal = true;
	pros::delay(2000);
	middleGoal = false;
	hood.set_value(HIGH);
	isSkills = true;
	intaking = true;
	chassis.moveToPoint(-48, -48, 2000);
	chassis.turnToHeading(180, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-49, -62, 1000, {}, false);
	leftMotors.set_brake_mode(pros::MotorBrake::hold);
	rightMotors.set_brake_mode(pros::MotorBrake::hold);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1700);
	chassis.moveToPoint(-61, -24, 2000, {.forwards = false});
	matchLoader.set_value(LOW);
	descore.set_value(HIGH);
	chassis.moveToPoint(-61, 44, 3000, {.forwards = false});
	chassis.turnToPoint(-48, 36, 700, {.forwards = false});
	chassis.moveToPoint(-48, 36, 1000, {.forwards = false}, false);
	chassis.turnToPoint(-47, 27, 700, {.forwards = false});
	chassis.moveToPoint(-47, 24.5, 1000, {.forwards = false, .maxSpeed = 100}, false);
	hood.set_value(LOW);
	intaking = true;
	matchLoader.set_value(HIGH);
	pros::delay(2000);
	chassis.moveToPoint(-46.5, 64, 1200, {.maxSpeed = 80}, false);
	hood.set_value(HIGH);
	leftMotors.brake();
	rightMotors.brake();
	pros::delay(1900);
	intaking = false;
	hood.set_value(LOW);
	chassis.moveToPoint(-47, 25.5, 1000, {.forwards = false, .maxSpeed = 100}, false);
	intaking = true;
	pros::delay(2200);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(-46, 36, 1000, {}, false);
	hood.set_value(HIGH);
	chassis.turnToPoint(-24, 65, 700);
	chassis.moveToPoint(-32, 64, 1000);
	chassis.moveToPoint(-22, 67, 1000);
	wheelLift.set_value(HIGH);
	chassis.moveToPoint(10, 67, 5000, {.maxSpeed = 80});
	chassis.moveToPoint(34, 48, 2000);
	chassis.turnToHeading(-135, 700);
	chassis.moveToPoint(24, 24, 1000);







}


// Right side long goal only code
void redRightLong(){
    
	//chassis.follow(skillsFollow["firstscore"], 8, 7000, false);


}


// Right side normal code
void redRight(){
    
    chassis.setPose({17.2, -51.2, 0});
	intaking = true;
	chassis.moveToPoint(20, -26, 1000, {.maxSpeed = 80});
	chassis.waitUntil(12);
	matchLoader.set_value(HIGH);
	chassis.turnToHeading(-45, 700, {}, false);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(17.5, -19.5, 2000, {.maxSpeed = 50}, false);
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
	hood.set_value(HIGH);
	pros::delay(2200);
	hood.set_value(LOW);
	matchLoader.set_value(LOW);
	intaking = false;
	chassis.moveToPoint(45, -43, 2000);
	chassis.moveToPoint(38, -30, 2000, {.forwards = false});
	chassis.turnToPoint(38, -11, 700, {.forwards = false});
	chassis.moveToPoint(37.75, -11, 2000, {.forwards = false, .maxSpeed = 80});

    

}

// Left side code
void redLeft(){
	//ramsete({{0, 0, 0}, {10, 10, 0}}, 6000);
}

// Solo Atonomous Win Point Code
void redAWP(){
    chassis.setPose({0, 0, -90});
    descore.set_value(HIGH);
	intaking = true;
    chassis.moveToPoint(-4, 0, 1000);
    chassis.moveToPoint(48, 0, 2000, {.forwards = false});
    chassis.turnToHeading(-180, 700);
    matchLoader.set_value(HIGH);
    chassis.moveToPoint(48, -10.5, 1000);
    pros::delay(1100);
    chassis.moveToPoint(48, 26, 2000, {.forwards = false} );
    pros::delay(600);
    intaking=false;
    scoring = true;
    pros::delay(1400);
    scoring = false;
    intaking = true;
    matchLoader.set_value(LOW);
    chassis.moveToPoint(48, 0, 1000);
    chassis.turnToPoint(0, 48, 700, {.minSpeed = 30});
    chassis.moveToPoint(15, 31.5, 800, {}, false);
    intaking = true;
    chassis.moveToPoint(5, 24, 1000, {.forwards = false});
    chassis.turnToHeading(-90, 700);
    chassis.moveToPoint(-24, 24, 4000);
    pros::delay(500);
    matchLoader.set_value(HIGH);
    chassis.turnToPoint(-48, 12, 300);
    chassis.moveToPoint(-48, 12, 4000, {}, false);
    matchLoader.set_value(LOW);
    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(-48, 26, 800, {.forwards = false, .maxSpeed = 90}, false);
    intaking = false;
    scoring = true;
    


}
