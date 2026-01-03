#include "geometry/controlvector.hpp"// IWYU pragma: keep
#include "geometry/pose.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "math/quinticpolynomial.hpp"// IWYU pragma: keep
#include "physicalmodel/physicalmodel.hpp"// IWYU pragma: keep
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp" // IWYU pragma: keep
#include <cstdint> // IWYU pragma: keep
#include <numbers>
#include "lemlib/pose.hpp"
#include "lemlib-tarball/api.hpp" // IWYU pragma: keep
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

	chassis.setPose({17.2, -51.2, 0});
	intaking = true;
	chassis.moveToPoint(20, -26, 1000, {.maxSpeed = 80});
	chassis.turnToHeading(-45, 700);
	chassis.moveToPoint(14.5, -17.5, 2000, {.maxSpeed = 50}, false);
	intaking = false;
	outtaking = true;
	pros::delay(1500); // Change to 1000 later
	outtaking = false;
	chassis.moveToPoint(49, -51, 2000, {.forwards = false, .maxSpeed = 100}, false);
	chassis.turnToHeading(180, 700);
	chassis.moveToPoint(48, -60, 1000);
	intaking = true;
	pros::delay(1200);
	chassis.moveToPoint(49, -34, 1000, {.forwards = false, .maxSpeed = 80} ,false);
	pros::delay(1700);
	intaking = false;
	chassis.moveToPoint(45, -43, 2000);
	chassis.moveToPoint(35, -30, 2000, {.forwards = false});
	chassis.turnToHeading(180, 700);

    

}


// Right side long goal only code
void redRightLong(){
    
	//chassis.follow(skillsFollow["firstscore"], 8, 7000, false);


}


// Right side normal code
void redRight(){
    
    chassis.setPose({0, 0, 17});
	intaking = true;
	chassis.moveToPoint(5, 26, 4000, {.maxSpeed = 70});
	pros::delay(550);
	matchLoader.set_value(HIGH);
	chassis.swingToPoint(29, 48, DriveSide::RIGHT, 700);
	matchLoader.set_value(LOW);
	chassis.moveToPoint(24, 41, 4000, {}, false);
	chassis.swingToHeading(80, DriveSide::RIGHT, 300);
	matchLoader.set_value(HIGH);
	pros::delay(300);
	chassis.moveToPoint(5, 28, 4000, {.forwards = false});
	chassis.turnToPoint(0, 37, 700);
	matchLoader.set_value(LOW);	
	intaking = false;
	chassis.moveToPoint(-6, 34.5, 700, {}, false);
	outtaking = true;
	pros::delay(1000);
	outtaking = false;
	intaking = true;
	chassis.moveToPoint(36, 7, 4000, {.forwards = false});
	chassis.turnToHeading(180, 700);
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(33, -12, 1000, {.maxSpeed = 100}, false);
	pros::delay(2000);
    intaking = false;
	chassis.moveToPoint(34.5, 23, 1000, {.forwards= false, .maxSpeed = 100});
    outtaking = true;
    pros::delay(200);
    

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
