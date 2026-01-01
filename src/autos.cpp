#include "geometry/controlvector.hpp"// IWYU pragma: keep
#include "geometry/pose.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
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

ASSET(skills_txt);

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


    chassis.setPose({48, 8, 0});
	chassis.moveToPoint(47, 44, 4000, {.maxSpeed = 50});
	chassis.turnToHeading(90, 900);
	matchLoader.set_value(HIGH);
	intaking = true;
	descore.set_value(HIGH);
	chassis.moveToPoint(58.5, 46.2, 2000, {.maxSpeed = 80}, false);
	chassis.turnToHeading(90, 900);
	pros::delay(3000);
	intaking=false;
	matchLoader.set_value(LOW);
    //chassis.follow(skillsFollow["firstscore"], 8, 7000, false);
	chassis.turnToHeading(-90, 900);
	chassis.moveToPoint(-25, 43, 2000, {.forwards = false, .maxSpeed = 50}, false);
	scoring = true;
	pros::delay(2000);
	scoring =false;
	intaking = true;
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-50.5, 43.5, 2000, {.maxSpeed = 80}, false);
	
	pros::delay(3000);
	intaking = false;
	chassis.moveToPoint(-23, 43, 2000, {.forwards = false, .maxSpeed = 50}, false);
	scoring = true;
	pros::delay(2500);
	scoring =false;
	intaking = true;
	matchLoader.set_value(LOW);
	//chassis.follow(skillsFollow["navigate"], 17, 7000);
	chassis.moveToPoint(-36, 45.5, 4000);
	chassis.turnToHeading(180, 900);
	chassis.moveToPoint(-36, -48, 4000, {.maxSpeed = 50});
	chassis.turnToHeading(-90, 900, {.maxSpeed = 70});
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(-55.5, -48, 2000, {.maxSpeed = 80}, false);
	pros::delay(3000);
	matchLoader.set_value(LOW);
	intaking=false;
	//chassis.follow(skillsFollow["navigate 2"], 10, 7000, false);
	chassis.turnToHeading(90, 900);
	chassis.moveToPoint(27, -46, 2000, {.forwards = false, .maxSpeed = 80}, false);
	scoring = true;
	pros::delay(2000);
	scoring =false;
	intaking = true;
	matchLoader.set_value(HIGH);
	chassis.moveToPoint(58, -47, 2000, {.maxSpeed = 80});

	pros::delay(3000);
	intaking = false;
	matchLoader.set_value(LOW);
	chassis.moveToPoint(27, -47, 2000, {.forwards = false, .maxSpeed = 80}, false);
	scoring = true;
	pros::delay(2500);
	scoring =false;
	intaking = true;
	chassis.moveToPoint(37, -46, 2000);
	chassis.turnToHeading(180, 900);
	chassis.moveToPoint(37, 0, 2000, {.forwards = false});
	chassis.turnToHeading(-90, 900);
	wheelLift.set_value(HIGH);
	chassis.moveToPoint(56, -0, 4000, {.forwards = false});
	

}


// Right side long goal only code
void redRightLong(){
    
	ramsete({squiggles::Pose(0, 0, 0), squiggles::Pose(0, 24, 0)}, 4000 );


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
