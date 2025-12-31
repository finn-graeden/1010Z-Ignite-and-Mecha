#include "api.h"
#include "geometry/profilepoint.hpp"
#include "spline.hpp"
#include "squiggles.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include <cmath>


void lemlib::Chassis::ramsete(std::vector<squiggles::Pose> points, float timeout, bool async, bool forwards){
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() {ramsete(points, async);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    Timer timer(timeout);

    squiggles::Constraints constraints(1.9, 10, 10);
    squiggles::TankModel model(10, constraints);
    squiggles::SplineGenerator drive(constraints, std::make_shared<squiggles::TankModel>(1, constraints));
    auto generatedPoints = drive.generate(points);

    lemlib::Pose currentpose = getPose(true);

    while(!timer.isDone() || (fabs(currentpose.x-points[0].x)<0.2 && fabs(currentpose.y-points[0].y)<0.2 && fabs(currentpose.theta-points[0].yaw))){
        currentpose = getPose(true);
        //squiggles::ProfilePoint goal = generatedPoints[ceil(timer.getTimePassed()/0.1)];
        squiggles::ProfilePoint goal = drive.get_point_at_time(points[0], points.back(), generatedPoints, timer.getTimePassed());
        float ex = cosf(currentpose.theta)*(goal.vector.pose.x/39.3701-currentpose.x)+sinf(currentpose.theta)*(goal.vector.pose.y/39.3701-currentpose.y);
        float ey = -1*sinf(currentpose.theta)*(goal.vector.pose.x/39.3701-currentpose.x)+cosf(currentpose.theta)*(goal.vector.pose.y/39.3701-currentpose.y);
        float etheta = goal.vector.pose.yaw-currentpose.theta;
        float wd = goal.vector.vel*goal.curvature;
        float vd = goal.vector.vel;
        float k = 2*0.7*sqrtf(powf(wd, 2)+5*powf(vd, 2));
        float v = vd*cosf(etheta)+k*ex;
        float w = wd+k*etheta+((5*wd*sinf(etheta)*ey)/(etheta));
        float rps = w/(2*std::numbers::pi);
        float inps = rps*(3.25*std::numbers::pi);
        float mps = inps*39.3701;
        float percentageAngular = mps/1.9;
        float percentageLinear = v/1.9;
        if (forwards){
            drivetrain.leftMotors -> move(percentageLinear*127 + percentageAngular*127);
            drivetrain.rightMotors -> move(percentageLinear*127 - percentageAngular*127);
        }else {
            drivetrain.leftMotors -> move(-percentageLinear*127 - percentageAngular*127);
            drivetrain.rightMotors -> move(-percentageLinear*127 + percentageAngular*127);
        }

        
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}