#include "api.h"
#include "squiggles/geometry/profilepoint.hpp"
#include "squiggles/spline.hpp"
#include "squiggles/squiggles.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include <cmath>


void lemlib::Chassis::ramsete(std::vector<squiggles::Pose> points, float timeout, bool async, bool forwards){
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() {ramsete(points, timeout, false, forwards);});
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    Timer timer(timeout);

    squiggles::Constraints constraints(1.9, 10, 10);
    squiggles::TankModel model(0.28, constraints);
    squiggles::SplineGenerator drive(constraints, std::make_shared<squiggles::TankModel>(1, constraints));
    auto generatedPoints = drive.generate(points);

    lemlib::Pose currentpose = getPose(true);

    while(!timer.isDone() && !(fabs(currentpose.x/39.3701-points.back().x)<0.02 && fabs(currentpose.y/39.3701-points.back().y)<0.02 && fabs(currentpose.theta-points.back().yaw)<0.05)){
        currentpose = getPose(true);
        //squiggles::ProfilePoint goal = generatedPoints[ceil(timer.getTimePassed()/0.1)];
        squiggles::ProfilePoint goal = drive.get_point_at_time(points[0], points.back(), generatedPoints, timer.getTimePassed());
        
        // Adjust heading for backwards driving
        float theta = currentpose.theta;
        if (!forwards) {
            theta += std::numbers::pi;
            // Normalize to [-pi, pi]
            if (theta > std::numbers::pi) theta -= 2 * std::numbers::pi;
            if (theta < -std::numbers::pi) theta += 2 * std::numbers::pi;
        }
        
        float dx = goal.vector.pose.x - currentpose.x/39.3701;
        float dy = goal.vector.pose.y - currentpose.y/39.3701;
        float ex = cosf(theta)*dx+sinf(theta)*dy;
        float ey = -1*sinf(theta)*dx+cosf(theta)*dy;
        float etheta = goal.vector.pose.yaw-theta;
        // Normalize angle error to [-pi, pi]
        while (etheta > std::numbers::pi) etheta -= 2 * std::numbers::pi;
        while (etheta < -std::numbers::pi) etheta += 2 * std::numbers::pi;
        
        float wd = goal.vector.vel*goal.curvature;
        float vd = goal.vector.vel;
        float k = 2*0.7*sqrtf(powf(wd, 2)+2*powf(vd, 2));
        float v = vd*cosf(etheta)+k*ex;
        float sinc_etheta = (fabs(etheta) < 1e-6) ? 1.0f : sinf(etheta) / etheta;
        float w = wd+k*etheta+5*vd*sinc_etheta*ey;
        //float rps = w/(2*std::numbers::pi);
        //float inps = rps*(3.25*std::numbers::pi);
        //float mps = inps*39.3701;
        float percentageAngular = w/1.9;
        float percentageLinear = v/1.9;
        
        if (forwards){
            //drivetrain.leftMotors -> move(percentageLinear*127 + percentageAngular*127);
            //drivetrain.rightMotors -> move(percentageLinear*127 - percentageAngular*127);
            drivetrain.leftMotors -> move_velocity(percentageLinear*600 + percentageAngular*600);
            drivetrain.leftMotors -> move_velocity(percentageLinear*600 - percentageAngular*600);
        }else {
            // Negate linear velocity, keep angular correction direction
            // drivetrain.leftMotors -> move(-percentageLinear*127 + percentageAngular*127);
            // drivetrain.rightMotors -> move(-percentageLinear*127 - percentageAngular*127);
            drivetrain.leftMotors -> move_velocity(-percentageLinear*600 + percentageAngular*600);
            drivetrain.leftMotors -> move_velocity(-percentageLinear*600 - percentageAngular*600);
        }

        pros::delay(10);
    }

    // stop the drivetrain
    drivetrain.leftMotors->move(0);
    drivetrain.rightMotors->move(0);
    // set distTraveled to -1 to indicate that the function has finished
    distTraveled = -1;
    this->endMotion();
}
