// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <cmath>
#include <math.h>
#include "pros/rtos.hpp"
#include "lemlib/util.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

// tracking thread
pros::Task* trackingTask = nullptr;

// global variables
lemlib::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr); // the sensors to be used for odometry
lemlib::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0); // the drivetrain to be used for odometry
lemlib::MCLSensors mclLocal(nullptr, nullptr, nullptr, nullptr, false, false, 0, 0, 0, 0, 0, 0, 0, 0);
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot

// Global variables for constant distance sensor reset
float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;

// Defines drivetrains sensors for use with odometry locally
void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

// Defines distance sensors for use with distance reset locally
void lemlib::setMCL(lemlib::MCLSensors* mclsensors){
    // Distance sensors
    mclLocal.verticalDistance1 = mclsensors->verticalDistance1;
    mclLocal.verticalDistance2 = mclsensors->verticalDistance2;
    mclLocal.horizontalDistance1 = mclsensors->horizontalDistance1;
    mclLocal.horizontalDistance2 = mclsensors->horizontalDistance2;

    // Booleans
    mclLocal.verticalForwards = mclsensors->verticalForwards;
    mclLocal.horizontalRight = mclsensors->horizontalRight;

    // Offsets
    mclLocal.vert1HorizontalOffset = mclsensors->vert1HorizontalOffset;
    mclLocal.vert2HorizontalOffset = mclsensors->vert2HorizontalOffset;
    mclLocal.vert1VerticalOffset   = mclsensors->vert1VerticalOffset;
    mclLocal.vert2VerticalOffset   = mclsensors->vert2VerticalOffset;

    mclLocal.horiz1HorizontalOffset = mclsensors->horiz1HorizontalOffset;
    mclLocal.horiz2HorizontalOffset = mclsensors->horiz2HorizontalOffset;
    mclLocal.horiz1VerticalOffset   = mclsensors->horiz1VerticalOffset;
    mclLocal.horiz2VerticalOffset   = mclsensors->horiz2VerticalOffset;
}

// Function to get the robot's position on feild
lemlib::Pose lemlib::getPose(bool radians) {
    if (radians) return odomPose;
    else return lemlib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}


// Function to set the robots starting position
void lemlib::setPose(lemlib::Pose pose, bool radians) {
    if (radians) odomPose = pose;
    else odomPose = lemlib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

// Function to get the robot's speed
lemlib::Pose lemlib::getSpeed(bool radians) {
    if (radians) return odomSpeed;
    else return lemlib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

// Function to get the robot's local speed
lemlib::Pose lemlib::getLocalSpeed(bool radians) {
    if (radians) return odomLocalSpeed;
    else return lemlib::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.theta));
}

// Function to estimate the position of the robot based on previous readings, time, and speed
lemlib::Pose lemlib::estimatePose(float time, bool radians) {
    // get current position and speed
    Pose curPose = getPose(true);
    Pose localSpeed = getLocalSpeed(true);
    // calculate the change in local position
    Pose deltaLocalPose = localSpeed * time;

    // calculate the future pose
    float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
    Pose futurePose = curPose;
    futurePose.x += deltaLocalPose.y * sin(avgHeading);
    futurePose.y += deltaLocalPose.y * cos(avgHeading);
    futurePose.x += deltaLocalPose.x * -cos(avgHeading);
    futurePose.y += deltaLocalPose.x * sin(avgHeading);
    if (!radians) futurePose.theta = radToDeg(futurePose.theta);

    return futurePose;
}

// Function used in distance reset code to calculate the difference of the distance sensors if they are contacting the wall correctly
float predictVertical(float theta, float rightDistance, float leftDistance){
    theta = lemlib::sanitizeAngle(theta, false);
    float a = fmod(theta, 360.0f);
    if (a < 0) a += 360.0f;

    // Nearest 90 degree multiple (0, 90, 180, 270, 360)
    float nearest90 = round(a / 90.0f) * 90.0f;

    // Distance from that angle (can be negative)
    float diff = a - nearest90;

    float distance = (tanf(lemlib::degToRad(diff)))*(mclLocal.vert1HorizontalOffset-mclLocal.vert2HorizontalOffset);
    return distance;
}

// Function used in distance reset code to calculate the difference of the distance sensors if they are contacting the wall correctly
float predictHorizontal(float theta, float rightDistance, float leftDistance){
    theta = lemlib::sanitizeAngle(theta, false);
    float a = fmod(theta, 360.0f);
    if (a < 0) a += 360.0f;

    // Nearest 90 degree multiple (0, 90, 180, 270, 360)
    float nearest90 = round(a / 90.0f) * 90.0f;

    // Distance from that angle (can be negative)
    float diff = a - nearest90;

    float distance = (tanf(lemlib::degToRad(diff)))*(mclLocal.horiz1HorizontalOffset-mclLocal.horiz2HorizontalOffset);


    return distance;
}

// Function that updates the position of the robot constantly
void lemlib::update() {
    // Get the current sensor values
    float centerToWall = 0;
    float vertical1Raw = 0;
    float vertical2Raw = 0;
    float horizontal1Raw = 0;
    float horizontal2Raw = 0;
    float imuRaw = 0;
    float verticalDis1 = 0;
    float verticalDis2 = 0;
    float horizontalDis1 = 0;
    float horizontalDis2 = 0;
    float verticalOffsetTheoretical = 0;
    float horizontalOffsetTheoretical = 0;
    float mclX = 100000;
    float mclY = 100000;
    if (odomSensors.vertical1 != nullptr) vertical1Raw = odomSensors.vertical1->getDistanceTraveled();
    if (odomSensors.vertical2 != nullptr) vertical2Raw = odomSensors.vertical2->getDistanceTraveled();
    if (odomSensors.horizontal1 != nullptr) horizontal1Raw = odomSensors.horizontal1->getDistanceTraveled();
    if (odomSensors.horizontal2 != nullptr) horizontal2Raw = odomSensors.horizontal2->getDistanceTraveled();
    if (odomSensors.imu != nullptr) imuRaw = degToRad(odomSensors.imu->get_rotation());
    if (mclLocal.verticalDistance1 != nullptr) verticalDis1 = mclLocal.verticalDistance1->get_distance()/25.4;
    if (mclLocal.verticalDistance2 != nullptr) verticalDis2 = mclLocal.verticalDistance2->get_distance()/25.4;
    if (mclLocal.horizontalDistance1 != nullptr) horizontalDis1 = mclLocal.horizontalDistance1->get_distance()/25.4;
    if (mclLocal.horizontalDistance2 != nullptr) horizontalDis2 = mclLocal.horizontalDistance2->get_distance()/25.4;

    // calculate the change in sensor values
    float deltaVertical1 = vertical1Raw - prevVertical1;
    float deltaVertical2 = vertical2Raw - prevVertical2;
    float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
    float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
    float deltaImu = imuRaw - prevImu;

    // update the previous sensor values
    prevVertical1 = vertical1Raw;
    prevVertical2 = vertical2Raw;
    prevHorizontal1 = horizontal1Raw;
    prevHorizontal2 = horizontal2Raw;
    prevImu = imuRaw;

    // calculate the heading of the robot
    // Priority:
    // 1. Horizontal tracking wheels
    // 2. Vertical tracking wheels
    // 3. Inertial Sensor
    // 4. Drivetrain
    float heading = odomPose.theta;
    // calculate the heading using the horizontal tracking wheels
    if (odomSensors.horizontal1 != nullptr && odomSensors.horizontal2 != nullptr)
        heading -= (deltaHorizontal1 - deltaHorizontal2) /
                   (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
    // else, if both vertical tracking wheels aren't substituted by the drivetrain, use the vertical tracking wheels
    else if (!odomSensors.vertical1->getType() && !odomSensors.vertical2->getType())
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    // else, if the inertial sensor exists, use it
    else if (odomSensors.imu != nullptr) heading += deltaImu;
    // else, use the the substituted tracking wheels
    else
        heading -= (deltaVertical1 - deltaVertical2) /
                   (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
    float deltaHeading = heading - odomPose.theta;
    float avgHeading = odomPose.theta + deltaHeading / 2;

    // Choose tracking wheels to use
    // Prioritize non-powered tracking wheels
    lemlib::TrackingWheel* verticalWheel = nullptr;
    lemlib::TrackingWheel* horizontalWheel = nullptr;
    if (!odomSensors.vertical1->getType()) verticalWheel = odomSensors.vertical1;
    else if (!odomSensors.vertical2->getType()) verticalWheel = odomSensors.vertical2;
    else verticalWheel = odomSensors.vertical1;
    if (odomSensors.horizontal1 != nullptr) horizontalWheel = odomSensors.horizontal1;
    else if (odomSensors.horizontal2 != nullptr) horizontalWheel = odomSensors.horizontal2;
    float rawVertical = 0;
    float rawHorizontal = 0;
    if (verticalWheel != nullptr) rawVertical = verticalWheel->getDistanceTraveled();
    if (horizontalWheel != nullptr) rawHorizontal = horizontalWheel->getDistanceTraveled();
    float horizontalOffset = 0;
    float verticalOffset = 0;
    if (verticalWheel != nullptr) verticalOffset = verticalWheel->getOffset();
    if (horizontalWheel != nullptr) horizontalOffset = horizontalWheel->getOffset();

    // calculate change in x and y
    float deltaX = 0;
    float deltaY = 0;
    if (verticalWheel != nullptr) deltaY = rawVertical - prevVertical;
    if (horizontalWheel != nullptr) deltaX = rawHorizontal - prevHorizontal;
    prevVertical = rawVertical;
    prevHorizontal = rawHorizontal;

    // calculate local x and y
    float localX = 0;
    float localY = 0;
    if (deltaHeading == 0) { // prevent divide by 0
        localX = deltaX;
        localY = deltaY;
    } else {
        localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
        localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
    }

    // START OF CONSTANT DISTANCE RESET CODE

    // Checks if the distance sensor readings are within a value that is consistantly accurate with all distance sensors
    if(mclLocal.verticalDistance1 != nullptr && (verticalDis1 < 35 || verticalDis2 < 35)){
        // Calculates the difference of the two distance sensors given we are getting valid readings
        verticalOffsetTheoretical = predictVertical(radToDeg(imuRaw), verticalDis1, verticalDis2);
        // Checks if the calculated value is within half an inch of the actual reading to verify we are getting a valid reading
        if (abs(verticalOffsetTheoretical-((verticalDis1+mclLocal.vert1VerticalOffset)-(verticalDis2+mclLocal.vert2VerticalOffset)))<0.5){
            // Calculates the distance from the smalll of the two distance readings to the wall at the center of the robot

            // NOTE: NEED TO FIX IF OPPOSITE SIDE THIS MIGHT BE OFF
            float scaledAddition = abs((verticalDis1+mclLocal.vert1VerticalOffset)-(verticalDis2+mclLocal.vert2VerticalOffset))
                *mclLocal.vert1HorizontalOffset/(mclLocal.vert1HorizontalOffset-mclLocal.vert2HorizontalOffset);
            // Computes total distance 
            if(verticalDis1< verticalDis2){
                centerToWall = scaledAddition + verticalDis1 + mclLocal.vert1VerticalOffset;
            } else {
                centerToWall = scaledAddition + verticalDis2 + mclLocal.vert2VerticalOffset;
            }
        float deg = radToDeg(imuRaw);
        deg = fmod(deg, 360.0);
        if (deg < 0) deg += 360.0;

        // Compute nearest multiple of 90
        int nearest = static_cast<int>(std::round(deg / 90.0)) * 90;

        // Wrap 360 back to 0
        if (nearest == 360) nearest = 0;
        switch (nearest){
            case 0:
                if (mclLocal.verticalForwards){
                    mclY = 70.2 - centerToWall;
                } else {
                    mclY = -70.2 + centerToWall;
                }
                break;
            case 90:
                if (mclLocal.verticalForwards){
                    mclX = 70.2 - centerToWall;
                } else {
                    mclX = -70.2 + centerToWall;
                }
                break;
            case 180:
                if (mclLocal.verticalForwards){
                    mclY = -70.2 + centerToWall;
                } else {
                    mclY = 70.2 - centerToWall;
                }
                break;
            case 270:
                if (mclLocal.verticalForwards){
                    mclX = -70.2 + centerToWall;
                } else {
                    mclX = 70.2 - centerToWall;
                }
                break;
        }
        }
    }
    
    // Checks if the distance sensor readings are within a value that is consistantly accurate with all distance sensors
    if(mclLocal.horizontalDistance1 != nullptr && (horizontalDis1 < 35 || horizontalDis2 < 35)){
        horizontalOffsetTheoretical = predictHorizontal(radToDeg(imuRaw), horizontalDis1, horizontalDis2);
        if (abs(horizontalOffsetTheoretical-((horizontalDis1+mclLocal.horiz1VerticalOffset)-(horizontalDis2+mclLocal.horiz2VerticalOffset)))<0.5){
            float x = abs((horizontalDis1+mclLocal.horiz1VerticalOffset)-(horizontalDis2+mclLocal.horiz2VerticalOffset))
                *mclLocal.horiz1HorizontalOffset/(mclLocal.horiz1HorizontalOffset-mclLocal.horiz2HorizontalOffset);
            if(horizontalDis1< horizontalDis2){
                centerToWall = x + horizontalDis1 + mclLocal.horiz1VerticalOffset;
            } else {
                centerToWall = x + horizontalDis2 + mclLocal.horiz2VerticalOffset;
            }
        float deg = radToDeg(imuRaw);
        deg = fmod(deg, 360.0);
        if (deg < 0) deg += 360.0;

        // Compute nearest multiple of 90
        int nearest = static_cast<int>(std::round(deg / 90.0)) * 90;

        // Wrap 360 back to 0
        if (nearest == 360) nearest = 0;
        switch (nearest){
            case 0:
                if (mclLocal.horizontalRight){
                    mclX = 70.2 - centerToWall;
                } else {
                    mclX = -70.2 + centerToWall;
                }
                break;
            case 90:
                if (mclLocal.horizontalRight){
                    mclY = 70.2 - centerToWall;
                } else {
                    mclY = -70.2 + centerToWall;
                }
                break;
            case 180:
                if (mclLocal.horizontalRight){
                    mclX = -70.2 + centerToWall;
                } else {
                    mclX = 70.2 - centerToWall;
                }
                break;
            case 270:
                if (mclLocal.horizontalRight){
                    mclY = -70.2 + centerToWall;
                } else {
                    mclY = 70.2 - centerToWall;
                }
                break;
            }
        }
    }

    // END CONSTANT DISTANCE RESET CODE 


    // save previous pose
    lemlib::Pose prevPose = odomPose;

    // calculate global x and y
    if (mclX == 100000){
        odomPose.x += localY * sin(avgHeading);
        odomPose.x += localX * -cos(avgHeading);
    } else {
        odomPose.x = mclX;
    }
    if (mclY == 100000){
        odomPose.y += localY * cos(avgHeading);
        odomPose.y += localX * sin(avgHeading);
    } else {
        odomPose.y = mclY;
    }
    odomPose.theta = heading;

    // calculate speed
    odomSpeed.x = ema((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
    odomSpeed.y = ema((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
    odomSpeed.theta = ema((odomPose.theta - prevPose.theta) / 0.01, odomSpeed.theta, 0.95);

    // calculate local speed
    odomLocalSpeed.x = ema(localX / 0.01, odomLocalSpeed.x, 0.95);
    odomLocalSpeed.y = ema(localY / 0.01, odomLocalSpeed.y, 0.95);
    odomLocalSpeed.theta = ema(deltaHeading / 0.01, odomLocalSpeed.theta, 0.95);
}

void lemlib::init() {
    if (trackingTask == nullptr) {
        trackingTask = new pros::Task {[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}
