// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include <cmath>
#include <math.h>
#include "pros/misc.hpp"
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
lemlib::MCLSensors mclLocal(nullptr, 0, 0, nullptr, 0, 0, nullptr, 0, 0, nullptr, 0, 0);
lemlib::Pose odomPose(0, 0, 0); // the pose of the robot
lemlib::Pose odomSpeed(0, 0, 0); // the speed of the robot
lemlib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot
lemlib::Pose prevPose(0, 0, 0);

// Global variables for constant distance sensor reset
float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;

int numOfResets = 0;

int acceptedMCLError = 5;

// Defines drivetrains sensors for use with odometry locally
void lemlib::setSensors(lemlib::OdomSensors sensors, lemlib::Drivetrain drivetrain) {
    odomSensors = sensors;
    drive = drivetrain;
}

// Defines distance sensors for use with distance reset locally
void lemlib::setMCL(lemlib::MCLSensors mclsensors){
    mclLocal = mclsensors;
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

struct Point {
    double x, y;
};

// 2D Cross product of vectors (A-C) and (B-C)
double cross_product(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Function to find the intersection point of a ray (P + t*R) and a line segment (Q + u*S)
// Ray origin P, Ray direction R (normalized or not)
// Segment endpoints Q and E, vector S = E-Q
bool raySegmentIntersection(Point p, Point r_dir, Point q, Point e, Point& intersection) {
    Point s_dir = {e.x - q.x, e.y - q.y}; // Segment direction vector
    Point q_minus_p = {q.x - p.x, q.y - p.y}; // Vector from ray origin to segment start

    // Denominator for t and u (cross product of direction vectors)
    double denominator = r_dir.x * s_dir.y - r_dir.y * s_dir.x;

    // Check if lines are parallel (denominator is close to zero)
    if (std::abs(denominator) < 1e-9) {
        // Lines are parallel. May be collinear and overlapping, or just parallel.
        // For this simple case, we assume no intersection to avoid complex collinear checks.
        return false;
    }

    // Solve for parameters t (ray) and u (segment) using Cramer's rule or similar
    double t = (q_minus_p.x * s_dir.y - q_minus_p.y * s_dir.x) / denominator;
    double u = (q_minus_p.x * r_dir.y - q_minus_p.y * r_dir.x) / denominator;

    // Check if the intersection point falls within the constraints:
    // t >= 0 for the ray (starts at origin and goes infinitely in one direction)
    // 0 <= u <= 1 for the line segment (between endpoints)
    if (t >= 0.0 && u >= 0.0 && u <= 1.0) {
        // Intersection found
        intersection.x = p.x + t * r_dir.x;
        intersection.y = p.y + t * r_dir.y;
        return true;
    }

    // No intersection within the valid ranges
    return false;
}

Point pointAtDistance(Point startPoint, float distance, float heading){
    return Point{startPoint.x+(sinf(heading)*distance), startPoint.y+(cosf(heading)*distance)};
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
    float frontDis = 0;
    float rightDis = 0;
    float backDis = 0;
    float leftDis = 0;
    float verticalOffsetTheoretical = 0;
    float horizontalOffsetTheoretical = 0;
    float mclX = 100000;
    float mclY = 100000;
    if (odomSensors.vertical1 != nullptr) vertical1Raw = odomSensors.vertical1->getDistanceTraveled();
    if (odomSensors.vertical2 != nullptr) vertical2Raw = odomSensors.vertical2->getDistanceTraveled();
    if (odomSensors.horizontal1 != nullptr) horizontal1Raw = odomSensors.horizontal1->getDistanceTraveled();
    if (odomSensors.horizontal2 != nullptr) horizontal2Raw = odomSensors.horizontal2->getDistanceTraveled();
    if (odomSensors.imu != nullptr) imuRaw = degToRad(odomSensors.imu->get_rotation());
    if (mclLocal.frontDistance != nullptr) frontDis = mclLocal.frontDistance->get_distance()/25.4;
    if (mclLocal.rightDistance != nullptr) rightDis = mclLocal.rightDistance->get_distance()/25.4;
    if (mclLocal.backDistance != nullptr) backDis = mclLocal.backDistance->get_distance()/25.4;
    if (mclLocal.leftDistance != nullptr) leftDis = mclLocal.leftDistance->get_distance()/25.4;

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
    
    // Front distance sesnor (if available)
    if (mclLocal.frontDistance != nullptr && frontDis < 70){
        Point sensor = pointAtDistance({odomPose.x, odomPose.y}, sqrtf(powf(mclLocal.frontLatOff, 2)+powf(mclLocal.frontVertOff, 2)), odomPose.theta);
        Point intersection;
        if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (frontDis + mclLocal.frontVertOff) - (tanf(odomPose.theta)*mclLocal.frontLatOff);
                mclY = 70.2-centerToWall;
            }
        } else if (raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, -70.2}, intersection)){
            if (1){
                centerToWall = (frontDis + mclLocal.frontVertOff) - (tanf(odomPose.theta)*mclLocal.frontLatOff);
                mclX = 70.2-centerToWall;
            }
        } else if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (frontDis + mclLocal.frontVertOff) - (tanf(odomPose.theta)*mclLocal.frontLatOff);
                mclY = -70.2+centerToWall;
            }
        }else {
            if (1){
                centerToWall = (frontDis + mclLocal.frontVertOff) - (tanf(odomPose.theta)*mclLocal.frontLatOff);
                mclX = 70.2-centerToWall;
            }
        }

    }

    // Right distance sesnor (if available)
    if (mclLocal.rightDistance != nullptr && rightDis < 70){
        Point sensor = pointAtDistance({odomPose.x, odomPose.y}, sqrtf(powf(mclLocal.rightLatOff, 2)+powf(mclLocal.rightVertOff, 2)), odomPose.theta);
        Point intersection;
        if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (rightDis + mclLocal.rightVertOff) - (tanf(odomPose.theta)*mclLocal.rightLatOff);
                mclY = 70.2-centerToWall;
            }
        } else if (raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, -70.2}, intersection)){
            if (1){
                centerToWall = (rightDis + mclLocal.rightVertOff) - (tanf(odomPose.theta)*mclLocal.rightLatOff);
                mclX = 70.2-centerToWall;
            }
        } else if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (rightDis + mclLocal.rightVertOff) - (tanf(odomPose.theta)*mclLocal.rightLatOff);
                mclY = -70.2+centerToWall;
            }
        }else {
            if (1){
                centerToWall = (rightDis + mclLocal.rightVertOff) - (tanf(odomPose.theta)*mclLocal.rightLatOff);
                mclX = 70.2-centerToWall;
            }
        }

    }

    // Back distance sesnor (if available)
    if (mclLocal.backDistance != nullptr && backDis < 70){
        Point sensor = pointAtDistance({odomPose.x, odomPose.y}, sqrtf(powf(mclLocal.backLatOff, 2)+powf(mclLocal.backVertOff, 2)), odomPose.theta);
        Point intersection;
        if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (backDis + mclLocal.backVertOff) - (tanf(odomPose.theta)*mclLocal.backLatOff);
                mclY = 70.2-centerToWall;
            }
        } else if (raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, -70.2}, intersection)){
            if (1){
                centerToWall = (backDis + mclLocal.backVertOff) - (tanf(odomPose.theta)*mclLocal.backLatOff);
                mclX = 70.2-centerToWall;
            }
        } else if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (backDis + mclLocal.backVertOff) - (tanf(odomPose.theta)*mclLocal.backLatOff);
                mclY = -70.2+centerToWall;
            }
        }else {
            if (1){
                centerToWall = (backDis + mclLocal.backVertOff) - (tanf(odomPose.theta)*mclLocal.backLatOff);
                mclX = 70.2-centerToWall;
            }
        }

    }

    // Left distance sesnor (if available)
    if (mclLocal.leftDistance != nullptr && leftDis < 70){
        Point sensor = pointAtDistance({odomPose.x, odomPose.y}, sqrtf(powf(mclLocal.leftLatOff, 2)+powf(mclLocal.leftVertOff, 2)), odomPose.theta);
        Point intersection;
        if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (leftDis + mclLocal.leftVertOff) - (tanf(odomPose.theta)*mclLocal.leftLatOff);
                mclY = 70.2-centerToWall;
            }
        } else if (raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, -70.2}, intersection)){
            if (1){
                centerToWall = (leftDis + mclLocal.leftVertOff) - (tanf(odomPose.theta)*mclLocal.leftLatOff);
                mclX = 70.2-centerToWall;
            }
        } else if(raySegmentIntersection({sensor.x, sensor.y}, pointAtDistance(sensor, 1, odomPose.theta), {-70.2, 70.2}, {-70.2, 70.2}, intersection)){
            if ((intersection.x<41.8&&intersection.x>51.8)&&(intersection.x>-41.8&&intersection.x<-51.8)){
                centerToWall = (leftDis + mclLocal.leftVertOff) - (tanf(odomPose.theta)*mclLocal.leftLatOff);
                mclY = -70.2+centerToWall;
            }
        }else {
            if (1){
                centerToWall = (leftDis + mclLocal.leftVertOff) - (tanf(odomPose.theta)*mclLocal.leftLatOff);
                mclX = 70.2-centerToWall;
            }
        }

    }
    // END DISTANCE RESET CODE

    // save previous pose
    prevPose = odomPose;

    // calculate global x and y
    odomPose.x += localY * sin(avgHeading);
    odomPose.x += localX * -cos(avgHeading);

    odomPose.y += localY * cos(avgHeading);
    odomPose.y += localX * sin(avgHeading);

    if(mclX != 1000000 && abs(odomPose.x-mclX)< acceptedMCLError){
        odomPose.x = mclX;
        numOfResets += 1;
    }
    if(mclY != 1000000 && abs(odomPose.y-mclY)< acceptedMCLError){
        odomPose.y = mclY;
        numOfResets += 1;
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
                if (pros::competition::is_autonomous()) pros::delay(10);
                else pros::delay(50);
            }
        }};
    }
}
