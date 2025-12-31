// Imports other C files for use, especially with lemlib
#include "main.h"
#include "autos.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp" // IWYU pragma: keep
#include <cstdint> // IWYU pragma: keep
#include <string>
#include "lemlib/pose.hpp"
#include "pros/misc.h"



// Global Variables for controlling color sort and what code is run
bool redTeam = true;
bool isSkills = false;
bool arcade = true;

int code = 5;
int numOfCodes = 5;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


// Left and Right drive smotor groups
pros::MotorGroup
    leftMotors({-1, -2, -3},
               pros::MotorGearset::blue); 
                                          
pros::MotorGroup rightMotors(
    {4, 5, 6},
    pros::MotorGearset::blue); 


// Inertial Sensor on port 19
pros::Imu imu(11);

// Limit switch for changing code
pros::adi::DigitalIn limitSwitch('d');

// Optical sensor for color sosrt
pros::Optical color(20);


// Matchloader piston
pros::adi::DigitalOut matchLoader('c');

// Tracking Wheel lift piston
pros::adi::DigitalOut wheelLift('b');

// Goal Descore piston
pros::adi::DigitalOut descore('e');

// Intake motors
pros::Motor intake(-12, pros::MotorGearset::blue);
pros::Motor intake_upper(-13, pros::MotorGearset::green);
pros::Motor direction(8, pros::v5::MotorGears::blue);

// Horizontal tracking wheel
pros::Rotation horizontalEnc(1);

// vertical tracking wheel 
pros::Rotation verticalEnc(-12);


// Distance sensors for a simplified version of Monte Carlo Localization
pros::Distance back1(1);
pros::Distance back2(2);
pros::Distance right1(19);
pros::Distance right2(20);


lemlib::MCLSensors mcl(nullptr, nullptr, &right1, &right2, false, true, 1.75, -0.25, 5,
     4, 1.75, -0.25, 5, 4);
// Horizontal Tracking wheel lemlib settings
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, 1);

// Vertical tracking wheel lemlib settings
lemlib::TrackingWheel vertical(&verticalEnc, 2, 0.5);


// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,              // left motor group
    &rightMotors,             // right motor group
    10,                       // 10 inch track width
    lemlib::Omniwheel::NEW_2, // using new 4" omnis
    600 * 0.75,               // drivetrain rpm is 360
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);


// lateral motion controller
lemlib::ControllerSettings
    linearController(11,  // proportional gain (kP)
                     0,   // integral gain (kI)
                     30,   // derivative gain (kD)
                     3, // anti windup
                    1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
    );


// angular motion controller
lemlib::ControllerSettings
    angularController(4.5,   // proportional gain (kP)
                      0,   // integral gain (kI)
                      30,  // derivative gain (kD)
                      3,   // anti windup
                      1,   // small error range, in degrees
                      100, // small error range timeout, in milliseconds
                      3,   // large error range, in degrees
                      500, // large error range timeout, in milliseconds
                      0    // maximum acceleration (slew)
    );


// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr,   // vertical tracking wheel 2, set to
                                       // nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr,     // horizontal tracking wheel 2, set to
                                     // nullptr as we don't have a second one
                            &imu // inertial sensor
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve
    throttleCurve(3,    // joystick deadband out of 127
                  10,   // minimum output where drivetrain will move out of 127
                  1.019 // expo curve gain
    );


// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3,    // joystick deadband out of 127
               10,   // minimum output where drivetrain will move out of 127
               1.019 // expo curve gain
    );


// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController,
                        sensors, &throttleCurve, &steerCurve);



// Global variables for intake management
bool intaking = false; 
bool outtaking = false;
bool middleGoal = false;
bool scoring = false;
int mogoStatus = 3;
int cancelStatus = 1;
bool cancel = false;




void intakeControl() {
    // Defines local variables for intake management
    bool redBall = false;
    bool blueBall = true;
    bool middle = false;
    float upperSpeed = 0;
    float intakeSpeed = 0;
    float directionSpeed = 0;

    if(code == 1){
        cancel = true;
    }

    descore.set_value(LOW);

    while (1) {

        // Detects what color of ball is in the intake
        if ((color.get_hue() < 25 && color.get_hue() > 5) && color.get_proximity()>80){
            redBall = true;
            blueBall = false;
        } else if (color.get_hue()<220 && color.get_hue() > 170&& color.get_proximity()>80){
            blueBall = true;
            redBall = false;
        } else {
            blueBall = false;
            redBall = false; 
        }

        // Code to control the intake
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || intaking){
            middle = false;
            upperSpeed += 127;
            intakeSpeed += 127;
            directionSpeed += 127;
            
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) || outtaking){
            middle = false;
            upperSpeed -= 127;
            intakeSpeed -= 90;
            directionSpeed -=127;

        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || middleGoal){
            middle = true;
            upperSpeed +=127;
            intakeSpeed +=127;
            directionSpeed -=80;
        } else {
            middle = false;
            upperSpeed = 0;
            intakeSpeed = 0;
            directionSpeed = 0;
        }



        // Code to stop color sort in case of emergency or misinput when starting the match
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && cancelStatus == 1){
        	cancel = true;
        	cancelStatus = 2;
    	}
		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && cancelStatus == 2){
        	cancelStatus = 3;
    	}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) and cancelStatus == 3){
            cancel = false;
        	cancelStatus = 4;
    	}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && cancelStatus == 4){
        	cancelStatus = 1;
    	}

        // Run motors
        intake_upper.move(upperSpeed);
        intake.move(intakeSpeed);
        direction.move(directionSpeed);
        upperSpeed = 0;
        intakeSpeed = 0;
        directionSpeed = 0;

        // Delay to save resourses for other tasks
        pros::delay(10);


        // Color sort
        /*
        if(!isSkills && !cancel){
            if((redTeam && blueBall) || (!redTeam && redBall)){
                if(!middle){
                    direction.move(-127);
                    intake.move(50);
                    intake_upper.move(50);
                    pros::delay(300);
                    direction.brake();
                    intake.brake();
                    intake_upper.brake();
                }else if (middle){
                    direction.move(127);
                    intake.move(50);
                    intake_upper.move(50);
                    pros::delay(500);
                    direction.brake();
                    intake.brake();
                    intake_upper.brake();
                }
            }
        }
            */

        

    }

}

// Controls which color the color sort sorts
void centerButton(){
	if (redTeam){
		redTeam = false;
	} else {
		redTeam = true;
	}
}

// Controls what code runs based on limit switch presses
void on_center_button() {
    while(1){
    //change to 0 to create auton roulette
    if (limitSwitch.get_new_press()==1){
        code ++;
    }
	
	if (code > numOfCodes){
		code = 1;
	}
    pros::delay(50);
	}
}


// Controls all inputs to the brain screen to consolidate the management of the brain
void screenUpdate(){
	while(1){
        pros::lcd::set_text(3, "X: " + std::to_string(chassis.getPose().x));
        pros::lcd::set_text(4, "Y: " + std::to_string(chassis.getPose().y));
	pros::lcd::set_text(5, "Theta: " + std::to_string(chassis.getPose().theta));
	switch (code){
		case 1:
			pros::lcd::set_text(1, "Skills");
            isSkills = true;
            break;
			
		case 2:
			pros::lcd::set_text(1, "Right");
            isSkills = false;
            break;
			
		case 3:
			pros::lcd::set_text(1, "Left");
            break;
			
		case 4:
			pros::lcd::set_text(1, "AWP");
            break;
		case 5:
			pros::lcd::set_text(1, "Long Goal Only");
            break;
			
            

    }
	if(redTeam){
		pros::lcd::set_text(2, "Red Team");
	} else {
		pros::lcd::set_text(2, "Blue Team");
	}
	pros::delay(50);
}
}



//Runs before the code starts to 
void initialize() {
    color.set_integration_time(3);
	color.set_led_pwm(100);
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    mcl.calibrate();
	pros::delay(1000); // waits for sensors to callibrate
	pros::Task autoSwitch(on_center_button); // starts the thread to manage autonomous switiching
	pros::Task intakeining(intakeControl); // starts the thread to manage intake control
	pros::Task screen(screenUpdate); // starts the thread to manage screen inputs
    
    pros::lcd::register_btn1_cb(centerButton); // starts the thread to manage team color changes
    
}





// Runs if the robot is disabled by a competition switch or field controller
void disabled() {
    
    

}


// Runs when the feild control or competition switch starts the match
void competition_initialize() {
}



// Starts the correct autonomous code based on the code selected 
void autonomous() {
    switch (code){
        case 1:
            skills();
			break;
        case 2:
            redRight();
			break;
        case 3:
            redLeft();
			break;
        case 4:
            redAWP();
			break;
		case 5:
			redRightLong();
			break;
    }
}


// Dirver Control Code
void opcontrol() {
    scoring = false;
	intaking = false;
	outtaking = false;
    // loop to continuously update motors

    if(code == 1){
        wheelLift.set_value(LOW);
    }
    
    int loaderStatus = 1;
    int liftStatus = 1;
    int lifStatus = 1;
    while (true) {
        float t = 0.1;  // can be changed
    float tt = 0.1; // can be changed

    float SatValue = 0.1; // can be changed
    int power = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X))*-1;
    int deadzone = 2; // can be changed
    // Apply Deadzone
    if (abs(power) <= deadzone) {
      power = 0;
    }
    if (abs(turn) <= deadzone) {
      turn = 0;
    }

    double CurvedPower = std::exp(((std::abs(power) - 127) * t) / 1270) * power;
    double CurvedTurn = std::exp(((std::abs(turn) - 127) * tt) / 1270) * turn;

    double RPower = CurvedPower * (1 - (std::abs(turn) / 127.0) * SatValue);

    float leftSpeed = (RPower + CurvedTurn);
    float rightSpeed = (RPower - CurvedTurn);
    // If code is set to use arcade drive control
    if(arcade){
        rightMotors.move(leftSpeed);
        leftMotors.move(rightSpeed);
    }
    // If code is set to use tank drive control
    else {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), 
            true);
    }

        // Matchloader piston control
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && loaderStatus == 1){
        	matchLoader.set_value(HIGH);
        	loaderStatus = 2;
    	}
		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && loaderStatus == 2){
        	loaderStatus = 3;
    	}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and loaderStatus == 3){
        	matchLoader.set_value(LOW);
        	loaderStatus = 4;
    	}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && loaderStatus == 4){
        	loaderStatus = 1;
    	}


        // Controls the descore piston
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && liftStatus == 1){
        	descore.set_value(HIGH);
        	liftStatus = 2;
    	}
		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && liftStatus == 2){
        	liftStatus = 3;
    	}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) and liftStatus == 3){
        	descore.set_value(LOW);
        	liftStatus = 4;
    	}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && liftStatus == 4){
        	liftStatus = 1;
    	}



        // Controls the tracking wheel lift piston
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && lifStatus == 1){
        	wheelLift.set_value(HIGH);
        	lifStatus = 2;
    	}
		if (!controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && lifStatus == 2){
        	lifStatus = 3;
    	}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) and lifStatus == 3){
        	wheelLift.set_value(LOW);
        	lifStatus = 4;
    	}
		if(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && lifStatus == 4){
        	lifStatus = 1;
    	}

    }
    // Delay to save resources for other tasks
	pros::delay(10);
}
