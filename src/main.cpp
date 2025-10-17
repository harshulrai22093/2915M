#include "main.h"

// Define controller 
pros::Controller master(pros::E_CONTROLLER_MASTER);
// Define left hand side drive motor group
pros::MotorGroup left_mg({-3, -4, -5});
// Define right hand side drive motor group
pros::MotorGroup right_mg({8, 9, 10});
// Define top intake motor 
pros::Motor topintakemotor({6});
// Define bottom intake motor
pros::Motor bottomintakemotor({7});
// Define intertial sensor
pros::IMU inertial ({1});
// Define colour sensor
pros::Optical colour_sensor({2});
// Define mid goal pneumatic
pros::adi::Pneumatics midgoaler('a', false);
// Define scraper pneumatic
pros::adi::Pneumatics scraper('h', false);

// Variables for position and orientation
struct Pose {
	// cm
    double x;
	// cm
    double y;
	// radians
    double theta;
};
// Reset robot orientation and position
Pose robotPose = {0.0, 0.0, 0.0};

// ???????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
void odometryTime();
// odometry constants ??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
constexpr double WHEEL_DIAMETER_IN = 4.0;
constexpr double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_IN * 2.54;
constexpr double EXTERNAL_GEAR_RATIO = 4.0/6.0; 
constexpr double DEG_TO_REV = 1.0 / 360.0;
// optional signs if your encoders come out inverted (keep at 1 unless needed) whatever this means ask harry later ????????????????????????????????????????????????????????????????????????????
constexpr double LEFT_SIGN = 1.0;
constexpr double RIGHT_SIGN = 1.0;

// Variable for alliance colour, this colour represents what's in the preloader, defaulting to blue
enum AllianceColour { RED, BLUE };
AllianceColour alliancecolour = BLUE;

// task handles so they can persist across modes ??????????????????????????????????????????????????????????????????????????????????
pros::Task* gOdomTask = nullptr;

// When the robot is intialized
void initialize() {
	// Reset and initialize interial sensor
	inertial.reset();
	while (inertial.is_calibrating()) {
		pros::delay(10);
	}
	// ?????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????
	if (!gOdomTask) {
		gOdomTask = new pros::Task(odometryTime);
	}
	// Enable colour sensor LED
    colour_sensor.set_led_pwm(100);
	// Set integration time to 5 ms
    colour_sensor.set_integration_time(5);
}

// Colour sensing function depending on alliance colour 
void coloursensefunction() {

	// Check if the block is close enoguh
    int proximity = colour_sensor.get_proximity();
    if (proximity > 150) {

		// Get block colour
        int hue = colour_sensor.get_hue();

		// If alliance colour red
        if (alliancecolour == RED) {

            // Reject blue
            if (hue >= 200 && hue <= 240) {
                topintakemotor.move(-127);
			
			// Else do nothing	
			} else {
                topintakemotor.move(0);
			}
		// If alliance colour blue
		} else if (alliancecolour == BLUE) {

            // Reject red
            if (hue >= 0 && hue <= 20)
                topintakemotor.move(-127);

			// Else do nothing
			} else {
                topintakemotor.move(0);
			}
		
		// Else do nothing
    } else {
    	topintakemotor.move(0);
	}
}

// Loop the colour sensing function
void coloursenseloop() {
    while (true) {
        coloursensefunction();
        pros::delay(10);
    }
}

// // Loop the colour sensor for background task
// void coloursenseloop(){
//     while (true){
//         if(master.get_digital(DIGITAL_R1)) {  // only run sorter when R1 is held
//             coloursensefunction();
//         } else {
//             topintakemotor.move(0);
//             bottomintakemotor.move(0);
//         }
//         pros::delay(10);
//     }
// }

// Drive distance with a pid controller
void driveDistance(float distance, float maxSpeed = 127, float errorExit = 1, float timeoutMs = 5000) {
	// Tuning constants
	const float kP = 10;
	const float kI = 0;
	const float kD = 100;

	// PID variables
	float lastError = distance; 
	float integral = 0;

	// Reset motor positions to 0
	left_mg.tare_position_all();
	right_mg.tare_position_all();

	// Calculating exit time
	float endTime = pros::millis() + timeoutMs;

	// Main loop
	while(pros::millis() < endTime) {
		// Getting positions of motors
		std::vector<double> leftPositions = left_mg.get_position_all();
		std::vector<double> rightPositions = right_mg.get_position_all();
		float leftAvgPos = (leftPositions.at(0) + leftPositions.at(1) + leftPositions.at(2)) / 3;
		float rightAvgPos = (rightPositions.at(0) + rightPositions.at(1) + rightPositions.at(2)) / 3;

		// Calculating distance moved
		float leftDistance = (M_PI * (4 * 2.54) * leftAvgPos * 4/6) / 360;
		float rightDistance = (M_PI * (4 * 2.54) * rightAvgPos * 4/6) / 360;
		float avgDistance = (leftDistance + rightDistance) / 2;

		// Calculating error, integral, and derivative
		float error = distance - avgDistance;
		integral += error;
		float derivative = error - lastError;
		
		// Updating last error for next iteration
		lastError = error;

		// Prints error to controller
		master.print(0, 0, "%f", error);

		// Calculating output 
		float output = (kP * error) + (kI * integral) + (kD * derivative);

		output = std::clamp(output, -maxSpeed, maxSpeed);

		// Moving motors 
		left_mg.move(output);
		right_mg.move(output);

		// Exit conditions
		if(fabs(error) < errorExit) {
			break;
		}

		// Running every 10 ms
		pros::delay(10);
	}
	left_mg.move(0);
	right_mg.move(0);
}

void turningTime(float targetRotation, float maxSpeed = 127, float errorExit = 1) {
	// Tuning constants
	const float kP = 3;
	const float kI = 0;
	const float kD = 18.5;

	// PID variables
	float lastError = targetRotation; 
	float integral = 0;

	// Main loop
	while(true)	{
		// Getting rotation
		double currentRotation = inertial.get_rotation();

		// Calculating error, intagral, and derivative
		float error = targetRotation - currentRotation;
		integral += error;
		float derivative = error - lastError;
		
		// Updating last error for next iteration
		lastError = error;

		// Prints error to controller
		master.print(0, 0, "%f", error);

		// Calculating output 
		float output = (kP * error) + (kI * integral) + (kD * derivative);

		output = std::clamp(output, -maxSpeed, maxSpeed);

		// Moving motors 
		left_mg.move(output);
		right_mg.move(-output);

		// Exit conditions
		if(fabs(error) < errorExit) {
			break;
		}

		// Running every 10 ms
		pros::delay(10);
	}
	left_mg.move(0);
	right_mg.move(0);
}

// Odometry
void odometryTime() {
	// Zero encoders and heading reference
	left_mg.tare_position_all();
	right_mg.tare_position_all();
	inertial.tare_rotation();

	double prevLeft = 0.0;
	double prevRight = 0.0;
	uint32_t lastPrint = 0;

	while (true) {
		// Get motor positions (deg) and average per side
		std::vector<double> leftPos = left_mg.get_position_all();
		std::vector<double> rightPos = right_mg.get_position_all();

		double leftAvgDeg = (leftPos.at(0) + leftPos.at(1) + leftPos.at(2)) / 3.0;
		double rightAvgDeg = (rightPos.at(0) + rightPos.at(1) + rightPos.at(2)) / 3.0;

		// Convert motor degrees -> linear distance at the chassis center (cm)
		double leftDist = LEFT_SIGN * (M_PI * WHEEL_DIAMETER_CM) * (leftAvgDeg * DEG_TO_REV) * EXTERNAL_GEAR_RATIO;
		double rightDist = RIGHT_SIGN * (M_PI * WHEEL_DIAMETER_CM) * (rightAvgDeg * DEG_TO_REV) * EXTERNAL_GEAR_RATIO;

		double dLeft = leftDist - prevLeft;
		double dRight = rightDist - prevRight;

		prevLeft = leftDist;
		prevRight = rightDist;

		double dCenter = (dLeft + dRight) / 2.0;

		// Heading from IMU (deg -> rad)
		double headingDeg = inertial.get_rotation();
		robotPose.theta = headingDeg * M_PI / 180.0;

		// Integrate into field coordinates (x right, y forward in cm)
		robotPose.x += dCenter * sin(robotPose.theta);
		robotPose.y += dCenter * cos(robotPose.theta);

		// Throttle prints to controller ~10 Hz
		uint32_t now = pros::millis();
		if (now - lastPrint >= 100) {
			master.print(0, 0, "%.1f, %.1f", robotPose.x, robotPose.y);
			lastPrint = now;
		}

		pros::delay(10);
	}
}

// Boomerang drive
void boomerang(double targetX, double targetY, double maxSpeed = 127) {
    // Tuning constants 
    const double kP_lin = 5.0; //cm   
    const double kP_ang = 2.2; //rad hopefully
    const double kCurve  = 0.015; //i have no clue

    while(true) {
        double dx = targetX - robotPose.x;
        double dy = targetY - robotPose.y;

        double distance = sqrt(dx*dx + dy*dy);

        double targetTheta = atan2(dy, dx);

        double angleError = targetTheta - robotPose.theta;
        while(angleError > M_PI) angleError -= 2*M_PI;
        while(angleError < -M_PI) angleError += 2*M_PI;

        // forward/turn
        double forwardPower = kP_lin * distance;
        double turnPower = (kP_ang * angleError) + (kCurve * forwardPower * angleError);

        forwardPower = std::clamp(forwardPower, -maxSpeed, maxSpeed);
        turnPower = std::clamp(turnPower, -maxSpeed, maxSpeed);

        left_mg.move(forwardPower + turnPower);
        right_mg.move(forwardPower - turnPower);

        pros::delay(10);
    }
    left_mg.move(0);
    right_mg.move(0);
}


// Autonomous, runs when auton runs
void autonomous() {

	// Check what colour is in the preloader
	// Check distance, if it is near
	int proximity = colour_sensor.get_proximity();
	if (proximity > 150) {
		
		// Get Colour
		int hue = colour_sensor.get_hue();
		
		// If blue detected make alliance colour blue
		if (hue >= 200 && hue <= 240) {
			alliancecolour = BLUE;

		// If red detected make allaince colour red
		} else if (hue >= 0 && hue <= 20) {
			alliancecolour = RED;

		}
	}
}

void opcontrol() {
	int speed = 127;
	
    // Background task to run the colour sensing loop
    pros::Task colourtask(coloursenseloop);

	while (true) {

		// tank control scheme
		int left = master.get_analog(ANALOG_LEFT_Y);      // Gets amount forward/backward from left joystick
		int notLeft = master.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick
		left_mg.move(left);                               // Sets left motor voltage
		right_mg.move(notLeft);                           // Sets right motor voltage                        

		//top goal
		if(master.get_digital(DIGITAL_L1)) {
			bottomintakemotor.move(127);
		}
		//mid goal
		else if(master.get_digital(DIGITAL_L2)) {
			topintakemotor.move(127);
		}
		//outtake
		else if(master.get_digital(DIGITAL_R2)) {
			topintakemotor.move(-127);
			bottomintakemotor.move(-127);
		}
		//Nothing
		else {
			topintakemotor.move(0);
			bottomintakemotor.move(0);
		}

    	if(master.get_digital_new_press(DIGITAL_Y)) {
			midgoaler.toggle();
		}

		if(master.get_digital_new_press(DIGITAL_B)) {
			scraper.toggle();
		}
	}
	// Run for 10 ms then update
	pros::delay(10);
	}



	






