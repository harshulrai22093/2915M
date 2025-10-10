#include "main.h"

//Defining electronics
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-3, -4, -5});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({8, 9, 10});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intakeMotor({6});
pros::Motor seperateMotor({7});
pros::IMU inertial ({1});

//scraper 
pros::adi::Pneumatics scraper('a', false);

//color sorter piston
pros::adi::Pneumatics colorSorterPiston('b', false);

//optical for coolor sorting
pros::Optical optical(1);

// Simple color sorter configuration
bool allianceColorLearned = false;
bool isRedAlliance = false; // true for red, false for blue

struct Pose {
    double x;       // cm
    double y;       // cm
    double theta;   // radians
};

Pose robotPose = {0.0, 0.0, 0.0};

void odometryTime();
void colorSortTask();

// odometry constants 
constexpr double WHEEL_DIAMETER_IN = 4.0;
constexpr double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_IN * 2.54;
constexpr double EXTERNAL_GEAR_RATIO = 4.0/6.0; 
constexpr double DEG_TO_REV = 1.0 / 360.0;

// optional signs if your encoders come out inverted (keep at 1 unless needed) whatever this means ask harry later
constexpr double LEFT_SIGN = 1.0;
constexpr double RIGHT_SIGN = 1.0;

// task handles so they can persist across modes
pros::Task* gOdomTask = nullptr;
pros::Task* gColorSortTask = nullptr;

void initialize() {
	inertial.reset();
	while (inertial.is_calibrating()) {
		pros::delay(10);
	}
	// Configure optical sensor 
	optical.set_led_pwm(100);      // turn on onboard LED for better readings
	optical.set_integration_time(25); // ms 

	// start background tasks
	if (!gOdomTask) {
		gOdomTask = new pros::Task(odometryTime);
	}
	if (!gColorSortTask) {
		gColorSortTask = new pros::Task(colorSortTask);
	}
}

//Drive distance with a pid controller
void driveDistance(float distance, float maxSpeed = 127, float errorExit = 1, float timeoutMs = 5000)
{
	//Tuning constants
	const float kP = 10;
	const float kI = 0;
	const float kD = 100;

	//Pid variables
	float lastError = distance; 
	float integral = 0;

	//Reset motor positions to 0
	left_mg.tare_position_all();
	right_mg.tare_position_all();

	//Calculating exit time
	float endTime = pros::millis() + timeoutMs;

	//Main loop
	while(pros::millis() < endTime)
	{
		//Getting positions of motors
		std::vector<double> leftPositions = left_mg.get_position_all();
		std::vector<double> rightPositions = right_mg.get_position_all();
		float leftAvgPos = (leftPositions.at(0) + leftPositions.at(1) + leftPositions.at(2)) / 3;
		float rightAvgPos = (rightPositions.at(0) + rightPositions.at(1) + rightPositions.at(2)) / 3;

		//Calculating distance moved
		float leftDistance = (M_PI * (4 * 2.54) * leftAvgPos * 4/6) / 360;
		float rightDistance = (M_PI * (4 * 2.54) * rightAvgPos * 4/6) / 360;
		float avgDistance = (leftDistance + rightDistance) / 2;

		//Calculating error, integral, and derivative
		float error = distance - avgDistance;
		integral += error;
		float derivative = error - lastError;
		
		//Updating last error for next iteration
		lastError = error;

		//prints error to controller
		master.print(0, 0, "%f", error);

		//Calculating output 
		float output = (kP * error) + (kI * integral) + (kD * derivative);

		output = std::clamp(output, -maxSpeed, maxSpeed);

		//Moving motors 
		left_mg.move(output);
		right_mg.move(output);

		//Exit conditions
		if(fabs(error) < errorExit)
		{
			break;
		}

		//Running every 10 ms
		pros::delay(10);
	}
	left_mg.move(0);
	right_mg.move(0);
}

void turningTime(float targetRotation, float maxSpeed = 127, float errorExit = 1)
{
	//Tuning constants
	const float kP = 3;
	const float kI = 0;
	const float kD = 18.5;

	//Pid variables
	float lastError = targetRotation; 
	float integral = 0;

	//Main loop
	while(true)
	{
		//Getting rotation
		double currentRotation = inertial.get_rotation();

		//Calculating error, intagral, and derivative
		float error = targetRotation - currentRotation;
		integral += error;
		float derivative = error - lastError;
		
		//Updating last error for next iteration
		lastError = error;

		//prints error to controller
		master.print(0, 0, "%f", error);

		//Calculating output 
		float output = (kP * error) + (kI * integral) + (kD * derivative);

		output = std::clamp(output, -maxSpeed, maxSpeed);

		//Moving motors 
		left_mg.move(output);
		right_mg.move(-output);

		//Exit conditions
		if(fabs(error) < errorExit)
		{
			break;
		}

		//Running every 10 ms
		pros::delay(10);
	}
	left_mg.move(0);
	right_mg.move(0);
}

// Simple color sorter task
void colorSortTask() {
	while (true) {
		int prox = optical.get_proximity();
		
		if (prox > 100) { // ball detected
			int hue = optical.get_hue();
			
			// Learn alliance color from first ball if not learned
			if (!allianceColorLearned) {
				if (hue >= 330 || hue <= 30) { // red ball
					isRedAlliance = true;
					allianceColorLearned = true;
					master.print(0, 0, "RED Alliance");
				} else if (hue >= 200 && hue <= 240) { // blue ball
					isRedAlliance = false;
					allianceColorLearned = true;
					master.print(0, 0, "BLUE Alliance");
				}
			}
			
			// Sort balls if alliance color is learned
			if (allianceColorLearned) {
				bool isRedBall = (hue >= 330 || hue <= 30);
				bool isBlueBall = (hue >= 200 && hue <= 240);
				
				// Reject wrong color balls
				if ((isRedAlliance && isBlueBall) || (!isRedAlliance && isRedBall)) {
					colorSorterPiston.set_value(true); // extend to reject
				} else {
					colorSorterPiston.set_value(false); // retract to accept
				}
			} else {
				colorSorterPiston.set_value(false); // retract if no alliance set
			}
		} else {
			colorSorterPiston.set_value(false); // retract when no ball
		}
		
		pros::delay(50);
	}
}

// odometry
void odometryTime() {
	// zero encoders and heading reference
	left_mg.tare_position_all();
	right_mg.tare_position_all();
	inertial.tare_rotation();

	double prevLeft = 0.0;
	double prevRight = 0.0;
	uint32_t lastPrint = 0;

	while (true) {
		// get motor positions (deg) and average per side
		std::vector<double> leftPos = left_mg.get_position_all();
		std::vector<double> rightPos = right_mg.get_position_all();

		double leftAvgDeg = (leftPos.at(0) + leftPos.at(1) + leftPos.at(2)) / 3.0;
		double rightAvgDeg = (rightPos.at(0) + rightPos.at(1) + rightPos.at(2)) / 3.0;

		// convert motor degrees -> linear distance at the chassis center (cm)
		double leftDist = LEFT_SIGN * (M_PI * WHEEL_DIAMETER_CM) * (leftAvgDeg * DEG_TO_REV) * EXTERNAL_GEAR_RATIO;
		double rightDist = RIGHT_SIGN * (M_PI * WHEEL_DIAMETER_CM) * (rightAvgDeg * DEG_TO_REV) * EXTERNAL_GEAR_RATIO;

		double dLeft = leftDist - prevLeft;
		double dRight = rightDist - prevRight;

		prevLeft = leftDist;
		prevRight = rightDist;

		double dCenter = (dLeft + dRight) / 2.0;

		// heading from IMU (deg -> rad)
		double headingDeg = inertial.get_rotation();
		robotPose.theta = headingDeg * M_PI / 180.0;

		// integrate into field coordinates (x right, y forward in cm)
		robotPose.x += dCenter * sin(robotPose.theta);
		robotPose.y += dCenter * cos(robotPose.theta);

		// throttle prints to controller ~10 Hz
		uint32_t now = pros::millis();
		if (now - lastPrint >= 100) {
			master.print(0, 0, "%.1f, %.1f", robotPose.x, robotPose.y);
			lastPrint = now;
		}

		pros::delay(10);
	}
}

// boomerang drive
void boomerang(double targetX, double targetY, double maxSpeed = 127, double arriveRadius = 2.0) {
    // Tuning constants 
    const double kP_lin = 5.0; //cm   
    const double kP_ang = 2.2; //rad hopefully
    const double kCurve  = 0.015; //i have no clue

    while(true) {
        double dx = targetX - robotPose.x;
        double dy = targetY - robotPose.y;

        double distance = sqrt(dx*dx + dy*dy);
        if (distance < arriveRadius) break; // arrival check first to reduce overshoot (im so friggin smart)

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
void autonomous()
{
	// color sort task is already running and will learn alliance color from preload
	// odometry task now started in initialize()

	// // LEFT SIDE
	// driveDistance(50, 50, 2);
	// turningTime(-90, 50, 3);
	// driveDistance(65, 50, 3);

	// turningTime(0, 50, 3);
	// driveDistance(30, 50, 13);
	// intakeMotor.move(127);
	// seperateMotor.move(127);
	// testMotor.move(-127);
	// pros::delay(1500);
	
	// driveDistance(-30, 50, 3);
	// turningTime(45, 50, 3);
	// pros::delay(300);
	// intakeMotor.move(127);
	// seperateMotor.move(127);
	// testMotor.move(127);
	// driveDistance(70, 30, 4);
	// pros::delay(300);
	// scraper.toggle();
	// driveDistance(40, 30, 5);
	// pros::delay(100);
	// intakeMotor.move(127);
	// seperateMotor.move(-127);
	// testMotor.move(-127);

	// RIGHT SIDE
	// driveDistance(50, 50, 2);
	// turningTime(90, 50, 3);
	// driveDistance(72, 50, 3);

	// turningTime(0, 50, 3);
	// driveDistance(30, 50, 13);
	// intakeMotor.move(127);
	// seperateMotor.move(127);
	// testMotor.move(-127);
	// pros::delay(1500);
	
	// driveDistance(-15, 50, 3);
	// turningTime(-45, 50, 3);
	// pros::delay(300);
	// intakeMotor.move(127);
	// seperateMotor.move(127);
	// testMotor.move(127);
	// driveDistance(70, 30, 4);
	// pros::delay(300);
	// driveDistance(40, 30, 5);
	// pros::delay(100);
	// intakeMotor.move(-127);
	// seperateMotor.move(-127);
	// testMotor.move(-127);
}

void opcontrol() {
	int speed = 127;

	while (true) {

		// tank control scheme
		int left = master.get_analog(ANALOG_LEFT_Y);      // Gets amount forward/backward from left joystick
		int notLeft = master.get_analog(ANALOG_RIGHT_Y);  // Gets the turn left/right from right joystick
		left_mg.move(left);                               // Sets left motor voltage
		right_mg.move(notLeft);                           // Sets right motor voltage                        

		//top goal
		if(master.get_digital(DIGITAL_L1))
		{
			intakeMotor.move(-127);
			seperateMotor.move(127);
		}
		//mid goal
		else if(master.get_digital(DIGITAL_L2))
		{
			seperateMotor.move(127);
		}
		//intake
		else if(master.get_digital(DIGITAL_R1))
		{
			// Simple intake - color sorting handled by background task
			intakeMotor.move(127);
			seperateMotor.move(127);
			testMotor.move(127);
		}
		//outtake
		else if(master.get_digital(DIGITAL_R2))
		{
			intakeMotor.move(-127);
			seperateMotor.move(-127);
		}
		//Nothing
		else
		{
			intakeMotor.move(0);
			seperateMotor.move(0);
			testMotor.move(0);
		}

    	if(master.get_digital_new_press(DIGITAL_Y))
		{
			scraper.toggle();
		}

		// Manual alliance color override
		if(master.get_digital_new_press(DIGITAL_LEFT))
		{
			isRedAlliance = true;
			allianceColorLearned = true;
			master.rumble("."); // short rumble to confirm
			master.print(0, 0, "RED Override");
		}
		else if(master.get_digital_new_press(DIGITAL_RIGHT))
		{
			isRedAlliance = false;
			allianceColorLearned = true;
			master.rumble(".."); // double rumble to confirm
			master.print(0, 0, "BLUE Override");
		}
		else if(master.get_digital_new_press(DIGITAL_DOWN))
		{
			allianceColorLearned = false;
			master.rumble("---"); // long rumble to confirm disabled
			master.print(0, 0, "Color Reset");
		}

	pros::delay(10);						// Run for 10 ms then update
		
	
	}
}


	






