#include "main.h"

//Defining electronics
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-8, -9, -10});   
pros::MotorGroup right_mg({11, 12, 13});  
pros::Motor intakeMotor({5});
pros::Motor seperateMotor({19});
pros::IMU inertial ({18});
pros::Motor testMotor({15});

//scraper 
pros::adi::Pneumatics scraper('a', false); 

// sorter piston 
pros::adi::Pneumatics sorter('b', false);

//optical for coolor sorting
pros::Optical optical(1);

// Color sorter state
bool colorSorterEnabled = true; // enable by default, set to false to disable
enum class AllianceColor { Red, Blue };
AllianceColor allianceColor = AllianceColor::Red; 
uint32_t rejectUntilMs = 0; 

struct Pose {
    double x;       // cm
    double y;       // cm
    double theta;   // radians
};

Pose robotPose = {0.0, 0.0, 0.0};

void odometryTime();

// odometry constants ask harry
constexpr double WHEEL_DIAMETER_IN = 4.0;
constexpr double WHEEL_DIAMETER_CM = WHEEL_DIAMETER_IN * 2.54;
constexpr double EXTERNAL_GEAR_RATIO = 4.0/6.0; 
constexpr double DEG_TO_REV = 1.0 / 360.0;

// optional signs if your encoders come out inverted (keep at 1 unless needed) whatever this means ask harry later
constexpr double LEFT_SIGN = 1.0;
constexpr double RIGHT_SIGN = 1.0;

// odom task handle so it can persist across modes
pros::Task* gOdomTask = nullptr;

void initialize() {
	inertial.reset();
	while (inertial.is_calibrating()) {
		pros::delay(10);
	}
	// Configure optical sensor 
	optical.set_led_pwm(100);      // turn on onboard LED for better readings
	optical.set_integration_time(25); // ms 

	// Try to auto-detect alliance from preload
	detectAllianceFromPreload();
	if (allianceColor == AllianceColor::Red) master.set_text(2, 0, "Alliance: RED ");
	else master.set_text(2, 0, "Alliance: BLUE");

	// start odometry background task 
	if (!gOdomTask) {
		gOdomTask = new pros::Task(odometryTime);
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

// Color detection helper for optical sensor
enum class DetectedColor { Red, Blue, Unknown };

static DetectedColor detectColor() {
    // Simple heuristic based on hue
    double hue = optical.get_hue();          // 0 - 360
    double sat = optical.get_saturation();   // 0 - 100
    double prox = optical.get_proximity();   // 0 - 255 (approx; higher == closer)

    if (prox < 50 || sat < 10) return DetectedColor::Unknown;
    if (hue >= 330.0 || hue < 30.0) return DetectedColor::Red;
    if (hue >= 190.0 && hue <= 250.0) return DetectedColor::Blue;
    return DetectedColor::Unknown;
}

// Attempt to detect alliance from the preload at startup
static void detectAllianceFromPreload() {
    // sample for up to something seconds looking for a color
    int redCount = 0, blueCount = 0;
    uint32_t start = pros::millis();
    while (pros::millis() - start < 2000) {
        DetectedColor c = detectColor();
        if (c == DetectedColor::Red) redCount++;
        else if (c == DetectedColor::Blue) blueCount++;
        pros::delay(20);
    }
    if (redCount > blueCount && redCount >= 5) allianceColor = AllianceColor::Red;
    else if (blueCount > redCount && blueCount >= 5) allianceColor = AllianceColor::Blue;
    // if neither wins keep default
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

//Autonomous, runs when auton runs
void autonomous()
{
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
	driveDistance(50, 50, 2);
	turningTime(90, 50, 3);
	driveDistance(72, 50, 3);

	turningTime(0, 50, 3);
	driveDistance(30, 50, 13);
	intakeMotor.move(127);
	seperateMotor.move(127);
	testMotor.move(-127);
	pros::delay(1500);
	
	driveDistance(-15, 50, 3);
	turningTime(-45, 50, 3);
	pros::delay(300);
	intakeMotor.move(127);
	seperateMotor.move(127);
	testMotor.move(127);
	driveDistance(70, 30, 4);
	pros::delay(300);
	driveDistance(40, 30, 5);
	pros::delay(100);
	intakeMotor.move(-127);
	seperateMotor.move(-127);
	testMotor.move(-127);
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
			intakeMotor.move(127);
			seperateMotor.move(127);
			testMotor.move(-127);
		}
		//mid goal
		else if(master.get_digital(DIGITAL_L2))
		{
			intakeMotor.move(127);
			seperateMotor.move(-127);
			testMotor.move(-127);
		}
		//intake
		else if(master.get_digital(DIGITAL_R1))
		{
			if (colorSorterEnabled) {
				DetectedColor c = detectColor();
				bool wrong = (c == DetectedColor::Red && allianceColor == AllianceColor::Blue)
				         || (c == DetectedColor::Blue && allianceColor == AllianceColor::Red);
				if (wrong) sorter.extend(); else sorter.retract();
			} else {
				sorter.retract();
			}
			intakeMotor.move(127);
			seperateMotor.move(127);
			testMotor.move(127);
		}
		//outtake
		else if(master.get_digital(DIGITAL_R2))
		{
			intakeMotor.move(-127);
			seperateMotor.move(-127);
			testMotor.move(-127);
		}
		//Nothing
		else
		{
			intakeMotor.move(0);
			seperateMotor.move(0);
			testMotor.move(0);
			sorter.retract();
		}

    	if(master.get_digital_new_press(DIGITAL_Y))
		{
			scraper.toggle();
		}

	pros::delay(10);						// Run for 10 ms then update
		
	}
}





