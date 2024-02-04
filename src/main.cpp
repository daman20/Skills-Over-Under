#include "main.h"

// SECTION: CONSTANTS

auto startingState = OdomState{1000_mm, -1400_mm, 0_deg};
auto goalLocation = Point{1200_mm, 0_mm};
auto matchLoadZone1 = Point{-1400_mm, 1400_mm};
auto matchLoadZone2 = Point{-1400_mm, -1400_mm};

// SECTION: DEVICE CONFIGURATION


// Okapilib Controller
Controller controller;




// okapilib Chassis

std::shared_ptr<OdomChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors({17, -16}, {-7, 6})
    // blue gearset, 4 in wheel diam, 5 in wheel track (center-to-center distance between the wheels (center-to-center meaning the width between the centers of both wheels))
    .withDimensions(AbstractMotor::gearset::blue, {{15_in, 5_in}, imev5BlueTPR})
    // TODO: SET UP AND TUNE PID??????
    .withMaxVelocity(100)
    .withOdometry()
    .buildOdometry();

// profile controller for autonomous to allow for preplanned routes
std::shared_ptr<AsyncMotionProfileController> profileController = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();

// end OKAPILIB Chassis

// catapult motors
Motor cat1(-5, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
Motor cat2(15, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
MotorGroup catapult({cat1, cat2});

// arm motor group
Motor wings1(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor wings2(19, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
MotorGroup wings ({wings1, wings2});

// catapult touch sensor (to figure out of it is down)
auto catapultLimitSwitch = ADIButton('A', false);

// intake motor
Motor intake(20, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

// acorn touch sensor to detect whether or not an acorn is loaded
auto acornLoad = OpticalSensor(5, OpticalSensorOutput::hue, true);

bool wingsOut = true;

pros::ADIDigitalOut matchLoadArm ('H');

// END SECTION: DEVICE CONFIGURATION

// START SECTION: HELPER FUNCTIONS

/*
 * @brief: catapult control function
 * @param: number of times to launch
*/
void launch(int numLaunches = 1, bool aimbot = false) {
  if(aimbot){
    chassis->turnToPoint(goalLocation); // AIMBOT: turn to face the goal
  }
  catapult.setBrakeMode(AbstractMotor::brakeMode::brake); // set the catapult to brake to hold at the bottom
  for (int i = 0; i < numLaunches; i++) { //repeat for the number of times to launch
    catapult.moveVelocity(-100); // move the catapult down
    pros::delay(1000); //wait so that the catapult can launch without hitting touch sensor
    // RELOAD: move the catapult down until the limit switch is pressed
    while(!catapultLimitSwitch.changedToPressed()) { // check if the limit switch is not pressed, meaning the catapult isn't down
      pros::delay(40); //delay not to overload
    }
    pros::delay(5);
    catapult.moveVelocity(0); // stop the catapult when done
  }
}

void matchLoadAutoLaunch(){
  chassis->driveToPoint(matchLoadZone1); // drive to the match load zone
  // launch acorns as they are loaded
  while(true){
    if (acornLoad.getHue() < 100 && acornLoad.getHue() > 80) { // if it is green
      launch();
    }
    pros::delay(20); // delay to not overload
  }
}

void alternateWings(){
  wings1.setBrakeMode(AbstractMotor::brakeMode::brake);
  wings2.setBrakeMode(AbstractMotor::brakeMode::brake);

  if(!wingsOut){
    wings.moveAbsolute(0, 60);
  }
  else{
    wings.moveAbsolute(95, 90);
  }
  wingsOut = !wingsOut;
}
// END SECTION: HELPER FUNCTIONS

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	matchLoadArm.set_value(true); // set the match load arm to the correct position
  	launch(50, true); // launch 3 times
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	

}