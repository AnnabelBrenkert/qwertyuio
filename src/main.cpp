#include "main.h"
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////
/*Flywheel Variables*/
	bool lastKnownButtonL1State;
	bool lastKnownButtonBState;
	int flywheelState = 0; /*0 = off, 1 = Spin Forward, 2 = Spin Backward*/

/*Flywheel Variables*/
	bool lastKnownButtonAState;
	int hangState = 0; /*0 = off, 1 = hang*/

/*bowling Variables*/
  bool lastKnownButtonDownState;
  int bowlingstate = 0;


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {14, -1, -15}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{19, 12, -20}
  

  // IMU Port
  ,13

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,0.6

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);

  //pros::Motor fly (17);
  //pros::Motor arm (11);
  //pros::Controller master (pros::E_CONTROLLER_MASTER);
  //#define DIGITAL_SENSOR_PORT 'A'
  //`pros::ADIDigitalOut wings(DIGITAL_SENSOR_PORT);

  const int DRIVE_SPEED = 550; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 300;
const int SWING_SPEED = 300;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(6, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  //Autonomous Selector using LLEMU
 /* ez::as::auton_selector.add_autons({
    Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
    
  }); */ 

  // Initialize chassis and auton selector
  chassis.initialize();
  //ez::as::initialize();
  

wings.set_value(0);
fly.set_brake_mode(MOTOR_BRAKE_COAST);
arm.set_brake_mode(MOTOR_BRAKE_BRAKE);
std::cout<< "Robot Initialized" << endl;
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

  // . . .
}



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
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set wwwssq q  to hold.  This helps autonomous consistency.
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 // ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
 std::cout << "Ran DT Auto Calibration" << endl;


//drive to goal and score matchload //////////////////////////////////////////////////////////////////////////////////////////////
fly.move_velocity(700);
  pros::c::delay(7000);
fly.move_velocity(0);
  pros::delay(50);

chassis.set_drive_pid(9, DRIVE_SPEED);
  chassis.wait_drive(); 
arm.move_relative(-1300, 100);
  pros::delay(1500);
  chassis.wait_drive();

//make way to other side of match load bar///////////////////////

chassis.set_turn_pid(208, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-9, DRIVE_SPEED);
  chassis.wait_drive();  
chassis.set_drive_pid(-256, DRIVE_SPEED);
  chassis.wait_drive();


//score on left side of goal 
chassis.set_turn_pid(180,TURN_SPEED);
  chassis.wait_drive();
wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-40, DRIVE_SPEED);
  chassis.wait_drive();
wings.set_value(false);
  chassis.wait_drive();  
chassis.set_drive_pid(-30, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(120, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-35, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();

//score left centar of goal   
chassis.set_turn_pid(40, TURN_SPEED);
  chassis.wait_drive();
 
chassis.set_drive_pid(-132, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(110, TURN_SPEED);
  chassis.wait_drive();
 chassis.set_drive_pid(-75, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(215, TURN_SPEED);
  chassis.wait_drive();
wings.set_value(true);
  chassis.wait_drive(); 
chassis.set_drive_pid(-102, DRIVE_SPEED);
  chassis.wait_drive();

wings.set_value(false);
  chassis.wait_drive();
chassis.set_drive_pid(80, DRIVE_SPEED);
  chassis.wait_drive();


//score right centar of goal 

chassis.set_turn_pid(110, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-62, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(215, DRIVE_SPEED);
  chassis.wait_drive();
wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-102, DRIVE_SPEED);
  chassis.wait_drive();

wings.set_value(false);
  chassis.wait_drive();
chassis.set_drive_pid(35, DRIVE_SPEED);
  chassis.wait_drive();


//score on right side of goal
/*chassis.set_turn_pid(110, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-62, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(215, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-85, DRIVE_SPEED);
  chassis.wait_drive();
  */

chassis.set_turn_pid(120, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-112, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(250, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-100, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(-80, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-110, DRIVE_SPEED);
  chassis.wait_drive();

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
void opcontrol() 
{
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  while (true) {
    
    chassis.tank();
   
   /*Flywheel Control Code*/
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) != lastKnownButtonL1State)
		{
			lastKnownButtonL1State = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && flywheelState == 0 || flywheelState == 2 || flywheelState == 3 )
			{
				flywheelState = 1;
			}
    
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && flywheelState == 1)
			{
				flywheelState = 0;
			}
      
		}  

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) != lastKnownButtonBState)
		{
			lastKnownButtonBState = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && flywheelState == 0 || flywheelState == 1 || flywheelState == 3 )
			{
				flywheelState = 2;
			}
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && flywheelState == 2)
			{
				flywheelState = 0;
			}
		}
	
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) != lastKnownButtonDownState)
		{
			lastKnownButtonDownState = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && flywheelState == 0 || flywheelState == 1 || flywheelState == 2)
			{
				flywheelState = 3;
			}
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && flywheelState == 3)
			{
				flywheelState = 0;
			}
		}
    
		switch (flywheelState)
		{
			case 0:
				fly.move(0);
				break;
			case 1:
				fly.move(106);
				break;
			case 2:
				fly.move(-106);
				break;
      case 3:
				fly.move(76);
				break;
      
		}

/*Arm Control*/
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			arm.move(127);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			arm.move(-127);
		}
		else 
		{
			arm.move(0);
		}

/*Wing Code*/
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			wings.set_value(1);
      pros::delay(10);
		}
		else 
		{
			wings.set_value(0);
			pros::delay(10);
		}
	
   
//hanging code
   	if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) != lastKnownButtonAState)
		{
			lastKnownButtonAState = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);
			if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && hangState == 0 || hangState == 2)
			{
				hangState = 1;
			} 
			else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) && hangState == 1)
			{
				hangState = 0;
			}
		}  

    switch (hangState)
    {
      case 0:
			 hang.set_value(0);
      pros::delay(10);
      break;
			
      case 1:
				hang.set_value(1);
      pros::delay(10);
				break;
    }
			

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }

}