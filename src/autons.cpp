#include "autons.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


const int DRIVE_SPEED = 550; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED  = 300;
const int SWING_SPEED = 300;

pros::Motor fly (2, pros::E_MOTOR_GEARSET_06);
  pros::Motor arm (11, pros::E_MOTOR_GEARSET_36);
  pros::Controller master (pros::E_CONTROLLER_MASTER);
  #define WING_PORT 'D'
  pros::ADIDigitalOut wings(WING_PORT);
  #define HANG_PORT 'A'
  pros::ADIDigitalOut hang(HANG_PORT);

///
// Constants
///

// It's best practice to tune constants when the robot is empty and with heavier game objects, or with lifts up vs down.
// If the objects are light or the cog doesn't change much, then there isn't a concern here.

void default_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void one_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void two_mogo_constants() {
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 11, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 5, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 35, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 45, 0);
}

void exit_condition_defaults() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}

void modified_exit_condition() {
  chassis.set_exit_condition(chassis.turn_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.swing_exit, 100, 3, 500, 7, 500, 500);
  chassis.set_exit_condition(chassis.drive_exit, 80, 50, 300, 150, 500, 500);
}




void Match_Auton() {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//drive to goal and score matchload //////////////////////////////////////////////////////////////////////////////////////////////
chassis.set_drive_pid(140, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
arm.move_relative(750, 100);
  pros::delay(500);
chassis.set_drive_pid(30, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-25, DRIVE_SPEED);
  chassis.wait_drive();
arm.move_relative(-750, 100);
  pros::delay(500);

//move
chassis.set_drive_pid(-77, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(15, DRIVE_SPEED);
  chassis.wait_drive();

//move to matchload bar and push tri out 
chassis.set_turn_pid(55,TURN_SPEED);
  chassis.wait_drive(); 
chassis.set_drive_pid(-162, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
 wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-50, 400);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-20, 400);
  chassis.wait_drive();
 wings.set_value(false);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

//push tri to offensive side and touch hang bar 
  chassis.set_drive_pid(-102,350);
  chassis.wait_drive();
arm.move_relative(3000, 100);
  pros::delay(50);


}


void Match_Auton_easy() 
{
//drive to goal and score matchload //////////////////////////////////////////////////////////////////////////////////////////////
chassis.set_drive_pid(140, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
arm.move_relative(750, 100);
  pros::delay(500);
chassis.set_drive_pid(30, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-25, DRIVE_SPEED);
  chassis.wait_drive();

 //mid tris 
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(50,DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-55, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(40, DRIVE_SPEED);
  chassis.wait_drive();

arm.move_relative(-750, 100);
  pros::delay(500);
chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(10, DRIVE_SPEED);
  chassis.wait_drive();
arm.move_relative(750, 100);
  pros::delay(500);

chassis.set_drive_pid(-10, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
 wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-89, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(15, DRIVE_SPEED);
  chassis.wait_drive();
 wings.set_value(false);
  chassis.wait_drive();

chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-144, DRIVE_SPEED);
  chassis.wait_drive();

arm.move_relative(750, 100);
  pros::delay(500);
chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-74, DRIVE_SPEED);
  chassis.wait_drive();
  
}

 void Match_Auton_mid() {
  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//drive to goal and score matchload //////////////////////////////////////////////////////////////////////////////////////////////
chassis.set_drive_pid(144, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
arm.move_relative(750, 100);
  pros::delay(500);
chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-20, DRIVE_SPEED);
  chassis.wait_drive();
arm.move_relative(-750, 100);
  pros::delay(500);

//push two over mid onto offensive side 
wings.set_value(true);
  chassis.wait_drive();
  pros::delay(500);
chassis.set_drive_pid(-92, DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(50);
chassis.set_drive_pid(15, DRIVE_SPEED);
  chassis.wait_drive();


//move to matchload bar and push tri out 
 wings.set_value(false);
  chassis.wait_drive();
chassis.set_turn_pid(45,TURN_SPEED);
  chassis.wait_drive(); 
chassis.set_drive_pid(-172, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
 wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-54, 400);
  chassis.wait_drive();
 
 wings.set_value(false);
  chassis.wait_drive();
  chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

//push tri to offensive side and touch hang bar 
  chassis.set_drive_pid(-102,350);
  chassis.wait_drive();
arm.move_relative(3000, 100);
  pros::delay(50);
 }




void Skills1() { 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//drive to goal and score matchload //////////////////////////////////////////////////////////////////////////////////////////////
fly.move_velocity(700);
  pros::c::delay(27000);
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
chassis.set_drive_pid(-60, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(120, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
wings.set_value(false);
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
chassis.set_drive_pid(97, DRIVE_SPEED);
  chassis.wait_drive();


//score on right side of goal
chassis.set_turn_pid(110, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-62, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(215, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-85, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(120, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-82, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-110, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(-305, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-100, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();


}



void Skills2() {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//drive to goal and score matchload //////////////////////////////////////////////////////////////////////////////////////////////

fly.move_velocity(700);
  pros::c::delay(3000);
fly.move_velocity(0);
  pros::delay(50);

chassis.set_drive_pid(9, DRIVE_SPEED);
  chassis.wait_drive(); 
arm.move_relative(-1300, 100);
  pros::delay(2000);
  chassis.wait_drive();

//make way to other side of match load bar///////////////////////

chassis.set_turn_pid(218, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-9, DRIVE_SPEED);
  chassis.wait_drive();  
chassis.set_drive_pid(-256, DRIVE_SPEED);
  chassis.wait_drive();


//score on left side of goal 

chassis.set_turn_pid(180,TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-101, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(120, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-60, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();


//score left centar of goal   
chassis.set_turn_pid(40, TURN_SPEED);
  chassis.wait_drive();
 
chassis.set_drive_pid(-152, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(110, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-82, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(215, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-112, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_drive_pid(87, DRIVE_SPEED);
  chassis.wait_drive();


//score right centar of goal 

chassis.set_turn_pid(110, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-72, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(215, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-92, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_drive_pid(87, DRIVE_SPEED);
  chassis.wait_drive();


//score on right side of goal
chassis.set_turn_pid(110, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-72, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(215, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-95, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(120, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-72, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-110, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_turn_pid(-305, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-100, DRIVE_SPEED);
  chassis.wait_drive();

chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-50, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(50, DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Wait Until and Changing Max Speed
///
void skills() {
  //machload 
fly.move_velocity(700);
  pros::c::delay(10000);
fly.move_velocity(0);
  pros::delay(50);
arm.move_relative(-750, 100);
  pros::delay(3000);
  chassis.wait_drive();

//make way to other side of match load bar///////////////////////
chassis.set_drive_pid(3, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-258, DRIVE_SPEED);
  chassis.wait_drive();

//score on left side of goal 
wings.set_value(true);
  chassis.wait_drive();
chassis.set_turn_pid(15,TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-72, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(72, DRIVE_SPEED);
  chassis.wait_drive();

//score left centar of goal   
chassis.set_turn_pid(35, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-148.5, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
wings.set_value(false);
  chassis.wait_drive();
chassis.set_drive_pid(144, DRIVE_SPEED);
  chassis.wait_drive();

//score right centar of goal 
wings.set_value(true);
  chassis.wait_drive();
chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-144, DRIVE_SPEED);
  chassis.wait_drive();
wings.set_value(false);
  chassis.wait_drive();
chassis.set_drive_pid(144, DRIVE_SPEED);
  chassis.wait_drive();

//score on right side of goal 
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-144, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-72, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-72,DRIVE_SPEED);
  chassis.wait_drive();
}



///
// Swing Example
///
void shortsideauton() {

chassis.set_drive_pid(72, DRIVE_SPEED);
chassis.wait_drive();
chassis.set_drive_pid(-25, DRIVE_SPEED);
chassis.wait_drive();

}



///
// Auto that tests everything
///
void combining_movements() {
 chassis.set_drive_pid(144, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();
arm.move_relative(750, 100);
  pros::delay(500);
chassis.set_drive_pid(25, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_drive_pid(-15, DRIVE_SPEED);
  chassis.wait_drive();
//arm.move_relative(-750, 100);
  //pros::delay(500);

//push two over mid onto offensive side 
wings.set_value(true);
  chassis.wait_drive();
  pros::delay(500);
chassis.set_drive_pid(-92, DRIVE_SPEED);
  chassis.wait_drive();
  pros::delay(50);
chassis.set_drive_pid(15, DRIVE_SPEED);
  chassis.wait_drive();


//move to matchload bar and push tri out 
 wings.set_value(false);
  chassis.wait_drive();
chassis.set_turn_pid(45,TURN_SPEED);
  chassis.wait_drive(); 
chassis.set_drive_pid(-172, DRIVE_SPEED);
  chassis.wait_drive();
chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();
 wings.set_value(true);
  chassis.wait_drive();
chassis.set_drive_pid(-54, 400);
  chassis.wait_drive();
 
 arm.move_relative(3000, 100);
  pros::delay(500);
 wings.set_value(false);
  chassis.wait_drive();
chassis.set_turn_pid(-90, TURN_SPEED);
  chassis.wait_drive();

//push tri to offensive side and touch hang bar 
chassis.set_drive_pid(-105,350);
  chassis.wait_drive();
}



///
// Interference example
///
void tug (int attempts) {
  for (int i=0; i<attempts-1; i++) {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered) {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees. 
// If interfered, robot will drive forward and then attempt to drive backwards. 
void interfered_example() {
 chassis.set_drive_pid(24, DRIVE_SPEED, true);
 chassis.wait_drive();

 if (chassis.interfered) {
   tug(3);
   return;
 }

 chassis.set_turn_pid(90, TURN_SPEED);
 chassis.wait_drive();
}



// . . .
// Make your own autonomous functions here!
// . . .