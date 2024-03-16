#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller con = controller(primary);
motor lMotor1 = motor(PORT2, ratio6_1, false);
motor lMotor2 = motor(PORT1, ratio6_1, false);
motor lMotor3 = motor(PORT3, ratio6_1, false);
motor rMotor1 = motor(PORT8, ratio6_1, true);
motor rMotor2 = motor(PORT9, ratio6_1, true);
motor rMotor3 = motor(PORT10, ratio6_1, true);
motor_group lDrive = motor_group(lMotor1, lMotor2, lMotor3);
motor_group rDrive = motor_group(rMotor1, rMotor2, rMotor3);
inertial inert = inertial(PORT21);
distance kickerTrack = distance(PORT12);
motor intake = motor(PORT7, ratio6_1, false);
motor kicker1 = motor(PORT11, ratio18_1, false);
motor kicker2 = motor(PORT16, ratio18_1, true);
motor_group kicker = motor_group(kicker1, kicker2);
potV2 autonSelect = potV2(Brain.ThreeWirePort.A); 
distance matchloadDetect = distance(PORT18);
pneumatics wingR = pneumatics(Brain.ThreeWirePort.B);
pneumatics wingL = pneumatics(Brain.ThreeWirePort.C);
pneumatics dropDown = pneumatics(Brain.ThreeWirePort.F);
pneumatics dropDown2 = pneumatics(Brain.ThreeWirePort.G);
pneumatics release = pneumatics(Brain.ThreeWirePort.E);
limit hangL = limit(Brain.ThreeWirePort.D);
limit hangR = limit(Brain.ThreeWirePort.H);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}