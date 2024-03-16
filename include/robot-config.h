using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller con;
extern motor lMotor1;
extern motor lMotor2;
extern motor lMotor3;
extern motor rMotor1;
extern motor rMotor2;
extern motor rMotor3;
extern motor_group lDrive;
extern motor_group rDrive;
extern inertial inert;
extern distance kickerTrack;
extern motor intake;
extern motor kicker1;
extern motor kicker2;
extern motor_group kicker;
extern potV2 autonSelect;
extern distance matchloadDetect;
//extern pneumatics lift;
extern pneumatics wingL;
extern pneumatics wingR;
//extern pneumatics hang;
extern pneumatics dropDown;
extern pneumatics dropDown2;
extern pneumatics release;
extern limit hangL;
extern limit hangR;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );