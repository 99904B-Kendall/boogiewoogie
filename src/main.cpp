#include "MiniPID.cpp"
#include "vex.h"
#include "MiniPID.h"
using namespace vex;
competition Competition;

double degToInch(double degrees) {
  return ((degrees / 360) * 2.9 * 3.1416 );
}
int rapid=0;
bool overRapid=false;
bool skily=false;
void drv(double dist, double targetHeading, double maxvel, double timeout) {
  MiniPID distControl = MiniPID(1800, 0, 4500);//1150,0,3000
  MiniPID diffControl = MiniPID(300, 0, 2000); //300 0 2000
  distControl.reset();
  diffControl.reset();
  distControl.setOutputLimits(-120 * maxvel, 120 * maxvel);
  diffControl.setOutputLimits(-120 * maxvel, 120 * maxvel);
  distControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  int counter = 0;
  lMotor1.setPosition(0, deg);
  rMotor1.setPosition(0, deg);
  while ((vex::timer::system() - startTime < timeout * 1000)) {
    if ((dist - degToInch(lMotor1.position(deg) + rMotor1.position(deg))) > 2) {
      distControl.errorSum = 0;
    }
    double speed = distControl.getOutput(
        (degToInch(lMotor1.position(deg) + rMotor1.position(deg)) / 2), dist);
    double turnErr = diffControl.getOutput(inert.rotation(), targetHeading);

    lDrive.spin(fwd, nearbyint(speed + turnErr), voltageUnits::mV);
    rDrive.spin(fwd, nearbyint(speed - turnErr), voltageUnits::mV);

    if (-1600 < (speed + turnErr) && (speed + turnErr) < 1600) {
      counter++;
      if (counter > 8) {
        break;
      }
    } else {
      counter = 0;
    }

    wait(20, msec);
    printf("%f\t", dist - degToInch((lMotor1.position(deg) + rMotor1.position(deg)) / 2));
    printf("%f\n", speed);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}
void drv2(double dist, double targetHeading, double maxvel, double timeout) {
  MiniPID distControl = MiniPID(1800, 0, 4500);//1150,0,3000
  MiniPID diffControl = MiniPID(300, 0, 2000); //300 0 2000
  distControl.reset();
  diffControl.reset();
  distControl.setOutputLimits(-120 * maxvel, 120 * maxvel);
  diffControl.setOutputLimits(-120 * maxvel, 120 * maxvel);
  distControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  int counter;
  lMotor1.setPosition(0, deg);
  rMotor1.setPosition(0, deg);
  while ((vex::timer::system() - startTime < timeout * 1000)) {
    if ((dist - degToInch(lMotor1.position(deg) + rMotor1.position(deg))) > 2) {
      distControl.errorSum = 0;
    }
    double speed = distControl.getOutput(
        (degToInch(lMotor1.position(deg) + rMotor1.position(deg)) / 2), dist);
    double turnErr = diffControl.getOutput(inert.rotation(), targetHeading);

    lDrive.spin(fwd, nearbyint(speed + turnErr), voltageUnits::mV);
    rDrive.spin(fwd, nearbyint(speed - turnErr), voltageUnits::mV);

    if (-1600 < (speed + turnErr) && (speed + turnErr) < 1600) {
      counter++;
      if (counter > 8) {
        break;
      }
    } else {
      counter = 0;
    }
    if(inert.roll()<-10){
      break;
    }

    wait(20, msec);
    printf("%f\t", dist - degToInch((lMotor1.position(deg) + rMotor1.position(deg)) / 2));
    printf("%f\n", speed);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}
void curve(double dist, double leftVel,double rightVel, double timeout) {
  MiniPID distControl = MiniPID(1800, 0, 4500);
  distControl.reset();
  distControl.setOutputLimits(-12000,12000);
  distControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  int counter;
  lMotor1.setPosition(0, deg);
  rMotor1.setPosition(0, deg);
  while ((vex::timer::system() - startTime < timeout * 1000)) {
  
    double speed = distControl.getOutput((degToInch(lMotor1.position(deg) + rMotor1.position(deg)) / 2), dist);
    

    lDrive.spin(fwd, nearbyint(speed*(leftVel/100)), voltageUnits::mV);
    rDrive.spin(fwd, nearbyint(speed*(rightVel/100)), voltageUnits::mV);

    if (-1600 < speed && speed < 1600) {
      counter++;
      if (counter > 8) {
        break;
      }
    } else {
      counter = 0;
    }

    wait(20, msec);
    printf("%f\t", dist - degToInch((lMotor1.position(deg) + rMotor1.position(deg)) / 2));
    printf("%f\n", speed);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}
void curve2(double dist, double leftVel,double rightVel, double finishDist, double timeout) {
  MiniPID distControl = MiniPID(1800, 0, 4500);
  distControl.reset();
  distControl.setOutputLimits(-12000,12000);
  distControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  int counter;
  lMotor1.setPosition(0, deg);
  rMotor1.setPosition(0, deg);
  while ((vex::timer::system() - startTime < timeout * 1000)) {
  
    double speed = distControl.getOutput((degToInch(lMotor1.position(deg) + rMotor1.position(deg)) / 2), dist);
    

    lDrive.spin(fwd, nearbyint(speed*(leftVel/100)), voltageUnits::mV);
    rDrive.spin(fwd, nearbyint(speed*(rightVel/100)), voltageUnits::mV);
    if((degToInch(lMotor1.position(deg) + rMotor1.position(deg)) / 2)>finishDist){
      leftVel=100;
      rightVel=100;
    }
    if (-1600 < speed && speed < 1600) {
      counter++;
      if (counter > 8) {
        break;
      }
    } else {
      counter = 0;
    }

    wait(20, msec);
    printf("%f\t", dist - degToInch((lMotor1.position(deg) + rMotor1.position(deg)) / 2));
    printf("%f\n", speed);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}

void trn(float target, float timeout) {
  MiniPID turnControl = MiniPID(430, 0, 3200);
  turnControl.setOutputLimits(-12000, 12000);
  turnControl.reset();
  // turnControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  double speed;
  int counter;
  bool finishedLoop = false;
  while ((vex::timer::system() - startTime < timeout * 1000) && !finishedLoop) {
   
    speed = turnControl.getOutput(inert.rotation(), target);

    lDrive.spin(fwd, nearbyintf(speed), voltageUnits::mV);
    rDrive.spin(reverse, nearbyintf(speed), voltageUnits::mV);

    if (-1600 < speed && speed < 1600) {
      counter++;
      if (counter > 8) {
        break;
      }
    } else {
      counter = 0;
    }

    printf("%f\t", inert.rotation());
    printf("%f\t", speed);
    printf("%f\t", fmod((target - inert.rotation() + 180), 360) - 180);
    printf("%lu\n", vex::timer::system() - startTime);
    wait(10, msec);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}
void swingL(float target, float timeout) {
  MiniPID turnControl = MiniPID(470, 0, 3000);
  turnControl.setOutputLimits(-12000, 12000);
  turnControl.reset();
  // turnControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  double speed;
  int counter;
  while ((vex::timer::system() - startTime < timeout * 1000)) {
    speed = turnControl.getOutput(inert.rotation(), target);

    lDrive.spin(fwd, nearbyintf(speed), voltageUnits::mV);
    rDrive.stop(hold);

    if (-1600 < speed && speed < 1600) {
      counter++;
      if (counter > 8) {
        break;
      }
    } else {
      counter = 0;
    }

    printf("%f\t", inert.rotation());
    printf("%f\t", speed);
    printf("%f\t", fmod((target - inert.rotation() + 180), 360) - 180);
    printf("%lu\n", vex::timer::system() - startTime);
    wait(10, msec);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}
void swingR(float target, float timeout) {
  MiniPID turnControl = MiniPID(470, 0, 3000);
  turnControl.setOutputLimits(-12000, 12000);
  turnControl.reset();
  // turnControl.setMaxIOutput(2000);
  int startTime = vex::timer::system();
  double speed;
  int counter;
  bool finishedLoop = false;
  while ((vex::timer::system() - startTime < timeout * 1000) && !finishedLoop) {

    speed = turnControl.getOutput(inert.rotation(), target);

    lDrive.stop(hold);
    rDrive.spin(reverse, nearbyintf(speed), voltageUnits::mV); 

    if (-1600 < speed && speed < 1600) {
      counter++;
      if (counter > 8) {
        finishedLoop = true;
      }
    } else {
      counter = 0;
    }

    printf("%f\t", inert.rotation());
    printf("%f\t", speed);
    printf("%lu\n", vex::timer::system() - startTime);
    wait(10, msec);
  }
  lDrive.stop(brake);
  rDrive.stop(brake);
}

double delayInput;

int wingRetDel ()
{
  wait(delayInput, sec);
  wingL.close();
  wingR.close();
  return 1;
}
int wingExtDel ()
{
  wait(delayInput, sec);
  
  wingR.open();
  return 1;
}
int dropDel ()
{
  wait(delayInput, sec);
  dropDown.close();
  return 1;
}
int wilybily ()
{
  wait(.3, sec);
  intake.spin(reverse,10,pct); 
  return 1;
}



void retractWingsDelay(double delay) {
  delayInput = delay;
  vex::task wingsTask(wingRetDel);
}
void extendWingsDelay(double delay) {
  delayInput = delay;
  vex::task wingsTask(wingExtDel);
}
void retractDropDelay(double delay) {
  delayInput = delay;
  vex::task wingsTask(dropDel);
}



int Kicker() {
  bool wap=true;
  while (true) {
      if(rapid==1){
        kicker.spin(fwd, 12100, vex::voltageUnits::mV);
      }
      else if(rapid==0){
        if (kickerTrack.objectDistance(inches) > .7) {
          kicker.spin(fwd, 11800, vex::voltageUnits::mV);
        } else {
          kicker.stop(brake);
        }
        if(release.value()==1&&wap) {
          if((hangL.value()+hangR.value())==2){
            rapid=4;
            wap=false;
          }
        }
      }
    else if(rapid==4){
      kicker.spin(reverse, 12700, vex::voltageUnits::mV);
      wait(.1,sec);
      waitUntil(kicker.velocity(pct)>-1&&kicker.velocity(pct)<1);
      kicker.stop(coast);
      wait(.3,sec);
      if(inert.roll()<-6){
        kicker.spin(fwd, 12700, vex::voltageUnits::mV);
        wait(.2,sec);
      }

      //wait(2.5,sec);
      //kicker.spin(fwd, 12700, vex::voltageUnits::mV);
      //wait(1.3,sec);
      rapid=0;
    }
    wait(20, msec);
  }
  return 0;
}
int johnUp ()
{
  overRapid=true;
  wait(.2,sec);
  overRapid=false;
  return 1;
}
void fs5ball(){
  inert.setRotation(-112, deg);
  wingR.open();
  retractWingsDelay(.2);
  intake.spin(fwd,100,pct);
  drv(58.5, -112, 100, 3);
  drv(-21, -112, 100, 3);
  trn(-45,3);
  intake.spin(reverse,3000,vex::voltageUnits::mV);
  drv(37,5,100,.8);
  drv(-11,0,100,1);
  trn(159,2);
  intake.spin(fwd,100,pct);
  drv(27.5,159,100,2);
  trn(50,2);
  intake.spin(reverse,100,pct);
  drv(30,18,100,1);
  swingL(155, 2);
  intake.spin(fwd,100,pct);
  //drv(47.5,180,100,1);
  curve2(45,100,56,20,1);
  wait(.2,sec);
  drv(-30,180,100,1);
  trn(350,2);
  drv(22 ,323,100,2);
  dropDown.open();
  retractDropDelay(.7);
  intake.spin(reverse,50,pct);
  swingR(270, 1);
  swingR(318, 1);
  wingL.open();
  drv(90,290,100,1.1);
  wingL.close();
  drv(-15,270,100,2);
}
void fs6ball(){
  inert.setRotation(-112, deg);
  wingR.open();
  retractWingsDelay(.2);
  intake.spin(fwd,100,pct);
  drv(59, -112, 100, 3);
 // drv(-21, -112, 100, 3);
  trn(0,3);
  intake.spin(reverse,3000,vex::voltageUnits::mV);
  wingL.open();
  drv(39,5,100,1);
  wingL.close();
  drv(-11,0,100,1);
  trn(155,2);
  intake.spin(fwd,30,pct);
  drv(31,155,100,2);
  trn(55,2);
  intake.spin(reverse,100,pct);
  drv2(31,21,100,1);
  wait(.2,sec);
  swingL(155, 2);
  intake.spin(fwd,100,pct);
  //drv(47.5,180,100,1);
  curve2(41,100,60,20,1);
  wait(.2,sec);
  drv(-30,180,100,1);
  trn(350,2);
  drv(22 ,323,100,2);
  dropDown.open();
  retractDropDelay(.7);
  intake.spin(reverse,50,pct);
  swingR(270, 1);
  swingR(318, 1);
  wingL.open();
  drv(90,290,100,1.1);
  wingL.close();
  drv(-15,270,100,2);
}
///////////////////////////////////
void newfs6ball(){
  inert.setRotation(-114, deg);
  wingR.open();
  retractWingsDelay(.2);
  intake.spin(fwd,100,pct);
  drv(60, -114, 100, 3);
  vex::task intakeTask(wilybily);
  //wait(.2,sec);
  drv(-66.2, -117, 100, 3);
  intake.spin(reverse,12000,vex::voltageUnits::mV);
  trn(162,3);
  inert.setRotation((inert.rotation()-360), deg);
  //wait(1,sec);
  //trn(-180,3);
  intake.spin(fwd,12000,vex::voltageUnits::mV);
  drv(33.75,-180,100,1);
  wait(.15,sec);
  drv(-30,-180,100,1);
  trn(-10,2);
  wingL.open();

  //drv(22.5,-43,100,2);
  //wait(.1,sec);
  drv(22,-43,100,2);
  intake.spin(reverse,65,pct);
  dropDown.open();
  retractDropDelay(.5);
  swingR(-108, 1);
  intake.spin(reverse,100,pct);
  swingR(-60, 1);
  curve(90, 75, 100, .8);
  drv(-60,-90,100,.3);
  drv(60,-90,100,.6);
  wingL.close();
  drv(-23,-90,100,2);
  trn(-156,2);
  intake.spin(fwd,6000,vex::voltageUnits::mV);
  drv(54,-156,100,.9);
  trn(-90,2);
  intake.spin(reverse,12000,vex::voltageUnits::mV);
  wingL.open();
  wingR.open();
  //curve2(60,100,30,10,2);
  drv(5,-90,100,.2);
  drv(60,-3,100,1.5);
}
///////////////////////////////
void nsCenterTouch(){
  inert.setRotation(-71, deg);
  wingL.open();
  retractWingsDelay(0.2);
  intake.spin(fwd,100,pct);
  drv(49, -71, 100, 1.5);
  drv(-44.5, -57, 100, 3);
  intake.stop(hold);
  swingR(40,2);
  dropDown.open();
  wait(.2,sec);
  retractDropDelay(.6);
  swingR(-10, 2);
  dropDown.close();
  trn(45,2);
  drv(-25, 45, 100, 2);
  swingR(90, 1.5);
  //drv(5,90,100,1.5);
  drv(-10,115,50,.4);
  drv(7,90,50,.5);
  drv(-10,115,50,.4);
  drv(3,100,100,2);
  swingR(63,1);
  intake.spin(reverse,100,pct);
  wingL.open();
  drv(31.5,63,100,3.3);
  swingR(25, 2);
  wait(2,sec);
  retractWingsDelay(.8);
  drv(33,5,100,3.3);
}
void nsPushBowl(){
  inert.setRotation(-71, deg);
  wingL.open();
  retractWingsDelay(0.2);
  intake.spin(fwd,100,pct);
  drv(49, -71, 100, 3);
  drv(-2, -71, 100, 3);
  trn(5,1);
  wingL.open();
  wingR.open();
  //drv2(26, 12, 100, 1);
  curve(20, 100, 30, 1);
  wingL.close();
  wingR.close();
  drv(-19,0, 100, 3);
  trn(-60,1);
  drv(-41, -60, 100, 2);
  intake.stop(hold);
  swingR(40,1.3);
  dropDown.open();
  wait(.2,sec);
  retractDropDelay(.6);
  swingR(-10, 2);
  dropDown.close();
  trn(45,2);
  drv(-23.5, 45, 100, 2);
  swingR(90, 1.5);
  //drv(5,90,100,1.5);
  drv(-10,115,50,.4);
  drv(7,90,50,.5);
  drv(-10,115,50,.4);
  drv(3,100,100,2);
  swingR(63,1);
  intake.spin(reverse,100,pct);
  wingL.open();
  drv(31.5,63,100,3.3);
  swingR(25, 2);
  retractWingsDelay(.8);
  drv(33,5,100,3.3);
  drv(-42,13,100,3.3);
  //swingR(38,3);
  dropDown.open();
  drv(-10.5,45,100,3.3);
}
void nsPushTouch(){
  inert.setRotation(-71, deg);
  wingL.open();
  retractWingsDelay(0.2);
  intake.spin(fwd,100,pct);
  drv(49, -71, 100, 3);
  drv(-2, -71, 100, 3);
  trn(5,1);
  wingL.open();
  wingR.open();
  //drv2(26, 12, 100, 1);
  curve(20, 100, 30, 1);
  wingL.close();
  wingR.close();
  drv(-19,0, 100, 3);
  trn(-60,1);
  drv(-41, -60, 100, 2);
  intake.stop(hold);
  swingR(40,1.3);
  dropDown.open();
  wait(.2,sec);
  retractDropDelay(.6);
  swingR(-10, 2);
  dropDown.close();
  trn(45,2);
  drv(-23.5, 45, 100, 2);
  swingR(90, 1.5);
  //drv(5,90,100,1.5);
  drv(-10,115,50,.4);
  drv(7,90,50,.5);
  drv(-10,115,50,.4);
  drv(3,100,100,2);
  swingR(63,1);
  intake.spin(reverse,100,pct);
  wingL.open();
  drv(31.5,63,100,3.3);
  swingR(25, 2);
  retractWingsDelay(.8);
  drv(33,5,100,3.3);
}
void nsCenterBowl(){
  inert.setRotation(-71, deg);
  wingL.open();
  retractWingsDelay(0.2);
  intake.spin(fwd,100,pct);
  drv(49, -71, 100, 1.5);
  drv(-44.5, -57, 100, 3);
  intake.stop(hold);
  swingR(40,2);
  dropDown.open();
  wait(.2,sec);
  retractDropDelay(.6);
  swingR(-10, 2);
  dropDown.close();
  trn(45,2);
  drv(-25, 45, 100, 2);
  swingR(90, 1.5);
  //drv(5,90,100,1.5);
  drv(-10,115,50,.4);
  drv(7,90,50,.5);
  drv(-10,115,50,.4);
  drv(3,100,100,2);
  swingR(63,1);
  intake.spin(reverse,100,pct);
  wingL.open();
  drv(31.5,63,100,3.3);
  swingR(25, 2);
  retractWingsDelay(.8);
  drv(33,5,100,3.3);
  drv(-43.5,13,100,3.3);
  //swingR(38,3);
  dropDown.open();
  drv(-10,45,100,3.3);
}


void nsBarrierTouch(){
  inert.setRotation(-59, deg);
  wingL.open();
  retractWingsDelay(0.2);
  intake.spin(fwd,100,pct);
  drv(59, -58, 100, 3);
  drv(-58, -53, 100, 3);
  intake.stop(hold);
  swingR(30,2);
  dropDown.open();
  wait(.6,sec);
  retractDropDelay(.6);
  swingR(-10, 2);
  dropDown.close();
  trn(45,2);
  drv(-24, 45, 100, 2);
  swingR(90, 1.5);
  drv(5,90,100,1.5);
  drv(-13,100,50,1.5);
  swingR(60,1);
  intake.spin(reverse,100,pct);
  wingL.open();
  drv(27,60,100,3.3);
  swingR(10, 2);
  retractWingsDelay(.2);
  drv(38,5,100,3.3);
  wingL.close();
}


void nsSimpleAWP() {
  inert.setRotation(0, deg);
  intake.spin(fwd, 12000, vex::voltageUnits::mV);
  swingR(22, 1.5);
  intake.stop(brake);
  dropDown.open();
  wait(.5, sec);
  //retractDropDownDelay(1.5);
  swingR(-10, 1.5);
  dropDown.close();
  swingR(-10, .5);
  swingL(18, 2);
  intake.spin(reverse, 12000, vex::voltageUnits::mV);
  drv(38.25, 0, 100, 1.5);
}
void fs2ball(){
  inert.setRotation(-70, deg);
  intake.spin(reverse, 5000, vex::voltageUnits::mV);
  drv(20,-70,50,2);
  drv(-12,-70,100,2);
  dropDown.open();
  retractDropDelay(.8);
  wait(.3,sec);
  swingR(-105, 1);
  swingR(-60, 1);
  wingL.open();
  intake.spin(reverse, 5000, vex::voltageUnits::mV);
  drv(50,-72,100,2);
  wingL.close();
  drv(-20,-75,100,2);
  drv(50,-76,100,2);
  drv(-7,-76,100,2);
  trn(-5,3);
  drv(-53,-5,100,2);
  dropDown.open();
  trn(20,8);
}

void skills(){
  inert.setRotation(70, deg);//70
  vex::task Kickertask(Kicker);
  //int c1 = 0, c2 = 0;
  
  drv(-34,70,100,.7); //push in them red balls
  drv(11.25,90,100,1.5);
  trn(-21, 2.2); //line up to shoot
  drv(-8.75,-19,100,.7); //go shoot fr
  dropDown.open();
  rapid=1;
  //kicker.spin(fwd, 12700, vex::voltageUnits::mV);
  lDrive.stop(hold);
  rDrive.stop(hold);
  wait(19.6,sec);//19
  rapid=0;
  //kicker.stop(coast);
  lDrive.stop(coast);
  rDrive.stop(coast);
  //dropDown.close();
  retractDropDelay(.15);
  intake.spin(fwd,12000,vex::voltageUnits::mV);
  drv(53,-3,100,1.5);
  trn(-55,3);
  intake.spin(reverse,12700,vex::voltageUnits::mV);//12000
  wingL.open();
  //wingR.open();
  drv2(108,-80,100,1.5);
  //trn(-120,2);
  drv(-20,-80,100,1.4);//15
  drv2(34,-75,100,1.3);
  intake.spin(fwd,12000,vex::voltageUnits::mV);
  drv(-14,-90,30,3);//15
  wingL.close();
  // wingR.close();
  trn(-155, 2);
  //intake.spin(fwd,12000,vex::voltageUnits::mV);
  drv(41,-155,100,3);
  //swingL(-10, 2);
  intake.spin(reverse,6000,vex::voltageUnits::mV);
  swingL(-40, 2);
  extendWingsDelay(.5);
  curve2(92,100,80,18,3);
  //drv(94.5,-5,70,3);
  intake.spin(reverse,12000,vex::voltageUnits::mV);
  curve(100,100,60,1.4);
  drv(-15,90,100,1.1);
  drv(35,70,100,.8);
  wingR.close();
  //intake.spin(reverse,3000,vex::voltageUnits::mV);
  drv(-14.5,85,100,1.5);




  trn(164,3);
  extendWingsDelay(.9);
  //intake.spin(reverse,3000,vex::voltageUnits::mV);
  drv(29,164,100,3);
  swingR(12, 2);
  //intake.spin(reverse,3000,vex::voltageUnits::mV);
  //wingR.open();
  //wingL.open();
  drv(50,12,100,1.4);
  // drv(-8,15,100,1.5);
  // drv(30,15,100,.4);
  wingL.close();
  wingR.close();
  drv(-8,0,100,1.5);
  trn(-155,2);
  drv(16.5,-155,100,1.5);
  extendWingsDelay(.4);
  swingR(-357,2);//-250
  inert.setRotation((inert.rotation()+365), deg);
  wingL.open();
  drv(50,3,100,1.1);
  // drv(-8,15,100,1.5);
  // drv(30,15,100,.4);
  wingL.close();
  wingR.close();
  drv(-36,30,100,1.5);
  trn(115,2);
  drv(35,85,100,1.5);
  extendWingsDelay(.4);
  trn(-38,2);
  wingR.open();
  wingL.open();
  intake.spin(reverse,12000,vex::voltageUnits::mV);
  curve2(70,100,70,20,1.5);
  //wingL.close();
  wingR.close();
  intake.spin(reverse,12000,vex::voltageUnits::mV);
  drv(-36,0,100,1.5);
  retractWingsDelay(1.6);
  trn(77,2);
  intake.spin(reverse,6000,vex::voltageUnits::mV);
  drv(55.5,74,80,1.5);
  swingR(-30,2);
  wingL.open();
  intake.spin(reverse,12000,vex::voltageUnits::mV);
  curve(100,70,100,1.2);
  // drv(10,-40,100,1.5);
  // //swingR(-60,2);
  // drv(30,-75,100,1.4);
  drv(-16,-55,100,1);
  intake.spin(reverse,3000,vex::voltageUnits::mV);
  drv(80,-45,100,.8);
  wingL.close();
  release.open();
  drv(-36,-40,100,1);
  trn(-180,3);
  drv(58.5,-179,100,2.5);
  rapid=4;
}
// bool auto_started = false;
bool calibrated = false;
void pre_auton(void) {
  vexcodeInit();
  wait(.1,sec);
  if(2<=autonSelect.value(deg)&&autonSelect.value(deg)<45){
    con.Screen.print("skills");
    skily=true;
  }
  if (!calibrated) {
    inert.calibrate();
    wait(3, sec);
    // Brain.Screen.print("Calibrated");
    calibrated = true;
  }
    
  else if(45<=autonSelect.value(deg)&&autonSelect.value(deg)<84){
    con.Screen.print("nsCenterTouch");
  }
  else if(84<=autonSelect.value(deg)&&autonSelect.value(deg)<127){
    con.Screen.print("nsPushTouch");
  }
  else if(127<=autonSelect.value(deg)&&autonSelect.value(deg)<170){
    con.Screen.print("nsCenterBowl");
  }
  else if(170<=autonSelect.value(deg)&&autonSelect.value(deg)<212){
    con.Screen.print("nsPushBowl");
  }
  else if(212<=autonSelect.value(deg)&&autonSelect.value(deg)<253){
    con.Screen.print("fs5ball");
  }
  else if(253<=autonSelect.value(deg)&&autonSelect.value(deg)<306){
    con.Screen.print("fs6ball");
  }
  else if(306<=autonSelect.value(deg)||autonSelect.value(deg)<2){
    con.Screen.print("fs2ballTouch");
    con.Screen.print(autonSelect.value(deg));

  }
}

void autonomous(void) {
  if(2<=autonSelect.value(deg)&&autonSelect.value(deg)<45){
    skills();
  }
  else if(45<=autonSelect.value(deg)&&autonSelect.value(deg)<84){
    nsCenterTouch();
  }
  else if(84<=autonSelect.value(deg)&&autonSelect.value(deg)<127){
    nsPushTouch();
  }
  else if(127<=autonSelect.value(deg)&&autonSelect.value(deg)<170){
    nsCenterBowl();
  }
  else if(170<=autonSelect.value(deg)&&autonSelect.value(deg)<212){
    nsPushBowl();
  }
  else if(212<=autonSelect.value(deg)&&autonSelect.value(deg)<253){
    fs5ball();
  }
  else if(253<=autonSelect.value(deg)&&autonSelect.value(deg)<306){
    //fs6ball();
    newfs6ball();
  }
  else if(306<=autonSelect.value(deg)||autonSelect.value(deg)<2){
    fs2ball();
  }
  //nsSimpleAWP();
  //nsCenterBowl();
  //nsCenterTouch();
  //skills();
  //fs5ball();
  //fs6ball();
  //nsBarrierTouch();
  //fs2ball();
  //nsPushOverBow();
}

double exponetial(double conValue) {
  double output;
  if (conValue < 24 && conValue > 6) {
    output = (.3 * conValue + 9.9);
  } else if (conValue >= 24) {
    output = (.864 * (conValue - 24) + 17.1);
  } else if (conValue > -24 && conValue < -6) {
    output = (.3 * conValue - 9.9);
  } else if (conValue <= -24) {
    output = (.864 * (conValue + 24) - 17.1);
  } else {
    output = 0;
  }

  if (output > 100)
  {
    return 100;
  }
  if (output < -100)
  {
    return -100;
  }
  return output;
}

void usercontrol(void) {
  lDrive.setStopping(coast);
  rDrive.setStopping(coast);
  wait(100, msec);
  vex::task KickerTask(Kicker); 
  wingL.close();
  wingR.close();
  double lDriveAmt;
  double rDriveAmt;
  bool t1 = true, t2 = true, t3 = true, t4 = true, t5 = true, t6 = true;
    if(skily){
      inert.setRotation(70, deg);
      drv(-34,70,100,.7); //push in them red balls
      drv(11.75,90,100,1.5);
      trn(-20, 2.2); //line up to shoot
      drv(-9,-19,100,.7); //go shoot fr
      dropDown.open();
      rapid=1;
      }


  while (1) {

    lDriveAmt = exponetial(con.Axis3.value());
    rDriveAmt = exponetial(con.Axis2.value());

    lDrive.spin(fwd, lDriveAmt * 120, vex::voltageUnits::mV);
    rDrive.spin(fwd, rDriveAmt * 120, vex::voltageUnits::mV);

    if (con.ButtonRight.pressing()) {
      if (t3) {
        t3 = false;
        if (wingL.value() == 0) {
          wingL.open();

        } else {
          wingL.close();
        }
      }
    } else {
      t3 = true;
    }
    if (con.ButtonY.pressing()) {
      if (t2) {
        t2 = false;
        if (wingR.value() == 0) {

          wingR.open();
        } else {
          wingR.close();
        }
      }
    } else {
      t2 = true;
    }
    if (con.ButtonDown.pressing()) {
      if (t1) {
        t1 = false;
        if (dropDown2.value() == 0) {

          dropDown.open();
          dropDown2.open();
        } else {
          dropDown.close();
          dropDown2.close();
        }
      }
    } else {
      t1 = true;
    }
    // Intake Control
    
    if (con.ButtonR1.pressing()) {
      intake.spin(fwd, 12000, vex::voltageUnits::mV);
    } else if (con.ButtonR2.pressing()) {
      intake.spin(reverse, 12000, vex::voltageUnits::mV);
    } else {
      intake.stop(hold);
    }
     if (con.ButtonL1.pressing()) {
      if (t5) {
        t5 = false;
        if(rapid==1) {
          rapid=0;
        } else {
          rapid=1;
        }
      }
    } else {
      t5 = true;
    }
    if (con.ButtonL2.pressing()) {
      if (t6) {
        t6 = false;
        if(release.value()==1) {
          rapid=4;
        } else {
          release.open();
        }
      }
    } else {
      t6 = true;
    }
    if (con.ButtonB.pressing()) {
      if (t4) {
        t4 = false;
        if (dropDown.value() == 0) {
          dropDown.open();
        } else {
          rapid=0;
          dropDown.close();
          dropDown2.close();
        }
      }
    } else {
      t4 = true;
    }
    //printf("%f\n", autonSelect.value(degrees));//
    printf("%f\t", lMotor1.power());
    printf("%f\n", rMotor1.power());
    wait(10, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
