#include <Arduino.h>
#include <Wire.h>
#include <EVShield.h>
#include <EVs_EV3Gyro.h>

// NOTE find your own path for EVShield library in the platformio.ini file !!!

EVShield evshield(0x34,0x36); // initialize EVShield
EVs_EV3Gyro myGyro; // initialize a Gyro

void find_current_pos();
void drive_straight_for();

volatile float currentposx = 0;
volatile float currentposy = 0;

volatile float targetposx = 0;
volatile float targetposy = 0;

volatile float dist_from_target = 0;
volatile float targetangle = 0;

#define motorspeed 50 // 0 - 100
#define turnspeed 25 // Motor speed when turning
#define wheeldiameter 35 // Wheel diameter in millimeters
#define turnstops_per_second 50 //how many times code samples for angle per second

void drive_straight_for(float dist){ // for calculating amount of distance
  dist = (dist * 360) / (wheeldiameter * 3.1415); // find amount of degrees the wheel has to turn to reach dist (original) millimeters
  evshield.bank_a.motorRunDegrees(SH_Motor_Both, SH_Direction_Forward, motorspeed, dist, SH_Completion_Wait_For, SH_Next_Action_Brake);
}

float findangle(){ // you can clean it up if you wish
  float angle = 0;
    angle = (90 - (atan2(targetposy - currentposy, targetposx - currentposx) / PI * 180)); // calculate angle to aim at // can be optimized
  return angle;
}

void drive_to(float targetx, float targety){ // grid is in millimeters
  targetposx = targetx;
  targetposy = targety;
  targetangle = findangle();
  while (myGyro.getAngle() != targetangle){ // whenever targetangle is not reached, probably some play room can be given
    if (myGyro.getAngle() < targetangle){
      evshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Forward, turnspeed);
      evshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Reverse, turnspeed);
    }
    else if (myGyro.getAngle() > targetangle){
      evshield.bank_a.motorRunUnlimited(SH_Motor_1, SH_Direction_Reverse, turnspeed);
      evshield.bank_a.motorRunUnlimited(SH_Motor_2, SH_Direction_Forward, turnspeed);
    }
    delay(1000 / turnstops_per_second); // defines how many times code samples for angle
  }
  evshield.bank_a.motorStop(SH_Motor_Both, SH_Next_Action_Brake); // stop turning once angle has been reaced
  delay(50);
  drive_straight_for(sqrt(sq(currentposx - targetposx) + sq(currentposy - targetposy))); // calculating the drive distance using the Pythagorean theorem
  currentposx = targetposx;
  currentposy = targetposy;
}

void setup() {
  evshield.init(SH_HardwareI2C);
  evshield.bank_a.motorReset();
  evshield.bank_b.motorReset();
  myGyro.init( &evshield, SH_BAS1 ); // We tell where th Gyro is connected to
  myGyro.setMode(MODE_Gyro_Angle); // set angle mode
  delay(2000); // small delay (optional)
}

void loop() {
  drive_to(200, 600); // this and following 4 lines of code make the robot draw a star
  drive_to(400, 0);
  drive_to(-100, 400);
  drive_to(500, 400);
  drive_to(0,0);
  delay(5000);
}