#include <AutoPID.h>
#include <Servo.h>

// CONFIGS
double DISTANCE_REF = 10;

// PINS
#define motor_front 5
#define motor_back 6
#define SERVO 8

Servo myservo;

double distance = 0;
double servo_pos = 0;

AutoPID pid(&distance, &DISTANCE_REF, &servo_pos, 0, 180, 2, 2, 2);

void setMotor(int motorSpeed){
  analogWrite(motor_back, motorSpeed>=0?motorSpeed:0); //  TODO: map
  analogWrite(motor_front, motorSpeed>=0?0:-motorSpeed);
}

void setup() {
  pinMode(motor_back, OUTPUT);
  pinMode(motor_front, OUTPUT);
  myservo.attach(SERVO);
}

void loop() {
  pid.run();
  setMotor(myPID.atSetPoint(2)?100:50);
  myservo.write(servo_pos);
}
