#define motor_back 5
#include <Servo.h>

#define motor_front 6
#define SERVO 6

#define FRONT 1
#define BACK 0

Servo myservo;

void setMotor(uint8_t dir, uint8_t speed){
  analogWrite(motor_back, dir?speed:0);
  analogWrite(motor_front, dir?0:speed);
  myservo.attach(SERVO);
}

void setup() {
  pinMode(motor_back, OUTPUT);
  pinMode(motor_front, OUTPUT);
}

void loop() {
  setMotor(FRONT, 50);
  delay(1000);
  setMotor(FRONT, 0);
  delay(3000);
  setMotor(BACK, 50);
  delay(1000);
  setMotor(BACK, 0);
  delay(3000);

  myservo.write(0);
}
