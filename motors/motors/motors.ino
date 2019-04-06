#define motor_back 5
#define motor_front 6

#define FRONT 0
#define BACK 1

void setup() {
  pinMode(motor_back, OUTPUT);
  pinMode(motor_front, OUTPUT);
}

void loop() {
  setMotor(FRONT, 100);
}

void setMotor(uint8_t dir, uint8_t speed){
  analogWrite(motor_back, dir?speed:0);
  analogWrite(motor_front, dir?0:speed);
}
