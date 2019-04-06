/*************************************************
* Etapa 1
* - Sensor sonar lateral le constantemente a distancia e guarda num vetor os valores obtidos.
*   Usou-se a biblioteca RunningMedian para calcular a Mediana desse vetor (maneira de filtrar os valores recebidos).
* - Controlo em PID do leme com o setpoint a metade da largura da pista
* - Controlo da velocidade como função do erro do controlador PID
*************************************************
* Created on: Apr 06, 2016
* Author: Cominhos
*************************************************/
 
#include <Servo.h>
#include <AutoPID.h>
#include <RunningMedian.h>

// DEFINES
#define SOUND_SPEED 340

#define N_SAMPLES 7

#define SONAR1_TRIG 7
#define SONAR1_ECHO 2

#define ON_PIN 12
#define MOTOR_FRONT 5 // Cor - Amarelo
#define MOTOR_BACK 6
#define SERVO 9

#define DEBUG 0

double DISTANCE_REF = 150;

Servo myservo;

double distance = 0;
double servo_pos = 90;

typedef struct {

  int echoState;
  bool readingFinished;
  bool echoRead;
  long echoTime;
  long distanceTime;
  long trigTime;

  int ECHO;
  int TRIG;

  int nReads;

} sonar_t;

RunningMedian s1distances = RunningMedian(N_SAMPLES);

int state;
int b = 0;
bool on;
sonar_t s1;

// Functions
void echoChange();
void SendTrigger(sonar_t s);
void SetState(int s);
int GetDistance(sonar_t s);
sonar_t initializeSonar(sonar_t s, int TRIG, int ECHO);

AutoPID pid(&distance, &DISTANCE_REF, &servo_pos, 60.0, 130.0, 3.5, 1, 1);

void setup() {
  // put your setup code here, to run once:

  distance = 0;
  state = 0;
  on = false;
  pinMode(ON_PIN, INPUT_PULLUP);
  digitalWrite(ON_PIN, 1);

  pinMode(MOTOR_BACK, OUTPUT);
  pinMode(MOTOR_FRONT, OUTPUT);
  myservo.attach(SERVO);

  pid.setBangBang(200);
  pid.setTimeStep(10);

#if DEBUG
  Serial.begin(115200);
  Serial.print("Initial State: ");
  Serial.println(state);
#endif
  // Sonar 1 definitions

  s1 = initializeSonar(s1, SONAR1_TRIG, SONAR1_ECHO);
  attachInterrupt(digitalPinToInterrupt(s1.ECHO), echoChange1, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(ON_PIN) == 0 && !on) {
    delay(5000);
    on = true;
  }

  if (digitalRead(ON_PIN) == 1 && on) {
    on = false;
    setMotor(0);
  }
  
  if (state == 0 && on) {
    SetState(1);
  } else if (state == 1) {
    SetState(2);
  } else if (state == 2 && s1.readingFinished) {
    SetState(1);
  }

  if (on) {

    if (state == 1) {

      SendTrigger(s1);

    } else if (state == 2) {

      int dist = GetDistance(s1);
      if (dist != -1)
        s1.readingFinished = true;

    }

    pid.run();
    setMotor(pid.atSetPoint(20) ? 150 : 75);
    myservo.write(servo_pos);

  }
}

// Sets the State
void SetState(int s) {

  state = s;
#if DEBUG
  Serial.print("State: ");
  Serial.println(s);
#endif
}

// ISR for chenge interrupt on Sonar ECHO pin
void echoChange1() {

  if (digitalRead(s1.ECHO) == 1) {
    s1.echoTime = micros();
#if DEBUG
    Serial.println("Rising");
#endif
    s1.echoRead = false;
    s1.echoState = 1;
    return;
  } else if (digitalRead(s1.ECHO) == 0) {
    s1.distanceTime = micros() - s1.echoTime;
#if DEBUG
    Serial.println("Falling");
#endif
    s1.echoRead = true;
    s1.echoState = 0;
    return;
  }
}

// Send trigger pulse
void SendTrigger(sonar_t s) {
  digitalWrite(s.TRIG, 0);
  digitalWrite(s.TRIG, 1);
  s.trigTime = micros();
  while (micros() - s.trigTime < 10);
#if DEBUG
  Serial.print("Sent Trigger: ");
  Serial.println(micros() - s.trigTime);
#endif
  digitalWrite(s.TRIG, 0);
  s.readingFinished = false;
  s.echoRead = false;
}

// Gets a distance read from the sonar and adds it to the array
int GetDistance(sonar_t s) {

  int ret = -1;

  // 240 micros

  if (s.echoRead) {

    ret = s.distanceTime / 1000.0 / 2.0 * SOUND_SPEED;

#if DEBUG
    Serial.print("Distance Time: ");
    Serial.println(s.distanceTime);

    Serial.print("Distance Read: ");
    Serial.println(ret);
#endif

    s.echoRead = false;
    s.distanceTime = 0;
    s1distances.add(ret);
    distance = s1distances.getMedian();
    ret = 0;
  }

  return ret;
}

// Initialize Sonar variables
sonar_t initializeSonar(sonar_t s, int TRIG, int ECHO) {

  s.echoState = 0;
  s.readingFinished = false;
  s.echoRead = false;
  s.distanceTime = 0;
  s.trigTime = 0;
  s.echoTime = 0;
  s.ECHO = ECHO;
  s.TRIG = TRIG;

  pinMode(s.TRIG, OUTPUT);
  pinMode(s.ECHO, INPUT);

  digitalWrite(s.TRIG, 0);

  return s;

}

// Set Motorspeed 
void setMotor(int motorSpeed) {

  analogWrite(MOTOR_BACK, 0);
  analogWrite(MOTOR_FRONT, 0);

  if(motorSpeed > 254)
    motorSpeed = 254;
  else if (motorSpeed < -254)
    motorSpeed = -254;

  analogWrite(MOTOR_BACK, (motorSpeed >= 0 ? motorSpeed : 0));
  analogWrite(MOTOR_FRONT, (motorSpeed >= 0 ? 0 : -motorSpeed));

}
