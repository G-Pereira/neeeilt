#define SONAR_TRIG 7
#define SONAR_ECHO 2
#define SOUNDSPEED 340

#define DEBUG 0
bool readingFinished;
long echoTime;
long distanceTime;
long trigTime;

int state;
int echoState;
bool echoRead;

int b = 0;

// Functions
void echoChange();
void SendTrigger();
void SetState(int s);
long GetDistance();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(SONAR_ECHO, INPUT);

  state = 0;
  echoState = 0;
  Serial.print("Initial State: ");
  Serial.println(state);

  digitalWrite(SONAR_TRIG, 0);
  attachInterrupt(digitalPinToInterrupt(SONAR_ECHO), echoChange, CHANGE);

  readingFinished = false;
  echoRead = false;

  distanceTime = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  if(state == 0 && b == 'g') {
    SetState(1);
  } else if(state == 1) {
    SetState(2);
    readingFinished = false;
    echoRead = false;
  } else if(state == 2 && readingFinished) {
    SetState(0); 
  }

  if(state == 0) {
    if(Serial.available() > 0){
      b = Serial.read();
    }
  } else if(state == 1) { 

    SendTrigger(); 
    
  } else if (state == 2) {
    if(echoRead) {

      long dist = GetDistance();
      
      Serial.write("Distance Read: ");
      Serial.println(dist);
      readingFinished = true;
      
    }
  }
}

void SetState(int s) {
  
  state = s;
  Serial.print("State: ");
  Serial.println(s);
  
}

void echoChange() {

  if(digitalRead(SONAR_ECHO) == 1) {
    echoTime = micros();
    #if DEBUG
      Serial.println("Rising");
    #endif
    echoRead = false; 
    echoState = 1;
    return;
  } else if(digitalRead(SONAR_ECHO) == 0) {
    distanceTime = micros() - echoTime;
    #if DEBUG
      Serial.println("Falling");
    #endif
    echoRead = true; 
    echoState = 0;
    return;
  }
}

void SendTrigger() {
  digitalWrite(SONAR_TRIG, 0);
  digitalWrite(SONAR_TRIG, 1);
  trigTime = micros();
  while(micros() - trigTime < 10);
  #if DEBUG
    Serial.print("Sent Trigger: ");
    Serial.println(micros() - trigTime);
  #endif
  digitalWrite(SONAR_TRIG, 0);
}

long GetDistance() {

  long ret = 0;

  ret = distanceTime / 1000.0 / 2.0 * SOUNDSPEED;
  #if DEBUG
    Serial.print("Distance Time: ");
    Serial.println(distanceTime);
  #endif
  echoRead = false;
  distanceTime = 0;

  return ret;  
}
