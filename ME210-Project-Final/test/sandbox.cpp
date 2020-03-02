#include <Arduino.h>


#define IR_SENSE_INTERVAL 2000000

// Left Motor
int IN1 = 4;
int IN2 = 3;
int ENC_A = 1;
int ENC_B = 2;

// Motor 2
int IN3 = 22;
int IN4 = 23;

int IR_PIN = A2;
int LINE_PIN_OUTER = 11;
int LINE_PIN_INNER = 12;
int LED_PIN = 5;

volatile unsigned long edgeCounter = 0;
volatile unsigned long edgeCounterRev = 0;

void countRisingEdges();
void setLeftMotorSpeed(int16_t);
void setRightMotorSpeed(int16_t);
void stopMotor();
void checkEvents();
void timerDoneResp();
bool TestForIR();
void IRResp();
void IRTimerExp();
void toggleOutput();
//bool lineDetected();
void LineResp();
void goForward();

void innerLineResp();
bool outerLineDetected();
bool innerLineDetected();



bool isIRDetected = false;
IntervalTimer myTimer;
IntervalTimer LEDTimer;
IntervalTimer IRDetectionTimer;
IntervalTimer LineDetectionTimer;
bool timerDone = false;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  pinMode(IR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(LINE_PIN_OUTER, INPUT);
  pinMode(LINE_PIN_INNER, INPUT);
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(ENC_A), countRisingEdges, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), countRisingEdges, RISING);

  // LEDTimer.begin(toggleOutput, 500);
  // IRDetectionTimer.begin(IRTimerExp, IR_SENSE_INTERVAL);
  delay(500);
  setLeftMotorSpeed(255);
  setRightMotorSpeed(-255);

  // myTimer.begin(stopMotor, 5000000);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(edgeCounter);

  checkEvents();
  //Serial.println(digitalRead(LINE_PIN_INNER));
  Serial.println(analogRead(IR_PIN));
}


void checkEvents() {
  if (TestForIR()) IRResp();

  //if (outerLineDetected()==0 && innerLineDetected()==0) goForward();
  //if (outerLineDetected()) LineResp();
  //if (outerLineDetected() && innerLineDetected()) LineResp();
  //if (innerLineDetected()==1 && outerLineDetected() == 0) innerLineResp();
  //if (timerDone) timerDoneResp();
}

bool outerLineDetected(){
  return digitalRead(LINE_PIN_OUTER);
}

bool innerLineDetected(){
  return digitalRead(LINE_PIN_INNER);
}

void LineResp(){
  //LineDetectionTimer.begin(goForward, 100000);
  setLeftMotorSpeed(255);
  setRightMotorSpeed(0);
  /*if(innerLineDetected() == 0){
    setLeftMotorSpeed(30);
    setRightMotorSpeed(-30);
  } else if(innerLineDetected()) {
    setLeftMotorSpeed(30);
    setRightMotorSpeed(-30);
  }*/
}


void innerLineResp() {
  setLeftMotorSpeed(255);
  setRightMotorSpeed(-150);
  while(outerLineDetected() == 0) {
  }
}

void goForward() {
  setRightMotorSpeed(255);
  setLeftMotorSpeed(255);
  //LineDetectionTimer.end();
}

bool TestForIR(){
  // is it over some threshold, e.g. 100MV?
  
  if( analogRead(IR_PIN) >= 800) {
    return 1;
  } else{
    return 0;
  }

}

void IRResp(){

  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
  if (isIRDetected == false) {
    Serial.println("IR DETECTED!");
    // setMotorSpeed(150);
    isIRDetected = true;
    edgeCounter = 0;
  }
  IRDetectionTimer.end();
  IRDetectionTimer.begin(IRTimerExp, IR_SENSE_INTERVAL);
}

void IRTimerExp() {
  if (isIRDetected) {
  Serial.println("NOT DETECTED!!");
  isIRDetected = false;
  Serial.println(edgeCounter);
  edgeCounterRev = edgeCounter / 2;
  edgeCounter = 0;
  // setMotorSpeed(-150);
  }
}

void toggleOutput() {
  if (digitalRead(LED_PIN)) {
    // do this if C2 is high
    digitalWrite(LED_PIN, LOW);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}


void timerDoneResp() {
  if (edgeCounter >= edgeCounterRev) {
    // digitalWrite(IN1, LOW);
    // analogWrite(IN2, 0);
    // setMotorSpeed(0);
    timerDone = false;
    Serial.println(edgeCounter);
  }
}

/* Helper functions for driving motors. 
Accepts a speed in the range of -255-255 (as per 
limits on the analogWrite command for PWM signal generation) 
and drives a motor at that speed. If the argument is negative it drives 
in the opposite direction */
void setLeftMotorSpeed(int16_t speed) {

  if (speed == 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
  else if (speed < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, -1*speed);
  }
  else {
    analogWrite(IN1, speed);
    analogWrite(IN2, 0);
  }
}

void setRightMotorSpeed(int16_t speed) {

  if (speed == 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
  else if (speed < 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, -1*speed);
  }
  else {
    analogWrite(IN3, speed);
    analogWrite(IN4, 0);
  }
}

void stopMotor() {
  // setMotorSpeed(0);
  // setMotorSpeed(100);

  myTimer.end();
  edgeCounterRev = edgeCounter / 2;
  Serial.println(edgeCounter);
  Serial.println(edgeCounterRev);
  edgeCounter = 0;
  timerDone = true;
}

void countRisingEdges() {
  edgeCounter++;
}