#include <Arduino.h>


#define IR_SENSE_INTERVAL 2000000

// Left Motor
int IN1 = 3;
int IN2 = 4;
int ENC_A = 1;
int ENC_B = 2;

// Motor 2
int IN3 = 23;
int IN4 = 22;

int IR_PIN = A2;
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
bool isIRDetected = false;
IntervalTimer myTimer;
IntervalTimer LEDTimer;
IntervalTimer IRDetectionTimer;
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
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(ENC_A), countRisingEdges, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B), countRisingEdges, RISING);

  // LEDTimer.begin(toggleOutput, 500);
  // IRDetectionTimer.begin(IRTimerExp, IR_SENSE_INTERVAL);

  setLeftMotorSpeed(150);
  setRightMotorSpeed(150);

  // myTimer.begin(stopMotor, 5000000);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(edgeCounter);

  checkEvents();
}


void checkEvents() {
  if (TestForIR()) IRResp();
  // if (timerDone) timerDoneResp();
}


bool TestForIR(){
  // is it over some threshold, e.g. 100MV?
  return analogRead(IR_PIN) >= 300;
}

void IRResp(){
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