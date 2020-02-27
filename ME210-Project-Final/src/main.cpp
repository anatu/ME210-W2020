///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Notes
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Encoders read 341.2 counts per full revolution of the gearbox output shaft
// When driving motors, DO NOT MIX DIGITAL AND ANALOG WRITE COMMANDS!

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

#include <Arduino.h>

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Define Declarations
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Pin Numbers
// TODO: Fill these out!
// Left Motor Pins
#define P_LMOTOR_IN1
#define P_LMOTOR_IN2
#define P_LMOTOR_ENC_A 
#define P_LMOTOR_ENC_B

// Right Motor Pins
#define P_RMOTOR_IN1
#define P_RMOTOR_IN2
#define P_RMOTOR_ENC_A 
#define P_RMOTOR_ENC_B

// Sensor Pins
#define P_IR_SENSOR A3
#define P_L_LINE_SENSOR
#define P_C_LINE_SENSOR
#define P_R_LINE_SENSOR

// Other Constants
// Sensing interval for the IR sensor
#define IR_SIGNAL_INTERVAL 1000000

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// State Definitions
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
typedef enum {
  STATE_ORIENT, STATE_DRIVE_FWD, STATE_DRIVE_REV, STATE_TURN_CW,
  STATE_TURN_CCW, STATE_STOPPED, STATE_PUSHING_WALL
} States_t;


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Function Prototypes and Module Variables
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Function prototypes
void eventCheck();

bool TestForIR();
void IRResp();
void IRTimerExp();
void IRDetectionEnded();


void setLeftMotorSpeed(int16_t);
void setRightMotorSpeed(int16_t);


void handleOrientation();
void handleDriveFwd();
void handleDriveRev();
void handleWallPush();
void handleTurnCW();
void handleTurnCCW();
void handleStop();

void DEBUG_printStuff();

// Module variables
States_t state;

uint8_t isIRDetected = false;

IntervalTimer IRDetectionTimer;
IntervalTimer DEBUG_PrintDelayTimer;



///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Core Function Implementations
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {
  // Initialize the state to orientation
  state = STATE_ORIENT;

  // Timers
  IRDetectionTimer.begin(IRTimerExp, IR_SIGNAL_INTERVAL);
  DEBUG_PrintDelayTimer.begin(DEBUG_printStuff, 100000);

  // Pin Settings
  // Motor PWM Pins
  pinMode(P_LMOTOR_IN1, OUTPUT);
  pinMode(P_LMOTOR_IN2, OUTPUT);
  pinMode(P_RMOTOR_IN1, OUTPUT);
  pinMode(P_RMOTOR_IN2, OUTPUT);

  // Motor Encoder Pins
  pinMode(P_LMOTOR_ENC_A, INPUT);
  pinMode(P_LMOTOR_ENC_B, INPUT);
  pinMode(P_RMOTOR_ENC_A, INPUT);
  pinMode(P_RMOTOR_ENC_B, INPUT);

  // Motor Encoder Pin Interrupts
  attachInterrupt(digitalPinToInterrupt(P_LMOTOR_ENC_A), countRisingEdges, RISING);
  attachInterrupt(digitalPinToInterrupt(P_LMOTOR_ENC_B), countRisingEdges, RISING);
  attachInterrupt(digitalPinToInterrupt(P_RMOTOR_ENC_A), countRisingEdges, RISING);
  attachInterrupt(digitalPinToInterrupt(P_RMOTOR_ENC_B), countRisingEdges, RISING);

  // Sensor Pins
  pinMode(P_IR_SENSOR, INPUT);
  pinMode(P_L_LINE_SENSOR, INPUT);
  pinMode(P_R_LINE_SENSOR, INPUT);
  pinMode(P_C_LINE_SENSOR, INPUT);


  // Initialize our state to the orientation pass
  state = STATE_ORIENT;

}


/* Core Function Loop */
void loop() {

  // TODO: State machine logic goes in here

  eventCheck();

  switch(state) {
    case STATE_ORIENT:
      handleOrientation();
      break;
    case STATE_DRIVE_FWD:
      handleDriveFwd();
      break;
    case STATE_DRIVE_REV:
      handleDriveRev();
      break;
    case STATE_PUSHING_WALL:
      handleWallPush();
      break;
    case STATE_TURN_CW:
      handleTurnCW();
      break;
    case STATE_TURN_CCW:
      handleTurnCCW();
      break;
    case STATE_STOPPED:
      handleStop();
      break;
  }

}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Module Function Implementations
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

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



/* Universal Event-Checking function used in loop. 
Tests conditionals implemented in separate functions, which in turn
trigger callbacks to provide services to events */
void eventCheck() {
  if (TestForIR()) IRResp();
  else Serial.println("NOT DETECTED!!!!");
  if (TestForKey()) keyPressResp();
}

void handleOrientation() {
  /* Handles the initial orientation pass where we
  center the robot around the IR beacon in our section of the arena.
  Strategy: Rotate until IR is detected, start encoders, keep rotating until IR is no
  longer detected, end encoder count, and reverse turn to half the encoder count so the robot
  is centered in the position between where the IR signal starts and stops being detected.

  For the edge case where we start in a position where the IR sensor is already in view, we 
  can either (1) turn until we don't see it, and do the orientation pass normally, or (2) if we
  are confident of the angular error of our pickup signal, simply start at localized and go from there
  (i.e. if we are fairly sure that the IR signal is only detected when the sensor is nearly directly facing the beacon) */

}

void handleDriveFwd() {
  /* Handles driving forward. Should just set left and right
  motor speeds to some equal value of speed. Need to calibrate this online */
}

void handleDriveRev() {
  /* Handles driving Backward. Should just set left and right
  motor speeds to some equal value of speed. Need to calibrate this online */
}

void handleDriveCW() {
  /* Handles driving Clockwise. Should be done by setting left motor
  faster than right to steer differentially. Needs online calibration */
}

void handleDriveCCW() {
  /* Handles driving Counter-Clockwise. Should be done by setting right motor
  faster than left to steer differentially. Needs online calibration */
}

void handleWallPush() {
  /* Handles pushing the wall. TODO: Still need to figure out a) whether
  we want sensing to detect the walls at all, and b) whether we need this
  state at all or we just want to set timers */
}

void handleStop() {
  /* Handles the stopped state. Simply turn both motors off */
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);
}


bool TestForIR(){
  // Tests if the IR signal is high enough to count as being "detected"
  return analogRead(P_IR_SENSOR) >= 300;
}

void IRResp(){
  Serial.println("IR DETECTED!");
  // isIRDetected = true;
  // IRDetectionTimer.end();
  // IRDetectionTimer.begin(IRTimerExp, IR_SIGNAL_INTERVAL);
}


void IRTimerExp(){
  if (isIRDetected) IRDetectionEnded(); 
  isIRDetected = false;
}


void IRDetectionEnded(){
  // DO SHIT WITH MOTOR
  Serial.println("IR NOT DETECTED");
}



/* DEBUG FUNCTION: This function is triggered by PrintDelayTimer 
so that test outputs can be printed to the serial monitor at a configurable rate
so it's easier to follow */
void DEBUG_printStuff(){
  // Serial.println(analogRead(PIN_IR));
}
