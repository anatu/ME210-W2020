///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Notes
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Encoders read 341.2 counts per full revolution of the gearbox output shaft

// When driving motors, DO NOT MIX DIGITAL AND ANALOG WRITE COMMANDS! Everything should be analog.
// If you need to write digital low to a motor pin just analogWrite 0.



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
#define P_LMOTOR_IN1 22
#define P_LMOTOR_IN2 23
#define P_LMOTOR_ENC_A 1 
#define P_LMOTOR_ENC_B 2

// Right Motor Pins
#define P_RMOTOR_IN1 4
#define P_RMOTOR_IN2 3
// #define P_RMOTOR_ENC_A 
// #define P_RMOTOR_ENC_B

// Sensor Pins
#define P_IR_SENSOR A2
#define P_LINE_OUTER 11
#define P_LINE_INNER 12
#define P_BACK_LINE 7
#define P_LED 5

// Other Constants
// Sensing interval for the IR sensor
// #define IR_SIGNAL_INTERVAL 1000000
// Global 2:10 stopping timer
#define GLOBAL_TIME_STOP_INTERVAL 130000000
// Nominal motor speed
#define NOMINAL_SPEED 250


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// State Definitions
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
typedef enum {
  STATE_ORIENT, STATE_DRIVE_FWD, STATE_DRIVE_REV, STATE_TURN_CW,
  STATE_SHARP_TURN_CW, STATE_TURN_CCW, STATE_SHARP_TURN_CCW, STATE_STOPPED, STATE_PUSHING_WALL
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

bool TestOuterLine();
bool TestInnerLine();
bool TestBackLine();

void setLeftMotorSpeed(int16_t);
void setRightMotorSpeed(int16_t);


void handleOrientation();
void handleDriveFwd();
void handleDriveRev();
void handleWallPush();
void handleTurnCW();
void handleTurnCCW();
void handleSharpTurnCW();
void handleSharpTurnCCW();
void handleStop();

void GlobalStop();
void ReverseCWTimerExp();

void DEBUG_printStuff();

// Module variables
States_t state;

uint8_t isIRDetected = false;

IntervalTimer DEBUG_PrintDelayTimer;
IntervalTimer GlobalStopTimer;
IntervalTimer ReverseCWTimer;



///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Core Function Implementations
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {

  // Timers
  GlobalStopTimer.begin(GlobalStop, GLOBAL_TIME_STOP_INTERVAL);
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
  // pinMode(P_RMOTOR_ENC_A, INPUT);
  // pinMode(P_RMOTOR_ENC_B, INPUT);

  // Motor Encoder Pin Interrupts
  // attachInterrupt(digitalPinToInterrupt(P_LMOTOR_ENC_A), countRisingEdges, RISING);
  // attachInterrupt(digitalPinToInterrupt(P_LMOTOR_ENC_B), countRisingEdges, RISING);
  // attachInterrupt(digitalPinToInterrupt(P_RMOTOR_ENC_A), countRisingEdges, RISING);
  // attachInterrupt(digitalPinToInterrupt(P_RMOTOR_ENC_B), countRisingEdges, RISING);

  // Sensor Pins
  pinMode(P_IR_SENSOR, INPUT);
  pinMode(P_LINE_OUTER, INPUT);
  pinMode(P_LINE_INNER, INPUT);
  pinMode(P_BACK_LINE, INPUT);
  

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
    case STATE_SHARP_TURN_CW:
      handleSharpTurnCW();
      break;
    case STATE_SHARP_TURN_CCW:
      handleSharpTurnCCW();
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


///////////////////////////////////////////////////////
// Utility Functions
///////////////////////////////////////////////////////


/* Helper functions for driving motors. 
Accepts a speed in the range of -255-255 (as per 
limits on the analogWrite command for PWM signal generation) 
and drives the motor at that speed. If the argument is negative it drives 
in the opposite direction */
void setLeftMotorSpeed(int16_t speed) {

  if (speed == 0) {
    analogWrite(P_LMOTOR_IN1, 0);
    analogWrite(P_LMOTOR_IN2, 0);
  }
  else if (speed < 0) {
    analogWrite(P_LMOTOR_IN1, 0);
    analogWrite(P_LMOTOR_IN2, -1*speed);
  }
  else {
    analogWrite(P_LMOTOR_IN1, speed);
    analogWrite(P_LMOTOR_IN2, 0);
  }
}

void setRightMotorSpeed(int16_t speed) {

  if (speed == 0) {
    analogWrite(P_RMOTOR_IN1, 0);
    analogWrite(P_RMOTOR_IN2, 0);
  }
  else if (speed < 0) {
    analogWrite(P_RMOTOR_IN1, 0);
    analogWrite(P_RMOTOR_IN2, -1*speed);
  }
  else {
    analogWrite(P_RMOTOR_IN1, speed);
    analogWrite(P_RMOTOR_IN2, 0);
  }
}


///////////////////////////////////////////////////////
// Event / Service Functions
///////////////////////////////////////////////////////

/* Universal Event-Checking function used in loop. 
Tests conditionals implemented in separate functions, which in turn
trigger callbacks to provide services to events */
void eventCheck() {
  // IR Sensing Events
  if (TestForIR()) IRResp();
 
  // Line-Sensing Events for Forward-Drive Line Following
  if (state == STATE_TURN_CW){
    if (TestOuterLine()) {
      ReverseCWTimerExp();
    }
  }

  if (state != STATE_ORIENT) {
    if (TestOuterLine() && TestInnerLine()) state = STATE_DRIVE_FWD;
    if (TestOuterLine()) state = STATE_TURN_CCW;
    if (TestInnerLine() && !TestOuterLine()) state = STATE_SHARP_TURN_CCW;

    // LEGACY CODE - GET RID OF THIS
    // if (TestBackOuterLine() && TestBackInnerLine()) state = STATE_DRIVE_REV;
    // if (!TestBackOuterLine() && !TestBackInnerLine() && (state == STATE_TURN_CW || state == STATE_SHARP_TURN_CW)) state = STATE_DRIVE_REV;

    if (TestBackLine() && (state != STATE_TURN_CW)) {
      Serial.println("TIMER STARTED");
      state = STATE_TURN_CW;
      //ReverseCWTimer.begin(ReverseCWTimerExp, 500000);      
      }
    }
}

void ReverseCWTimerExp(){
  Serial.println("TIMER EXPIRED");
  state = STATE_DRIVE_REV;
  // setLeftMotorSpeed(0);
  // setRightMotorSpeed(0);
  //ReverseCWTimer.end();
}


bool TestOuterLine(){
  return digitalRead(P_LINE_OUTER);
}

bool TestInnerLine(){
  return digitalRead(P_LINE_INNER);
}


bool TestBackLine(){
  return digitalRead(P_BACK_LINE);
}



bool TestForIR(){
  // Tests if the IR signal is high enough to count as being "detected"
  if (analogRead(P_IR_SENSOR) >= 800) {
    return 1;
  }
  else {
    return 0;
  }
}


void IRResp(){
  // If event takes place, print to Serial monitor
  // TODO: Start counting encoders here
  if (state == STATE_ORIENT) {
    setLeftMotorSpeed(0);
    setRightMotorSpeed(0);    
    state = STATE_DRIVE_REV;
  }
}



///////////////////////////////////////////////////////
// State-Handling Functions
///////////////////////////////////////////////////////


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

  setLeftMotorSpeed(NOMINAL_SPEED);
  setRightMotorSpeed(-NOMINAL_SPEED);

}


void handleDriveFwd() {
  /* Handles driving forward. Should just set left and right
  motor speeds to some equal value of speed. Need to calibrate this online */
  setLeftMotorSpeed(NOMINAL_SPEED);
  setRightMotorSpeed(NOMINAL_SPEED);
}

void handleDriveRev() {
  /* Handles driving Backward. Should just set left and right
  motor speeds to some equal value of speed. Need to calibrate this online */
  setLeftMotorSpeed(NOMINAL_SPEED);
  setRightMotorSpeed(NOMINAL_SPEED);
}

void handleTurnCW() {
  /* Handles driving Clockwise. Should be done by setting left motor
  forward and right motor too zero. */
  setLeftMotorSpeed(NOMINAL_SPEED);
  setRightMotorSpeed(0);
}

void handleSharpTurnCW() {
  /* Handles SHARP driving Clockwise. Should be done by setting left motor
  forward and right motor in reverse. */
  setLeftMotorSpeed(NOMINAL_SPEED);
  setRightMotorSpeed(-NOMINAL_SPEED/2);
}

void handleTurnCCW() {
  /* Handles driving Counter-Clockwise. Should be done by setting right motor
  forward and left motor in reverse. */
  setLeftMotorSpeed(-NOMINAL_SPEED);
  setRightMotorSpeed(0);
}

void handleSharpTurnCCW() {
  /* Handles SHARP driving Counter-Clockwise. Should be done by setting left motor
  forward and right motor in reverse. */
  setLeftMotorSpeed(-NOMINAL_SPEED);
  setRightMotorSpeed(NOMINAL_SPEED/2);
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


// void IRTimerExp(){
//   if (isIRDetected) {
//   Serial.println("NOT DETECTED!!");
//   isIRDetected = false;
//   Serial.println(edgeCounter);
//   edgeCounterRev = edgeCounter / 2;
//   edgeCounter = 0;
//   // setMotorSpeed(-150);
// }


void IRDetectionEnded(){
  /* Once the IR beacon falls out of range, we must 
  perform the "backwards pass" where we calculate the encoder 
  midpoint, and drive the motors in reverse orientation to recover
  the radial midpoint of the region where the IR signal was detected. 
  TODO: Consider using another boolean variable (e.g. isCorrecting) or another
  state altogether so the effect of sensing the IR beacon doesn't interfere
  when we are turning to this midpoint. */
  Serial.println("IR NOT DETECTED!");
}

void GlobalStop() {
  /* Callback function to force the stop state 
  after the 2:10 universal time limit has elapsed. 
  The stop state handling logic will stop the motors directly. */
  state = STATE_STOPPED;
  GlobalStopTimer.end();
}



/* DEBUG FUNCTION: This function is triggered by PrintDelayTimer 
so that test outputs can be printed to the serial monitor at a configurable rate
so it's easier to follow */
void DEBUG_printStuff(){
  // Serial.println(analogRead(PIN_IR));
}
