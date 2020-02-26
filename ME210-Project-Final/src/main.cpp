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
void eventCheck();
bool TestForKey();
void keyPressResp();
bool TestForIR();
void IRResp();
void IRTimerExp();
void IRTurnedOff();
static uint16_t VOLTAGE_TOGGLE_FREQ = 128;
uint8_t isIRDetected = false;
void DEBUG_printStuff();

void handleOrientation();
void handleDriveFwd();
void handleDriveRev();
void handleWallPush();
void handleTurnCW();
void handleTurnCCW();
void handleStop();


IntervalTimer IRDetectionTimer;
IntervalTimer DEBUG_PrintDelayTimer;

States_t state;



///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Core Function Implementations
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {
  // Initialize the state to orientation
  state = STATE_ORIENT;

  // Start timers
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
}

void handleDriveFwd() {
}

void handleDriveRev() {
}

void handleDriveCW() {
}

void handleDriveCCW() {
}

void handleWallPush() {
}

void handleStop() {
}


bool TestForIR(){
  // is it over some threshold, e.g. 100MV?
  return analogRead(PIN_IR) >= 300;
}



void IRResp(){
  Serial.println("IR DETECTED!");
  // isIRDetected = true;
  // IRDetectionTimer.end();
  // IRDetectionTimer.begin(IRTimerExp, IR_SIGNAL_INTERVAL);
}


void IRTimerExp(){
  if (isIRDetected) IRTurnedOff(); 
  isIRDetected = false;
}


void IRTurnedOff(){
  // DO SHIT WITH MOTOR
  Serial.println("THE LIGHT HAS BEEN TURNED OFF!! :D");
}



/* DEBUG FUNCTION: This function is triggered by PrintDelayTimer 
so that test outputs can be printed to the serial monitor at a configurable rate
so it's easier to follow */
void DEBUG_printStuff(){
  // Serial.println(analogRead(PIN_IR));
}
