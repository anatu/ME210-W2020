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
#define PIN_SIGNAL_IN 7
#define MOTOR_PWM_PIN 4
#define MOTOR_DIR_PIN 3
#define PIN_MOTOR_ENC_A 9
#define PIN_MOTOR_ENC_B 10
#define PIN_IR A3

// Other Constants
#define IR_SIGNAL_INTERVAL 1000

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

  // Set the modes of the pins we are using 
  // Motor Voltage-Driving Pins
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, VOLTAGE_TOGGLE_FREQ);

  // Motor Direction Pins
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  digitalWrite(MOTOR_DIR_PIN, HIGH);

  // Motor Encoder Pins


  // Sensor Pins
  pinMode(PIN_IR, INPUT);

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
// TODO: Replace this with two functions for L/R motors
void setMotorSpeed(int16_t speed) {

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
