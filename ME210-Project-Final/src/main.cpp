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
// Function Prototypes
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

IntervalTimer IRDetectionTimer;
IntervalTimer DEBUG_PrintDelayTimer;

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// State Definitions
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// TODO: Update this with our actual states
// typedef enum {
//   STATE_MOVE_FORWARD, STATE_MOVE_BACKWARD, STATE_STOPPED, STATE_TURN_RIGHT
// } States_t;


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
// Function Implementations
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
void setup() {
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

  // Serial.println(analogRead(PIN_IR));
  eventCheck();

}


/* Event-Checking function used in loop. 
Tests conditionals implemented in separate functions, which in turn
trigger callbacks to provide services to events */
void eventCheck() {
  if (TestForIR()) IRResp();
  else Serial.println("NOT DETECTED!!!!");
  if (TestForKey()) keyPressResp();
}


bool TestForKey() {
  static bool KeyEventOccurred = false;
  KeyEventOccurred = Serial.available();
  while (Serial.available()) {
    Serial.read();
  }
  return KeyEventOccurred;
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


void keyPressResp() {
  static uint8_t MOTOR_DIRECTION = HIGH;
  if (MOTOR_DIRECTION == HIGH) {
    MOTOR_DIRECTION = LOW;
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);
  } else {
    MOTOR_DIRECTION = HIGH;
    digitalWrite(MOTOR_DIR_PIN, MOTOR_DIRECTION);
  }
}

/* DEBUG FUNCTION: This function is triggered by PrintDelayTimer 
so that test outputs can be printed to the serial monitor at a configurable rate
so it's easier to follow */
void DEBUG_printStuff(){
  // Serial.println(analogRead(PIN_IR));
}
