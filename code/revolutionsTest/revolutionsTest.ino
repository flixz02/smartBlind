#include <Stepper.h>

const int stepsPerRevolution = 4096;  // change this to fit the number of steps per revolution for your motor
const int numberOfRevolutions = 2;

int count = 0;
const int switchInput = 7;
const int switchInputSingle = 6;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  // set the rpm:
  myStepper.setSpeed(6);
  // configure switchPin
  pinMode(switchInput, INPUT_PULLUP);
  pinMode(switchInputSingle, INPUT_PULLUP);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  if(!digitalRead(switchInput)){
    count = count + 2;
    Serial.println(count);
    myStepper.step(-stepsPerRevolution*numberOfRevolutions);
  }
  if(!digitalRead(switchInputSingle)){
    count = count + 1;
    Serial.println(count);
    myStepper.step(-stepsPerRevolution);
  }
}

