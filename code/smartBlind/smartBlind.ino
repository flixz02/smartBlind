#include <Stepper.h>
#include "LowPower.h"

const unsigned int stepsPerRev = 4096;  // change this to fit the number of steps per revolution for your motor
const int numberOfRevs = 23;

// Pin declarations 
const int switchInputUP = 3;
const int switchInputDOWN = 4;

bool commandUP;
bool commandDOWN;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(4096, 8, 10, 9, 11);

// turn up handler
void moveUP(){
  commandUP = !commandUP;
}

// turn down handler
void moveDOWN(){
  commandDOWN = !commandDOWN;
}

void setup() {
  // configure stepper speed to 6 RPM
  myStepper.setSpeed(6);
  // configure switchPins
  pinMode(switchInputUP, INPUT_PULLUP);
  pinMode(switchInputDOWN, INPUT_PULLUP);
}

void loop() {

  attachInterrupt(digitalPinToInterrupt(3), moveUP, LOW);
  attachInterrupt(digitalPinToInterrupt(4), moveDOWN, LOW);

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

  detachInterrupt(0); 

  if(commandUP){
    for(int i = 0; i < 23; ++i){
      myStepper.step(-stepsPerRev);
      if(!commandUP) break;
    }
  }

  if(commandDOWN){
    for(int i = 0; i < 23; ++i){
      myStepper.step(stepsPerRev);
      if(!commandDOWN) break;
    }
  }
}


