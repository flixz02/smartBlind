#include <Arduino.h>

// Motor parameters
#define MOTOR_STEPS 2000
#define RPM 12
#define MICROSTEPS 1

// Pins
#define DIR 10
#define STEP 20
#define SLEEP 21

#define BUTTON_UP 3
#define BUTTON_DOWN 4

#include "BasicStepperDriver.h"
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

uint8_t step = 0;
#define STATE_NOTHING 0
#define STATE_UP 1
#define STATE_DOWN 2

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(12);

  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(HIGH);

  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
}

void loop() {
  switch (step) {
    case STATE_NOTHING:
      if (digitalRead(BUTTON_UP) == LOW) {
        stepper.enable();
        stepper.startMove(50 * MOTOR_STEPS * MICROSTEPS);
        step = STATE_UP;
      } 
      else if (digitalRead(BUTTON_DOWN) == LOW) {
        stepper.enable();
        stepper.startMove(-50 * MOTOR_STEPS * MICROSTEPS);
        step = STATE_DOWN;
      }
      break;
    
    case STATE_UP:
      if (digitalRead(BUTTON_DOWN) == LOW) {
        step = STATE_NOTHING;
        stepper.stop();
      }
      break;

    case STATE_DOWN:
      if (digitalRead(BUTTON_UP) == LOW) {
        step = STATE_NOTHING;
        stepper.stop();
      }
      break;
  }

  // motor control loop - send pulse and return how long to wait until next pulse
  unsigned wait_time_micros = stepper.nextAction();

  // 0 wait time indicates the motor has stopped
  if (wait_time_micros <= 0) {
    stepper.disable();       // comment out to keep motor powered
    delay(500);
  }

  // (optional) execute other code if we have enough time
  if (wait_time_micros > 100){
    // put your main code here, to run repeatedly:
    // read the analog / millivolts value for pin 0:
    int analogVolts = analogReadMilliVolts(0);

    if (analogVolts > 60){
      stepper.disable();
    }
  }
}

// put function definitions here: