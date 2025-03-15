/*
 * Simple demo, should work with any driver board
 *
 * Connect STEP, DIR as indicated
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 2000
#define RPM 13

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// All the wires needed for full functionality
#define DIR 8
#define STEP 9
#define SLEEP 10

#define PIN_UP 5
#define PIN_DOWN 6 

#define LED 4

#include "BasicStepperDriver.h"
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

uint8_t step = 0;
#define STATE_NOTHING 0
#define STATE_UP 1
#define STATE_DOWN 2

void setup() {
  stepper.begin(RPM, MICROSTEPS);
  stepper.setEnableActiveState(HIGH);

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
}

void loop() {
  switch (step) {
    case STATE_NOTHING:
      digitalWrite(LED, LOW);
      if (digitalRead(PIN_UP) == LOW) {
        stepper.enable();
        stepper.startMove(50 * MOTOR_STEPS * MICROSTEPS);
        step = STATE_UP;
      } 
      else if (digitalRead(PIN_DOWN) == LOW) {
        stepper.enable();
        stepper.startMove(-50 * MOTOR_STEPS * MICROSTEPS);
        step = STATE_DOWN;
      }
      break;
    
    case STATE_UP:
      digitalWrite(LED, HIGH);
      if (digitalRead(PIN_DOWN) == LOW) {
        step = STATE_NOTHING;
        stepper.stop();
      }
      break;

    case STATE_DOWN:
      digitalWrite(LED, HIGH);
      if (digitalRead(PIN_UP) == LOW) {
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
      // other code here
  }
}
