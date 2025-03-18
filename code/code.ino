#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <Ewma.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include "FastAccelStepper.h"
#include "button.h"

// Settings
#define SLOW_BOOT 0
#define HOSTNAME "smartBlind1"
#define WIFI_SSID "FRITZ!Box 7560 2.4G"
#define WIFI_PASSWORD "20104412208840283021"

// Delays
#define MEASSURE_DELAY 400

// Analog
#define CONVERSIONS_PER_PIN 500
volatile bool adc_coversion_done = false;

adc_continuous_data_t *result = NULL;
void ARDUINO_ISR_ATTR adcComplete()
{
  adc_coversion_done = true;
}

uint8_t adc_pins[] = {0};
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

// Motor parameters
#define MOTOR_STEPS 2000
#define RPM 14
#define MICROSTEPS 1
#define ACCEL_TIME 2

// statemachine
#define STATE_NOTHING 0
#define STATE_UP 1
#define STATE_DOWN 2

// Pins
#define DIR_PIN 10
#define ENABLE_PIN 21
#define STEP_PIN 20
#define BUTTON_UP 3
#define BUTTON_DOWN 4

// Buttons
Button btnUP;
Button btnDWN;

// Current Meassurement
Ewma adcFilter(0.1);
#define LOOKBACK_DISTANCE 4

// Stepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
uint8_t endstopVoltage = 100;

// Function Prototypes
bool stopMotor();
void connectWifi();

void setup()
{
  // Init Buttons
  btnUP.begin(BUTTON_UP);
  btnDWN.begin(BUTTON_DOWN);

  Serial.begin(9600);
  delay(200);
  Serial.println("Serial started");

  // Analog setup
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_0db);
  // Setup ADC Continuous with following input:
  // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 50000, &adcComplete);

  randomSeed(0);
  connectWifi();

  // Stepper setup
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper)
  {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN, false);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    stepper->setDelayToEnable(10);
    stepper->setDelayToDisable(10);

    stepper->setSpeedInUs(2500); // the parameter is us/step !!!
    stepper->setAcceleration(100000);
  }
}

void loop()
{
  static uint8_t step = STATE_NOTHING;

  switch (step)
  {
  case STATE_NOTHING:
    if (btnUP.debounce())
    {
      stepper->runBackward();
      analogContinuousStart();
      step = STATE_UP;
    }
    else if (btnDWN.debounce())
    {
      stepper->runForward();
      analogContinuousStart();
      step = STATE_DOWN;
    }
    break;

  case STATE_UP:
    if (btnDWN.debounce())
    {
      stepper->stopMove();
      step = STATE_NOTHING;
    }

    if (stopMotor())
    {
      analogContinuousStop();
      stepper->stopMove();
      step = STATE_NOTHING;
    }
    break;

  case STATE_DOWN:
    if (btnUP.debounce())
    {
      stepper->stopMove();
      step = STATE_NOTHING;
    }

    if (stopMotor())
    {
      analogContinuousStop();
      stepper->stopMove();
      stepper->move(-2000);
      step = STATE_NOTHING;
    }
    break;
  }
}

// Fuction to stop motor when physical limit is reached, return true if motor is stopped
bool stopMotor()
{
  bool stopped = false;

  if (adc_coversion_done)
  {
    // Set ISR flag back to false
    adc_coversion_done = false;
    // Read data from ADC
    if (analogContinuousRead(&result, 0))
    {
      yield();
      // Optional: Stop ADC Continuous conversions to have more time to process (print) the data
      analogContinuousStop();

      static float meassurements[10];
      static uint8_t writeIndex = 0;

      meassurements[writeIndex] = adcFilter.filter(result[0].avg_read_raw);

      // find previous entry for comparison
      uint8_t prevIndex = writeIndex - LOOKBACK_DISTANCE < 0 ? writeIndex - LOOKBACK_DISTANCE + 10 : writeIndex - LOOKBACK_DISTANCE;

      float deltaV = meassurements[writeIndex] - meassurements[prevIndex];
      Serial.print(deltaV);
      Serial.print(", ");
      Serial.println(meassurements[writeIndex]);

      if (deltaV > 10)
      {
        if (meassurements[writeIndex] > 250)
        {
          stopped = true;
        }
      }

      // Serial.printf("\n %.1f", meassurements[writeIndex]);

      if (writeIndex >= 9)
      {
        writeIndex = 0;
      }
      else
      {
        writeIndex++;
      }

      // Optional: If ADC was stopped, start ADC conversions and wait for callback function to set adc_coversion_done flag to true
      analogContinuousStart();
    }
    else
    {
      Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
    }
  }
  return stopped;
}

void connectWifi()
{
  int connect_timeout;
  WiFi.setHostname(HOSTNAME);
  Serial.println("Begin wifi...");

  yield();

  // Try to connect with stored credentials, fire up an access point if they don't work.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  connect_timeout = 28; // 7 seconds
  while (WiFi.status() != WL_CONNECTED && connect_timeout > 0)
  {
    delay(40);
    Serial.print(".");
    connect_timeout--;
  }
}