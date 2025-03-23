#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <Ewma.h>
#include <EEPROM.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include "FastAccelStepper.h"
#include "button.h"

// Settings
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

// EEPROM
// EEPROM_BLIND_STATE stores permanent state of the blind bitwise
//  Bit 0: first run done, will be reset when blind is reset
//  Bit 1: POSITION_UP set
//  Bit 2: POSITION_DOWN set
//  Bit 3: POSITION_LAST set
#define EEPROM_BLIND_STATE 0
#define STATE_FIRST_RUN_DONE 0
#define STATE_POSITION_UP_SET 1
#define STATE_POSITION_DOWN_SET 2
#define STATE_POSITION_LAST_SET 3

// EEPROM_POSITION_UP stores the position of the upper limit
// EEPROM_POSITION_DOWN stores the position of the lower limit
// EEPROM_POSITION_LAST stores the last known position of the stepper
#define EEPROM_POSITION_UP 4
#define EEPROM_POSITION_DOWN 8
#define EEPROM_POSITION_LAST 12

bool positionSet = false;

// Function Prototypes
bool stopMotor();
void connectWifi();
bool setBlindPosition(int eepromLocation, int32_t position);
int32_t getBlindPosition(int32_t* blindPosition, int eepromLocation);
bool setBlindState(uint8_t state, bool unset);
uint8_t getBlindState(uint8_t state);

void setup()
{
  // Button setup
  btnUP.begin(BUTTON_UP);
  btnDWN.begin(BUTTON_DOWN);

  // Serial setup
  Serial.begin(9600);
  delay(200);
  Serial.println("Serial started");

  // Analog setup
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_0db);
  // Setup ADC Continuous with following input:
  // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 50000, &adcComplete);

  // Wifi setup
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

  // Stepper position setup
  if (readPosition() == 0 || (digitalRead(BUTTON_DOWN) && digitalRead(BUTTON_DOWN)))
  {
    stepper->runBackward();
    
    while (true)
    {
      if(stopMotor())
      {
        stepper->stopMove();
        
        writePosition(stepper->getCurrentPosition())
      }
      delay(50);
    }
  }
}

// main programm loop
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
      stepper->setPositionAfterCommandsCompleted()
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

// stop motor when physical limit is reached, return true if motor is stopped
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

      float voltageDelta = meassurements[writeIndex] - meassurements[prevIndex];
      Serial.print(voltageDelta);
      Serial.print(", ");
      Serial.println(meassurements[writeIndex]);

      if (voltageDelta > 10)
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

// connect to wifi using stored credentials
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

// write blind position to eeprom
bool setBlindPosition(int eepromLocation, int32_t position)
{
  // eepromLocation is the eeprom location of the position to be written
  // position is the position of the stepper that should be set
  if (eepromLocation < 2044) {
    EEPROM.write(eepromLocation, (position >> 24) & 0xFF);
    EEPROM.write(eepromLocation + 1, (position >>16) & 0xFF);
    EEPROM.write(eepromLocation + 2, (position >> 8) & 0xFF);
    EEPROM.write(eepromLocation + 3, position & 0xFF);
    return true;
  } else {
    return false;
  }
}

// read blind position from eeprom
bool getBlindPosition(int32_t* blindPosition, int eepromLocation)
{
  // eepromLocation is the eeprom location of the blind position to be written
  if (blindPosition && eepromLocation < 2044) {
    blindPosition = 
      ((int32_t)EEPROM.read(eepromLocation) << 24) +
      ((int32_t)EEPROM.read(eepromLocation + 1) << 16) +
      ((int32_t)EEPROM.read(eepromLocation + 2) << 8) +
      (int32_t)EEPROM.read(eepromLocation + 3);
    return true;
  } else {
    return false;
  };
}

// write blind state to eeprom
bool setBlindState(uint8_t state, bool unset)
{
  if (state < 32) {
    uint8_t currentState = EEPROM.read(EEPROM_BLIND_STATE + (state/8));
    if (unset) {
      EEPROM.write(EEPROM_BLIND_STATE + (state/8), currentState & ~(1<<(state % 8)));
    } else {
      EEPROM.write(EEPROM_BLIND_STATE + (state/8), currentState | (1<<(state % 8)));
    }
    return true;
  } else {
    return false;
  }
}

// read blind state to eeprom
uint8_t getBlindState(uint8_t state)
{
  uint8_t currentState = EEPROM.read(EEPROM_BLIND_STATE + (state/8));
  return (currentState & (1<<(state % 8))) != 0;
}