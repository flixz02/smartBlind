#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <EEPROM.h>
#include <ESPUI.h>
#include "FS.h"
#include <LittleFS.h>
#include <Ewma.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include "FastAccelStepper.h"

// Settings
#define SLOW_BOOT 0
#define HOSTNAME "smartBlind1"
#define FORCE_USE_HOTSPOT 0

// Delays
#define BUTTON_DELAY 500
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
uint8_t step = 0;
#define STATE_NOTHING 0
#define STATE_UP 1
#define STATE_DOWN 2

// Pins
#define DIR_PIN 10
#define ENABLE_PIN 21
#define STEP_PIN 20
#define BUTTON_UP 3
#define BUTTON_DOWN 4

// Moving average filter
Ewma adcFilter(0.1);

// Stepper
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
uint32_t buttonTime = 0;
uint8_t endstopVoltage = 100;

// Function Prototypes
bool stopMotor();
void connectWifi();
void setUpUI();
void enterWifiDetailsCallback(Control *sender, int type);
void textCallback(Control *sender, int type);
void blindUpCallback(Control *sender, int type);
void blindStopCallback(Control *sender, int type);
void blindDownCallback(Control *sender, int type);
void endstopVoltageCallback(Control *sender, int type);

// UI handles
uint16_t wifi_ssid_text, wifi_pass_text;
uint16_t mainSlider;

// This is the main function which builds our GUI
void setUpUI()
{
  // Turn off verbose debugging
  ESPUI.setVerbosity(Verbosity::Quiet);

  // Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = true;

  // Tab: Basic Controls
  // This tab contains all the basic ESPUI controls, and shows how to read and update them at runtime.
  auto maintab = ESPUI.addControl(Tab, "", "Basic controls");

  auto groupbutton = ESPUI.addControl(Button, "UP", "UP", Dark, maintab, blindUpCallback);
  ESPUI.addControl(Button, "STOP", "STOP", Alizarin, groupbutton, blindStopCallback);
  ESPUI.addControl(Button, "DOWN", "DOWN", Alizarin, groupbutton, blindDownCallback);

  ESPUI.addControl(Graph, "GRAPH1");

  // TAB: Parameters
  auto paramtab = ESPUI.addControl(Tab, "", "Parameters");

  mainSlider = ESPUI.addControl(Slider, "Slider", "200", Turquoise, paramtab, endstopVoltageCallback);
  ESPUI.addControl(Min, "", "10", None, mainSlider);
  ESPUI.addControl(Max, "", "400", None, mainSlider);

  /*
   * Tab: WiFi Credentials
   * You use this tab to enter the SSID and password of a wifi network to autoconnect to.
   *-----------------------------------------------------------------------------------------------------------*/
  auto wifitab = ESPUI.addControl(Tab, "", "WiFi Credentials");
  wifi_ssid_text = ESPUI.addControl(Text, "SSID", "", Alizarin, wifitab, textCallback);
  // Note that adding a "Max" control to a text control sets the max length
  ESPUI.addControl(Max, "", "32", None, wifi_ssid_text);
  wifi_pass_text = ESPUI.addControl(Text, "Password", "", Alizarin, wifitab, textCallback);
  ESPUI.addControl(Max, "", "64", None, wifi_pass_text);
  ESPUI.addControl(Button, "Save", "Save", Peterriver, wifitab, enterWifiDetailsCallback);

  // Finally, start up the UI.
  // This should only be called once we are connected to WiFi.
  ESPUI.begin(HOSTNAME);
}

void blindDownCallback(Control *sender, int type)
{
  if (type == B_UP)
  {
    stepper->runForward();
    step = STATE_DOWN;
    buttonTime = millis();
  }
}

void blindStopCallback(Control *sender, int type)
{
  if (type == B_UP)
  {
    stepper->stopMove();
    step = STATE_NOTHING;
    buttonTime = millis();
  }
}

void blindUpCallback(Control *sender, int type)
{
  if (type == B_UP)
  {
    stepper->runBackward();
    step = STATE_UP;
    buttonTime = millis();
  }
}

void endstopVoltageCallback(Control *sender, int type)
{
  endstopVoltage = (uint8_t)sender->value.toInt();
}

void setup()
{
  // Pin settings
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  Serial.begin(115200);
  delay(200);

  // Analog setup
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_0db);
  // Setup ADC Continuous with following input:
  // array of pins, count of the pins, how many conversions per pin in one cycle will happen, sampling frequency, callback function
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, 50000, &adcComplete);

  randomSeed(0);
  connectWifi();
#if defined(ESP32)
  WiFi.setSleep(false); // For the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
#endif
  setUpUI();

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
  }
}

void loop()
{
  static uint8_t prevStep = STATE_NOTHING;
  Serial.println(step);
  
  switch (step)
  {
  case STATE_NOTHING:
    if (digitalRead(BUTTON_UP) == LOW)
    {
      buttonTime = millis();
      stepper->runBackward();
      analogContinuousStart();
      delay(BUTTON_DELAY);
      step = STATE_UP;
    }
    else if (digitalRead(BUTTON_DOWN) == LOW)
    {
      buttonTime = millis();
      stepper->runForward();
      analogContinuousStart();
      delay(BUTTON_DELAY);
      step = STATE_DOWN;
    }
    break;

  case STATE_UP:
    if (prevStep = STATE_NOTHING) {
      delay(BUTTON_DELAY);
    }

    if (digitalRead(BUTTON_DOWN) == LOW)
    {
      stepper->stopMove();
      buttonTime = millis();
      step = STATE_NOTHING;
    }

    if (stopMotor())
    {
      stepper->stopMove();
      analogContinuousStop();
      step = STATE_NOTHING;
    }
    break;

  case STATE_DOWN:
    if (prevStep = STATE_NOTHING) {
      delay(BUTTON_DELAY);
    }

    if (digitalRead(BUTTON_UP) == LOW)
    {
      stepper->stopMove();
      buttonTime = millis();
      step = STATE_NOTHING;
    }

    if (stopMotor())
    {
      stepper->stopMove();
      analogContinuousStop();
      step = STATE_NOTHING;
    }
    break;
  }

  prevStep = step;
}

// Fuction to stop motor when physical limit is reached, return true if motor is stopped
bool stopMotor()
{
  bool ret = false;

  if (adc_coversion_done)
  {
    // Set ISR flag back to false
    adc_coversion_done = false;
    // Read data from ADC
    if (analogContinuousRead(&result, 0))
    {
      // Optional: Stop ADC Continuous conversions to have more time to process (print) the data
      // analogContinuousStop();

      static float meassurements[10];
      static uint8_t writeIndex;

      meassurements[writeIndex] = adcFilter.filter(result[0].avg_read_raw);

      // prev index entry lut
      const uint8_t lut[10] = {7, 8, 9, 0, 1, 2, 3, 4, 5, 6};

      if((meassurements[writeIndex] - meassurements[lut[writeIndex]]) / 3 > 15){
        if(meassurements[writeIndex] > 230) {
          ret = true;
        }
      }

      //Serial.printf("\n %.1f", meassurements[writeIndex]);

      if (writeIndex > 9) {
        writeIndex = 0;
      }
      else {
        writeIndex++;
      }

      // Optional: If ADC was stopped, start ADC conversions and wait for callback function to set adc_coversion_done flag to true
      // analogContinuousStart();
    }
    else
    {
      Serial.println("Error occurred during reading data. Set Core Debug Level to error or lower for more information.");
    }
  }
  return ret;
}

void readStringFromEEPROM(String &buf, int baseaddress, int size)
{
  buf.reserve(size);
  for (int i = baseaddress; i < baseaddress + size; i++)
  {
    char c = EEPROM.read(i);
    buf += c;
    if (!c)
      break;
  }
}

void connectWifi()
{
  int connect_timeout;
  WiFi.setHostname(HOSTNAME);
  Serial.println("Begin wifi...");

  // Load credentials from EEPROM
  if (!(FORCE_USE_HOTSPOT))
  {
    yield();
    EEPROM.begin(100);
    String stored_ssid, stored_pass;
    readStringFromEEPROM(stored_ssid, 0, 32);
    readStringFromEEPROM(stored_pass, 32, 96);
    EEPROM.end();

    // Try to connect with stored credentials, fire up an access point if they don't work.
    WiFi.begin(stored_ssid.c_str(), stored_pass.c_str());
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    connect_timeout = 28; // 7 seconds
    while (WiFi.status() != WL_CONNECTED && connect_timeout > 0)
    {
      delay(40);
      Serial.print(".");
      connect_timeout--;
    }
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(WiFi.localIP());
    Serial.println("Wifi started");

    if (!MDNS.begin(HOSTNAME))
    {
      Serial.println("Error setting up MDNS responder!");
    }
  }
  else
  {
    Serial.println("\nCreating access point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
    WiFi.softAP(HOSTNAME);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);

    connect_timeout = 120;
    do
    {
      delay(250);
      Serial.print(",");
      connect_timeout--;
    } while (connect_timeout);
  }
}

void enterWifiDetailsCallback(Control *sender, int type)
{
  if (type == B_UP)
  {
    Serial.println("Saving credentials to EPROM...");
    Serial.println(ESPUI.getControl(wifi_ssid_text)->value);
    Serial.println(ESPUI.getControl(wifi_pass_text)->value);
    unsigned int i;
    EEPROM.begin(100);
    for (i = 0; i < ESPUI.getControl(wifi_ssid_text)->value.length(); i++)
    {
      EEPROM.write(i, ESPUI.getControl(wifi_ssid_text)->value.charAt(i));
      if (i == 30)
        break; // Even though we provided a max length, user input should never be trusted
    }
    EEPROM.write(i, '\0');

    for (i = 0; i < ESPUI.getControl(wifi_pass_text)->value.length(); i++)
    {
      EEPROM.write(i + 32, ESPUI.getControl(wifi_pass_text)->value.charAt(i));
      if (i == 94)
        break; // Even though we provided a max length, user input should never be trusted
    }
    EEPROM.write(i + 32, '\0');
    EEPROM.end();
  }
}

void textCallback(Control *sender, int type)
{
  // This callback is needed to handle the changed values, even though it doesn't do anything itself.
}

void randomString(char *buf, int len)
{
  for (auto i = 0; i < len - 1; i++)
    buf[i] = random(0, 26) + 'A';
  buf[len - 1] = '\0';
}