#include <Arduino.h>
#include <EEPROM.h>
#include <ESPUI.h>
#include "FS.h"
#include <LittleFS.h>

#include <WiFi.h>
#include <ESPmDNS.h>

// Settings
#define SLOW_BOOT 0
#define HOSTNAME "ESPUITest"
#define FORCE_USE_HOTSPOT 0

// Delays
#define BUTTON_DELAY 500
#define MEASSURE_DELAY 400

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


#include "FastAccelStepper.h"

// Pins
#define DIR_PIN 10
#define ENABLE_PIN 21
#define STEP_PIN 20
#define BUTTON_UP 3
#define BUTTON_DOWN 4

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// Function Prototypes
bool stopMotor();
void connectWifi();
void setUpUI();
void enterWifiDetailsCallback(Control *sender, int type);
void textCallback(Control *sender, int type);
void generalCallback(Control *sender, int type);
void scrambleCallback(Control *sender, int type);
void styleCallback(Control *sender, int type);
void updateCallback(Control *sender, int type);
void getTimeCallback(Control *sender, int type);
void blindUpCallback(Control *sender, int type);
void blindStopCallback(Control *sender, int type);
void blindDownCallback(Control *sender, int type);
void graphAddCallback(Control *sender, int type);
void graphClearCallback(Control *sender, int type);
void randomString(char *buf, int len);
void extendedCallback(Control *sender, int type, void *param);
void endstopVoltageCallback(Control *sender, int type);

// UI handles
uint16_t wifi_ssid_text, wifi_pass_text;
uint16_t mainLabel, mainSwitcher, mainSlider, mainText, mainNumber, mainScrambleButton, mainTime;
uint16_t styleButton, styleLabel, styleSwitcher, styleSlider, styleButton2, styleLabel2, styleSlider2;

uint32_t buttonTime = 0;
uint8_t endstopVoltage = 100;

// This is the main function which builds our GUI
void setUpUI()
{
  // Turn off verbose debugging
  ESPUI.setVerbosity(Verbosity::Quiet);

  // Make sliders continually report their position as they are being dragged.
  ESPUI.sliderContinuous = true;

  // This GUI is going to be a tabbed GUI, so we are adding most controls using ESPUI.addControl
  // which allows us to set a parent control. If we didn't need tabs we could use the simpler add
  // functions like:
  //     ESPUI.button()
  //     ESPUI.label()

  // Tab: Basic Controls
  // This tab contains all the basic ESPUI controls, and shows how to read and update them at runtime.
  auto maintab = ESPUI.addControl(Tab, "", "Basic controls");

  auto groupbutton = ESPUI.addControl(Button, "UP", "UP", Dark, maintab, blindUpCallback);
  ESPUI.addControl(Button, "STOP", "STOP", Alizarin, groupbutton, blindStopCallback);
  ESPUI.addControl(Button, "DOWN", "DOWN", Alizarin, groupbutton, blindDownCallback);

  // TAB: Parameters
  auto paramtab = ESPUI.addControl(Tab, "", "Parameters");

  mainSlider = ESPUI.addControl(Slider, "Slider", "200", Turquoise, paramtab, endstopVoltageCallback);
  ESPUI.addControl(Min, "", "10", None, mainSlider);
  ESPUI.addControl(Max, "", "400", None, mainSlider);

  // /*
  //  * Tab: Colours
  //  * This tab shows all the basic colours
  //  *-----------------------------------------------------------------------------------------------------------*/
  // auto colourtab = ESPUI.addControl(Tab, "", "Colours");
  // ESPUI.addControl(Button, "Alizarin", "Alizarin", Alizarin, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Turquoise", "Turquoise", Turquoise, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Emerald", "Emerald", Emerald, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Peterriver", "Peterriver", Peterriver, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Wetasphalt", "Wetasphalt", Wetasphalt, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Sunflower", "Sunflower", Sunflower, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Carrot", "Carrot", Carrot, colourtab, generalCallback);
  // ESPUI.addControl(Button, "Dark", "Dark", Dark, colourtab, generalCallback);

  // /*
  //  * Tab: Styled controls
  //  * This tab shows off how inline CSS styles can be applied to elements and panels in order
  //  * to customise the look of the UI.
  //  *-----------------------------------------------------------------------------------------------------------*/
  // auto styletab = ESPUI.addControl(Tab, "", "Styled controls");
  // styleButton = ESPUI.addControl(Button, "Styled Button", "Button", Alizarin, styletab, generalCallback);
  // styleLabel = ESPUI.addControl(Label, "Styled Label", "This is a label", Alizarin, styletab, generalCallback);
  // styleSwitcher = ESPUI.addControl(Switcher, "Styled Switcher", "1", Alizarin, styletab, generalCallback);
  // styleSlider = ESPUI.addControl(Slider, "Styled Slider", "0", Alizarin, styletab, generalCallback);

  // // This button will randomise the colours of the above controls to show updating of inline styles
  // ESPUI.addControl(Button, "Randomise Colours", "Randomise Colours", Sunflower, styletab, styleCallback);

  // ESPUI.addControl(Separator, "Other styling examples", "", None, styletab);
  // styleButton2 = ESPUI.addControl(Button, "Styled Button", "Button", Alizarin, styletab, generalCallback);
  // ESPUI.setPanelStyle(styleButton2, "background: linear-gradient(90deg, rgba(131,58,180,1) 0%, rgba(253,29,29,1) 50%, rgba(252,176,69,1) 100%); border-bottom: #555;");
  // ESPUI.setElementStyle(styleButton2, "border-radius: 2em; border: 3px solid black; width: 30%; background-color: #8df;");

  // styleSlider2 = ESPUI.addControl(Slider, "Styled Slider", "0", Dark, styletab, generalCallback);
  // ESPUI.setElementStyle(styleSlider2, "background: linear-gradient(to right, red, orange, yellow, green, blue);");

  // styleLabel2 = ESPUI.addControl(Label, "Styled Label", "This is a label", Dark, styletab, generalCallback);
  // ESPUI.setElementStyle(styleLabel2, "text-shadow: 3px 3px #74b1ff, 6px 6px #c64ad7; font-size: 60px; font-variant-caps: small-caps; background-color: unset; color: #c4f0bb; -webkit-text-stroke: 1px black;");

  // /*
  //  * Tab: Grouped controls
  //  * This tab shows how multiple control can be grouped into the same panel through the use of the
  //  * parentControl value. This also shows how to add labels to grouped controls, and how to use vertical controls.
  //  *-----------------------------------------------------------------------------------------------------------*/
  // auto grouptab = ESPUI.addControl(Tab, "", "Grouped controls");

  // // The parent of this button is a tab, so it will create a new panel with one control.
  // auto groupbutton = ESPUI.addControl(Button, "Button Group", "Button A", Dark, grouptab, generalCallback);
  // // However the parent of this button is another control, so therefore no new panel is
  // // created and the button is added to the existing panel.
  // ESPUI.addControl(Button, "", "Button B", Alizarin, groupbutton, generalCallback);
  // ESPUI.addControl(Button, "", "Button C", Alizarin, groupbutton, generalCallback);

  // // Sliders can be grouped as well
  // // To label each slider in the group, we are going add additional labels and give them custom CSS styles
  // // We need this CSS style rule, which will remove the label's background and ensure that it takes up the entire width of the panel
  // String clearLabelStyle = "background-color: unset; width: 100%;";
  // // First we add the main slider to create a panel
  // auto groupsliders = ESPUI.addControl(Slider, "Slider Group", "10", Dark, grouptab, generalCallback);
  // // Then we add a label and set its style to the clearLabelStyle. Here we've just given it the name "A"
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "A", None, groupsliders), clearLabelStyle);
  // // We can now continue to add additional sliders and labels
  // ESPUI.addControl(Slider, "", "20", None, groupsliders, generalCallback);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "B", None, groupsliders), clearLabelStyle);
  // ESPUI.addControl(Slider, "", "30", None, groupsliders, generalCallback);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "C", None, groupsliders), clearLabelStyle);

  // // We can also usefully group switchers.
  // auto groupswitcher = ESPUI.addControl(Switcher, "Switcher Group", "0", Dark, grouptab, generalCallback);
  // ESPUI.addControl(Switcher, "", "1", Sunflower, groupswitcher, generalCallback);
  // ESPUI.addControl(Switcher, "", "0", Sunflower, groupswitcher, generalCallback);
  // ESPUI.addControl(Switcher, "", "1", Sunflower, groupswitcher, generalCallback);
  // // To label these switchers we need to first go onto a "new line" below the line of switchers
  // // To do this we add an empty label set to be clear and full width (with our clearLabelStyle)
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "", None, groupswitcher), clearLabelStyle);
  // // We will now need another label style. This one sets its width to the same as a switcher (and turns off the background)
  // String switcherLabelStyle = "width: 60px; margin-left: .3rem; margin-right: .3rem; background-color: unset;";
  // // We can now just add the styled labels.
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "A", None, groupswitcher), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "B", None, groupswitcher), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "C", None, groupswitcher), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "D", None, groupswitcher), switcherLabelStyle);

  // // You can mix and match different control types, but the results might sometimes
  // // need additional styling to lay out nicely.
  // auto grouplabel = ESPUI.addControl(Label, "Mixed Group", "Main label", Dark, grouptab);
  // auto grouplabel2 = ESPUI.addControl(Label, "", "Secondary label", Emerald, grouplabel);
  // ESPUI.addControl(Button, "", "Button D", Alizarin, grouplabel, generalCallback);
  // ESPUI.addControl(Switcher, "", "1", Sunflower, grouplabel, generalCallback);
  // ESPUI.setElementStyle(grouplabel2, "font-size: x-large; font-family: serif;");

  // // Some controls can even support vertical orientation, currently Switchers and Sliders
  // ESPUI.addControl(Separator, "Vertical controls", "", None, grouptab);
  // auto vertgroupswitcher = ESPUI.addControl(Switcher, "Vertical Switcher Group", "0", Dark, grouptab, generalCallback);
  // ESPUI.setVertical(vertgroupswitcher);
  // // On the following lines we wrap the value returned from addControl and send it straight to setVertical
  // ESPUI.setVertical(ESPUI.addControl(Switcher, "", "0", None, vertgroupswitcher, generalCallback));
  // ESPUI.setVertical(ESPUI.addControl(Switcher, "", "0", None, vertgroupswitcher, generalCallback));
  // ESPUI.setVertical(ESPUI.addControl(Switcher, "", "0", None, vertgroupswitcher, generalCallback));
  // // The mechanism for labelling vertical switchers is the same as we used above for horizontal ones
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "", None, vertgroupswitcher), clearLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "A", None, vertgroupswitcher), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "B", None, vertgroupswitcher), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "C", None, vertgroupswitcher), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "D", None, vertgroupswitcher), switcherLabelStyle);

  // auto vertgroupslider = ESPUI.addControl(Slider, "Vertical Slider Group", "15", Dark, grouptab, generalCallback);
  // ESPUI.setVertical(vertgroupslider);
  // ESPUI.setVertical(ESPUI.addControl(Slider, "", "25", None, vertgroupslider, generalCallback));
  // ESPUI.setVertical(ESPUI.addControl(Slider, "", "35", None, vertgroupslider, generalCallback));
  // ESPUI.setVertical(ESPUI.addControl(Slider, "", "45", None, vertgroupslider, generalCallback));
  // // The mechanism for labelling vertical sliders is the same as we used above for switchers
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "", None, vertgroupslider), clearLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "A", None, vertgroupslider), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "B", None, vertgroupslider), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "C", None, vertgroupslider), switcherLabelStyle);
  // ESPUI.setElementStyle(ESPUI.addControl(Label, "", "D", None, vertgroupslider), switcherLabelStyle);

  // // Note that combining vertical and horizontal sliders is going to result in very messy layout!

  // /*
  //  * Tab: Example UI
  //  * An example UI for the documentation
  //  *-----------------------------------------------------------------------------------------------------------*/
  // auto exampletab = ESPUI.addControl(Tab, "Example", "Example");
  // ESPUI.addControl(Separator, "Control and Status", "", None, exampletab);
  // ESPUI.addControl(Switcher, "Power", "1", Alizarin, exampletab, generalCallback);
  // ESPUI.addControl(Label, "Status", "System status: OK", Wetasphalt, exampletab, generalCallback);

  // ESPUI.addControl(Separator, "Settings", "", None, exampletab);
  // ESPUI.addControl(PadWithCenter, "Attitude Control", "", Dark, exampletab, generalCallback);
  // auto examplegroup1 = ESPUI.addControl(Button, "Activate Features", "Feature A", Carrot, exampletab, generalCallback);
  // ESPUI.addControl(Button, "Activate Features", "Feature B", Carrot, examplegroup1, generalCallback);
  // ESPUI.addControl(Button, "Activate Features", "Feature C", Carrot, examplegroup1, generalCallback);
  // ESPUI.addControl(Slider, "Value control", "45", Peterriver, exampletab, generalCallback);

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

// This callback generates and applies inline styles to a bunch of controls to change their colour.
// The styles created are of the form:
//	"border-bottom: #999 3px solid; background-color: #aabbcc;"
//	"background-color: #aabbcc;"
void styleCallback(Control *sender, int type)
{
  // Declare space for style strings. These have to be static so that they are always available
  // to the websocket layer. If we'd not made them static they'd be allocated on the heap and
  // will be unavailable when we leave this function.
  static char stylecol1[60], stylecol2[30];
  if (type == B_UP)
  {
    // Generate two random HTML hex colour codes, and print them into CSS style rules
    sprintf(stylecol1, "border-bottom: #999 3px solid; background-color: #%06X;", (unsigned int)random(0x0, 0xFFFFFF));
    sprintf(stylecol2, "background-color: #%06X;", (unsigned int)random(0x0, 0xFFFFFF));

    // Apply those styles to various elements to show how controls react to styling
    ESPUI.setPanelStyle(styleButton, stylecol1);
    ESPUI.setElementStyle(styleButton, stylecol2);
    ESPUI.setPanelStyle(styleLabel, stylecol1);
    ESPUI.setElementStyle(styleLabel, stylecol2);
    ESPUI.setPanelStyle(styleSwitcher, stylecol1);
    ESPUI.setElementStyle(styleSwitcher, stylecol2);
    ESPUI.setPanelStyle(styleSlider, stylecol1);
    ESPUI.setElementStyle(styleSlider, stylecol2);
  }
}

// This callback updates the "values" of a bunch of controls
void scrambleCallback(Control *sender, int type)
{
  static char rndString1[10];
  static char rndString2[20];
  static bool scText = false;

  if (type == B_UP)
  { // Button callbacks generate events for both UP and DOWN.
    // Generate some random text
    randomString(rndString1, 10);
    randomString(rndString2, 20);

    // Set the various controls to random value to show how controls can be updated at runtime
    ESPUI.updateLabel(mainLabel, String(rndString1));
    ESPUI.updateSwitcher(mainSwitcher, ESPUI.getControl(mainSwitcher)->value.toInt() ? false : true);
    ESPUI.updateSlider(mainSlider, random(10, 400));
    ESPUI.updateText(mainText, String(rndString2));
    ESPUI.updateNumber(mainNumber, random(100000));
    ESPUI.updateButton(mainScrambleButton, scText ? "Scrambled!" : "Scrambled.");
    scText = !scText;
  }
}

void getTimeCallback(Control *sender, int type)
{
  if (type == B_UP)
  {
    ESPUI.updateTime(mainTime);
  }
}

// Most elements in this test UI are assigned this generic callback which prints some
// basic information. Event types are defined in ESPUI.h
void generalCallback(Control *sender, int type)
{
  Serial.print("CB: id(");
  Serial.print(sender->id);
  Serial.print(") Type(");
  Serial.print(type);
  Serial.print(") '");
  Serial.print(sender->label);
  Serial.print("' = ");
  Serial.println(sender->value);
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

void basicControlCallback(Control *sender, int type)
{
}

// Most elements in this test UI are assigned this generic callback which prints some
// basic information. Event types are defined in ESPUI.h
// The extended param can be used to hold a pointer to additional information
// or for C++ it can be used to return a this pointer for quick access
// using a lambda function
void extendedCallback(Control *sender, int type, void *param)
{
  Serial.print("CB: id(");
  Serial.print(sender->id);
  Serial.print(") Type(");
  Serial.print(type);
  Serial.print(") '");
  Serial.print(sender->label);
  Serial.print("' = ");
  Serial.println(sender->value);
  Serial.print("param = ");
  Serial.println((long)param);
}

void setup()
{
  randomSeed(0);
  connectWifi();
#if defined(ESP32)
  WiFi.setSleep(false); // For the ESP32: turn off sleeping to increase UI responsivness (at the cost of power use)
#endif
  setUpUI();

  // analog input settings

  // Pin settings
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper)
  {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN, false);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    stepper->setDelayToEnable(50);
    stepper->setDelayToDisable(100);

    stepper->setSpeedInUs(2500); // the parameter is us/step !!!
    stepper->setAcceleration(1000);
  }
}

void loop()
{
  switch (step)
  {
  case STATE_NOTHING:
    if (millis() - buttonTime > BUTTON_DELAY)
    {
      if (digitalRead(BUTTON_UP) == LOW)
      {
        buttonTime = millis();
        stepper->runBackward();
        step = STATE_UP;
      }
      else if (digitalRead(BUTTON_DOWN) == LOW)
      {
        buttonTime = millis();
        stepper->runForward();
        step = STATE_DOWN;
      }
    }
    break;

  case STATE_UP:
    if (millis() - buttonTime > BUTTON_DELAY)
    {
      if (digitalRead(BUTTON_DOWN) == LOW)
      {
        step = STATE_NOTHING;
        stepper->stopMove();
        buttonTime = millis();
      }
    }
    if (millis() - buttonTime > MEASSURE_DELAY)
    {
      if (analogReadMilliVolts(0) > endstopVoltage)
      {
        step = stopMotor();
      }
    }
    break;

  case STATE_DOWN:
    if (millis() - buttonTime > BUTTON_DELAY)
    {
      if (digitalRead(BUTTON_UP) == LOW)
      {
        step = STATE_NOTHING;
        stepper->stopMove();
        buttonTime = millis();
      }
    }
    if (millis() - buttonTime > MEASSURE_DELAY)
    {
      if (analogReadMilliVolts(0) > endstopVoltage)
      {
        if(stopMotor())
        {
          step = STATE_NOTHING;
        }
      }
    }
    break;
  }
}

// Fuction to stop motor when physical limit is reached, return true if motor is stopped
bool stopMotor()
{
  static uint16_t counter = 0;
  static uint32_t lastTime = micros();

  if (micros() - lastTime < 200000)
  {
    counter++;
    lastTime = micros();
    if (counter >= 3)
    {
      stepper->stopMove();
      return true;
    }
  }
  else
  {
    counter = 0;
  }

  return false;
}

// Function for controlling Motorspeed
void motorSpeed(int speed)
{
  
  stepper->setSpeedInUs(speed);
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