/**
    Blackmagic Design 3G-SDI Camera Remote control

    Analog volume
      - A0 - Iris
      - A1 - Focus (mode 0) / Color temp (mode 1)
    GPIO
      - 2 - LED
      - 3 - Button push switch
      - 4 - Rotary encoder:A
      - 5 - Rotary encoder:B

    Setup Steps:
      1) Connect the Blackmagic Design 3G-SDI Shield to an Arduino board.
      2) Connect the MIDI/Analog volume shield to the Arduino board.
      3) Attach a camera's SDI input connector to the output SDI connector of
         the shield. Configure the camera as camera number 1.
      4) Build and run the sketch.

     Copyright 2020 by Takumi Aamno / studio bluegreen / Takumi DRIVE OU
**/

#define VERSION "ver.0.2 "

#include <SimpleRotary.h>
#include <Wire.h>
#include <I2CLiquidCrystal.h>
#include <BMDSDIControl.h>

// if the rotary switch is gnd common, uncomment this
#define GNDCOMMON

// Hardware pin mappings
const int variableRegPin0 = A0;
const int variableRegPin1 = A1;

// Blackmagic Design SDI control shield globals
#define SHEILDADDRESS 0x6E
BMD_SDITallyControl_I2C   sdiTallyControl(SHEILDADDRESS);
BMD_SDICameraControl_I2C  sdiCameraControl(SHEILDADDRESS);

// Tnitialize other libraries
I2CLiquidCrystal lcd(31, (bool)false);
SimpleRotary rotary(4, 5, 1); // A, B

// AnalogReg sample rate limiter globals
unsigned long lastAnalogRegUpdateTime;

// Analog values
int focusValue = 0;
int apatureValue = 0;

int cameraNumber = 1; // Camera number (should be configurable later
bool valueMode = false;

const int gain_value[] =      { 1, 2, 4, 8, 16 };
const String gain_string[] =  { "-12dB", " -6dB", "  0dB", "  6dB", " 12dB" };
#define GAIN_DEFAULT_IDX      2 // default 4
#define GAIN_COUNT            5

const int shutter_value[] =      { 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
const String shutter_string[] =  { "  24", "  30", "  50", "  60", "  75", "  90", " 100", " 120", " 150", " 180", " 250", " 360", " 500", " 725", "1000", "1450", "2000"};
#define SHUTTER_DEFAULT_IDX      1  // default 30?
#define SHUTTER_COUNT            17

const int color_value[] =  {2500, 2800, 3000, 3200, 3400, 3600, 4000, 4500, 4800, 5000, 5200, 5400, 5600, 6000, 6500, 7000, 7500, 8000 };
#define COLOR_DEFAULT_IDX  12 // default 5600
#define COLOR_COUNT        18

// Current target - Blackmagic Micro Studio Camera doesn't recognize TINT, so now disabled
// It should be turned on/off by CAMERA setup
// const int tint_value[] =    { -40, -35, -30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40};
// #define TINT_DEFAULT_IDX    8 // default 0
// #define TINT_COUNT          16

// Parameters definitions as struct
struct PARAM {
  String title;
  int *values;
  String *value_strings;
  int index;
  int count;
};

#define PARAM_COUNT 3
struct PARAM Parameters[PARAM_COUNT] = {
   {"Gain   ", gain_value,    gain_string,    GAIN_DEFAULT_IDX,    GAIN_COUNT   }
  ,{"Shutter", shutter_value, shutter_string, SHUTTER_DEFAULT_IDX, SHUTTER_COUNT}
  ,{"ColTemp", color_value,   NULL,           COLOR_DEFAULT_IDX,   COLOR_COUNT  }
//,{"Tint   ", tint_value,    NULL,           TINT_DEFAULT_IDX,    TINT_COUNT   }
};
int param_idx = 0;

/**
 * Functions
 */
int getValueAtIndex(int param_idx)
{
  int value_index = Parameters[param_idx].index;
  return Parameters[param_idx].values[value_index];
}

String getValueStringAtIndex(int param_idx)
{
  int value_index = Parameters[param_idx].index;
  if (Parameters[param_idx].value_strings == NULL)
  {
    // no string defined, so return converted value
    return String(Parameters[param_idx].values[value_index]);
  }
  else
  {
    return Parameters[param_idx].value_strings[value_index];
  }
}

String getParamTitle(int param_idx)
{
  return Parameters[param_idx].title;
}

void incValueIndex(int param_idx)
{
  int count = Parameters[param_idx].count;
  if (Parameters[param_idx].index < (count - 1))
  {
    Parameters[param_idx].index++;
    sendParam(param_idx);
  }
}

void decValueIndex(int param_idx)
{
  if (Parameters[param_idx].index > 0)
  {
    Parameters[param_idx].index--;
    sendParam(param_idx);
  }
}

bool getAnalogRegUpdateReady()
{
  /**
  unsigned long currentTime = millis();
  if ((lastAnalogRegUpdateTime - currentTime) > 100) {
    lastAnalogRegUpdateTime = currentTime;
    return true;
  }

  return false;
  **/
  return true;
}

int getAnalogRegPercent(int analogPin)
{
  // Reads the analog value on the given analog pin as a [0 - 100] scaled value

  int rawAnalogValue    = analogRead(analogPin);
  int scaledAnalogValue = map(rawAnalogValue, 0, 1023, 0, 100);

  return scaledAnalogValue;
}

void showParamTitle(int idx)
{
  lcd.setCursor(0, 0);
  lcd.print(getParamTitle(idx));
  Serial.println(getParamTitle(idx));
}

void showParamValue(int idx)
{
  String valueString = getValueStringAtIndex(idx);
  lcd.setCursor(0, 1);
  lcd.print("        ");
  lcd.setCursor(0, 1);
  lcd.print(valueString);
  Serial.println(valueString);
}

void sendCameraGain(int8_t gainvalue)
{
    Serial.print("Gain: ");
    Serial.println(gainvalue);
    sdiCameraControl.writeCommandInt8(
      cameraNumber,         // Destination:    Camera 1
      1,                    // Group:          Video
      1,                    // Param:          Camera Gain(ISO)
      0,                    // Operation:      Set Absolute,
      gainvalue             // Values
    );
}

void sendCameraShutter(int16_t shuttervalue)
{
    Serial.print("Shutter:");
    Serial.println(shuttervalue);

    sdiCameraControl.writeCommandInt16(
      cameraNumber,         // Destination:    Camera 1
      1,                    // Group:          Video
      6,                    // Param:          Shutter speed
      0,                    // Operation:      Set Absolute,
      shuttervalue          // Values
    );
}

void sendCameraWB(int16_t colortemp, int16_t colortint)
{
    int16_t tempArray[] = {colortemp, colortint};

    Serial.print("Color temp: ");
    Serial.print(tempArray[0]);
    Serial.print(" tint: ");
    Serial.println(tempArray[1]);

    sdiCameraControl.writeCommandInt16(
      cameraNumber,         // Destination:    Camera 1
      1,                    // Group:          Video
      2,                    // Param:          White balance
      0,                    // Operation:      Set Absolute,
      tempArray             // Values
    );
}

void sendParam(int paramIdx)
{
  int16_t c_temp;
  int16_t c_tint;
  switch(paramIdx)
  {
    case 0: // Gain dB
      sendCameraGain((int8_t)getValueAtIndex(paramIdx));
      break;

    case 1: // Shutter
      sendCameraShutter((int16_t)getValueAtIndex(paramIdx));
      break;

    case 2: // ColTemp
      c_temp = (int16_t)getValueAtIndex(paramIdx);
      c_tint = 0; // (int16_t)getValueAtIndex(paramIdx + 1);
      sendCameraWB(c_temp, c_tint);
      break;

    case 3: // Tint
      // c_temp = (int16_t)getValueAtIndex(paramIdx - 1);
      // c_tint = (int16_t)getValueAtIndex(paramIdx);
      // sendCameraWB(c_temp, c_tint);
      break;

    default:
      break;
  }
}

/**
 * Setup and initialize
 */
void setup()
{
  Serial.begin(9600);
  Serial.println("Blackmagic Design SDI Control Shield");

  // set parameter value
  // TBD: read from EEPROM
  // Serial.println("Param initialized");

  // Configure digital inputs
  pinMode(2, OUTPUT);    // LED
#ifdef GNDCOMMON
  digitalWrite(2, LOW);
#else
  digitalWrite(2, HIGH);
#endif
  pinMode(3, INPUT);     // set button as input (pull-downed)
  Serial.println("Pinmode initialized");

  // The shield supports up to 400KHz, use faster
  // I2C speed to reduce latency
  // Wire.setClock(400000);

  // initialize I2C LCD
  Serial.println("LCD initializing");
  lcd.begin(8, 2);
  lcd.display();
  Serial.println("LCD initialized");

  // Set up the BMD SDI control library
  sdiTallyControl.begin();
  sdiCameraControl.begin();

  // Enable both tally and control overrides
  sdiTallyControl.setOverride(true);
  sdiCameraControl.setOverride(true);

  // Print version message to the LCD.
  //         12345678
  lcd.setCursor(0, 0);
  lcd.print("Cam RMT");
  lcd.setCursor(0, 1);
  lcd.print(VERSION);

  delay(2000);
  Serial.println("Ready.");

  showParamTitle(0);
  showParamValue(0);
}

/**
 * Main loop
 */
void loop()
{
  // On update analog value
  if (getAnalogRegUpdateReady())
  {
    // get Focus value
    int currentAnalogReg1 = getAnalogRegPercent(variableRegPin1);
    if (currentAnalogReg1 != focusValue)
    {
      focusValue = currentAnalogReg1;
      Serial.print("Focus value: ");
      Serial.println(focusValue);
      float lensValue = (float)focusValue / 100.0;
      sdiCameraControl.writeCommandFixed16(
        cameraNumber,         // Destination:    Camera 1
        0,                    // Group:          Lens
        0,                    // Param:          Focus [0.0 - 1.0]
        0,                    // Operation:      Set Absolute,
        lensValue             // Values
      );
      delay(50);
    }

    int currentAnalogReg0 = getAnalogRegPercent(variableRegPin0);
    if (currentAnalogReg0 != apatureValue)
    {
      apatureValue = currentAnalogReg0;
      Serial.print("Aperture value: ");
      Serial.println(apatureValue);
      float lensValue = (float)apatureValue / 100.0;
      sdiCameraControl.writeCommandFixed16(
        cameraNumber,         // Destination:    Camera 1
        0,                    // Group:          Lens
        3,                    // Param:          Apature [0.0 - 1.0]
        0,                    // Operation:      Set Absolute,
        lensValue             // Values
      );
      delay(50);
    }
  }

  // On update rotary encoder
  byte enc_changed = rotary.rotate();
  if (enc_changed != 0)
  {
    // CW
    if ( enc_changed == 1 )
    {
      if (!valueMode)
      {
        if (param_idx < PARAM_COUNT - 1)
        {
          // change parameter index
          param_idx++;
        }
      }
      else
      {
        incValueIndex(param_idx);
      }
    }
    else if ( enc_changed == 2 )
    // CCW
    {
      if (!valueMode)
      {
        if (param_idx > 0)
        {
          // change parameter index
          param_idx--;
        }
      }
      else
      {
        decValueIndex(param_idx);
      }
    }

    // update display
    showParamTitle(param_idx);
    showParamValue(param_idx);
  }

  // On update button status
#ifdef GNDCOMMON
  if (digitalRead(3) == LOW)
#else
  if (digitalRead(3) == HIGH)
#endif
  {
    if (valueMode)
    {
      valueMode = false;
#ifdef GNDCOMMON
      digitalWrite(2, LOW); // Turn off LED
#else
      digitalWrite(2, HIGH); // Turn off LED
#endif
    }
    else
    {
      valueMode = true;
#ifdef GNDCOMMON
      digitalWrite(2, HIGH); // Turn off LED
#else
      digitalWrite(2, LOW); // Turn off LED
#endif
    }
    delay(400);
  }
}
