/********************************************************
 * PID RelayOutput Example
 * Same as basic example, except that this time, the output
 * is going to a digital pin which (we presume) is controlling
 * a relay.  the pid is designed to Output an analog value,
 * but the relay can only be On/Off.
 *
 *   to connect them together we use "time proportioning
 * control"  it's essentially a really slow version of PWM.
 * first we decide on a window size (5000mS say.) we then
 * set the pid to adjust its output between 0 and that window
 * size.  lastly, we add some logic that translates the PID
 * output into "Relay On Time" with the remainder of the
 * window being "Relay Off Time"
 ********************************************************/

#define DEBUG_OUTPUT    1
#define SD_DATA_LOGGING 1

void(* resetFunc) (void) = 0;

// Rotary Encoder Inputs
#define CLK 51
#define DT  52
#define SW  53

int currentStateCLK = 0;
int lastStateCLK = 0;
String currentDir ="";

int iResetCounter = 0;
#define RESET_AT_COUNT  100

bool bOK = true;

bool bChangeSetpoint = true;

long refresh = 0;
long iRefresh = 10;

#define STATE_START_MSG                   0
#define STATE_WAIT_FOR_START              1
#define STATE_RUNNING_CHECK_FOR_STOP      2
#define STATE_SETTING_MSG                 3
#define STATE_WAIT_FOR_SETTING            4
#define STATE_SELECT_SETTING_MSG          5
#define STATE_WAIT_FOR_SELECT_SETTING     6
#define STATE_SELECT_HEATING_PAD_MSG      7
#define STATE_WAIT_FOR_SELECT_HEATING_PAD 8
#define STATE_UPDATE_SETPOINT_MSG         9
#define STATE_WAIT_FOR_SELECT_SETPOINT    10
#define STATE_GET_OK_OR_CANCEL            11
#define STATE_AUTO_RUN_AT_STARTUP         12
#define STATE_UPDATE_DISABLED_MSG         13
#define STATE_WAIT_FOR_SELECT_DISABLED    14

#define SELECT_SETTING_1      1
#define SELECT_SETTING_2      2
#define SELECT_SETTING_3      3

#define SELECT_HEATING_PAD_1  1
#define SELECT_HEATING_PAD_2  2
#define SELECT_HEATING_PAD_3  3
#define SELECT_CANCEL         4

int iSettingSelect = SELECT_SETTING_1;
char *sSettingSelect[3] = {" Change Setpoint\0",
                           " Disable HeatPad\0",
                           " CANCEL         \0"};

int iSelectHeadingPad = SELECT_HEATING_PAD_1;
char *sHeatingPad[4] = {"#1 Heat Pad     \0",
                        "#2 Heat Pad     \0",
                        "#3 Heat Pad     \0",
                        "CANCEL          \0"};


int iState = STATE_START_MSG;

#include <EEPROM.h>

#define EEPROM_LEGO_SIGNATURE F("HEATPADS");

// EEPROM state variables
#define EEPROM_SIZE 512

#define EEPROM_SIGNATURE       0 // 000-007  Signature 'HEATPADS'
#define EEPROM_SETPOINT_1      8 // 008-010  Setpoint 1 XXX
#define EEPROM_SETPOINT_2     11 // 011-013  Setpoint 2 XXX
#define EEPROM_SETPOINT_3     14 // 014-016  Setpoint 3 XXX
#define EEPROM_DISABLE_1      17 // 017-017  Disable Head Pad 1
#define EEPROM_DISABLE_2      18 // 018-018  Disable Head Pad 1
#define EEPROM_DISABLE_3      19 // 019-019  Disable Head Pad 1
#define EEPROM_RUNNING        20 // 020-020  1 = Running

#include <LiquidCrystal.h> 

LiquidCrystal lcd(27, 26, 25, 24, 23, 22);  

int iCounter = 0;
unsigned long ulStartTime = 0;

#include "RunningAverage.h"

RunningAverage myRA_1(20);
RunningAverage myRA_2(20);
RunningAverage myRA_3(20);

long samples = -1;
int iNmbSamples = 100; //50; //25; //50;

#include <Adafruit_MAX31856.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 max[3] = {
  Adafruit_MAX31856(40, 41, 42, 43),
  Adafruit_MAX31856(39, 41, 42, 43),
  Adafruit_MAX31856(38, 41, 42, 43)
};

#include <PID_v1.h>

#define RELAY_1 33  //21
#define RELAY_2 32  //20
#define RELAY_3 31  //19

int Relay_Pins[3] = {RELAY_1,RELAY_2,RELAY_3};
int iRelay = 0;

double SetpointNew = 0;

int DisabledHeatPads[3] = {0,0,0};
int ErrHeatPads[3] =  {0,0,0};

// Heating Pad #1
double Offsets[3] = {0.2, 0.0, 0.0};
// Heating Pad #2
//double Offsets[3] = {0.8, 0.0, 0.0};
// NO Offsets
//double Offsets[3] = {0.0, 0.0, 0.0};

  //initialize the PID variables we're linked to
double Setpoint[3] = {28.0, 28.0, 28.0};

//Specify the links and initial tuning parameters
//double Kp=2, Ki=5, Kd=1;

double Kp[3] = {4000.0, 4000.0, 4000.0};
double Ki[3] = {25.0, 25.0, 25.0};
double Kd[3] = {1.0, 1.0, 1.0};

int WindowSize[3] = {5000,5000,5000};

//Define Variables we'll be connecting to
double Input[3], Output[3], Average[3];

PID myPID[3] = {
                  PID(&Average[0], &Output[0], &Setpoint[0], Kp[0], Ki[0], Kd[0], DIRECT),
                  PID(&Average[1], &Output[1], &Setpoint[1], Kp[1], Ki[1], Kd[1], DIRECT),
                  PID(&Average[2], &Output[2], &Setpoint[2], Kp[2], Ki[2], Kd[2], DIRECT)
};

unsigned long windowStartTime[3];


#if SD_DATA_LOGGING
// SD Card used for data logging
#include <SD.h>

// set up variables using the SD utility library functions:
Sd2Card card;
SdVolume volume;
SdFile root;

// SD Shield
//
// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// MKRZero SD: SDCARD_SS_PIN
//#define chipSelectSDCard 4
#define chipSelectSDCard 10

File fileSDCard;

// Date and time functions using a DS1307 RTC connected via I2C and Wire lib
#include "RTClib.h"

RTC_PCF8523 rtc;
DateTime now;

bool bSDLogFail = false;

unsigned long timeLastLog = 0;
int currentYear  = 0;;
int currentMonth = 0;
int currentDay   = 0;
int currentHour  = 0;
int currentMin   = 0;
int currentSec   = 0;

// String constants
char ERROR_NO_RTC[]       = "Couldnt find RTC";
char ERROR_NO_SDCARD[]    = "SD Card Missing?";

char szOperation[32] = "Startup";

#endif

void setup()
{
  Serial.begin(115200);
  delay(2000);

  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.clear();

  for (int i=0; i<3; i++)
  {
    pinMode(Relay_Pins[i] , OUTPUT);
    digitalWrite(Relay_Pins[i] , HIGH);
  }
  //**Serial.println("PID Relay Output Test");
  Serial.println("Thermocouple type: ");
  
  for (int i=0; i<3; i++) {
    max[i].begin();
    max[i].setThermocoupleType(MAX31856_TCTYPE_K);

    Serial.print(i+1);
    switch ( max[i].getThermocoupleType() ){
      case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
      case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
      case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
      case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
      case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
      case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
      case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
      case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
      case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
      case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
      default: 
        ErrHeatPads[i] = -1;
        Serial.println("Unknown");
        break;
    }
    max[i].setTempFaultThreshholds(0.0, 200.0);    
  }

  //tell the PID to range between 0 and the full window size
  myPID[0].SetOutputLimits(0, WindowSize[0]);
  myPID[1].SetOutputLimits(0, WindowSize[1]);
  myPID[2].SetOutputLimits(0, WindowSize[2]);

  //turn the PID on
  myPID[0].SetMode(AUTOMATIC);
  myPID[1].SetMode(AUTOMATIC);
  myPID[2].SetMode(AUTOMATIC);

#if 0
  // Wait to receive "GO" from serial input
  Serial.println("Send any string to begin");
  String data;
  while (Serial.available() == 0);
  data = Serial.readString();
  data.toUpperCase();
  Serial.println("Starting Loop");
#endif

	// Set encoder pins as inputs
	pinMode(CLK,INPUT);
	pinMode(DT,INPUT);
	pinMode(SW, INPUT_PULLUP);

	// Read the initial state of CLK
	lastStateCLK = digitalRead(CLK);

#if DEBUG_OUTPUT
  for (int i = 0; i < 32; i++)
  {
    int iByte = EEPROM[i];
    if (i % 16 == 0)
      Serial.println("");
    if (iByte < 16)
      Serial.print(F("0"));
    Serial.print(iByte, HEX);
    Serial.print(F(" "));
  }
  Serial.println("");
#endif

  LoadEEPROM();

#if SD_DATA_LOGGING
  // Initialize the Real Time Clock
  if (! rtc.begin())
  {
    Serial.println(ERROR_NO_RTC);
    LCD_DisplayErrorAndHALT(ERROR_NO_RTC);
  }
  if (! rtc.initialized())
  {
    Serial.println("RTC isnt running");

    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  now = rtc.now();
  currentYear = now.year();
  currentMonth = now.month();
  currentDay  = now.day();
  currentHour = now.hour();
  currentMin  = now.minute();
  currentSec  = now.second();

  Serial.println("*** DATE ***    ");
  Serial.print(currentYear, DEC);
  Serial.print("/");
  SerialPrintTwoDigits(currentMonth);
  Serial.print("/");
  SerialPrintTwoDigits(currentDay);
  Serial.print(" ");
  SerialPrintTwoDigits(currentHour);
  Serial.print(":");
  SerialPrintTwoDigits(currentMin);
  Serial.print(":");
  SerialPrintTwoDigits(currentSec);
  Serial.println("");

  SetupSDCardOperations();

  Serial.println("Setup complete.");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setup complete.");

#endif

  DisplaySetpoints();

}

float readSetpointFromEEPROM(int iOffset)
{
  float fReturn = 0;
  fReturn += float(EEPROM[iOffset+0]-'0')*10.0;
  fReturn += float(EEPROM[iOffset+1]-'0');
  fReturn += float(EEPROM[iOffset+2]-'0')*0.1;
  return (fReturn);
}

void writeSetpointToEEPROM(int iOffset, float fSetpoint)
{
  EEPROM[iOffset+0] = '0' + int(fSetpoint)/10;
  EEPROM[iOffset+1] = '0' + int(fSetpoint)%10;
  EEPROM[iOffset+2] = '0' + int(fSetpoint*10.0)%10;
}

void LoadEEPROM()
{
  if (EEPROM[EEPROM_SIGNATURE + 0] == 'H' &&
      EEPROM[EEPROM_SIGNATURE + 1] == 'E' &&
      EEPROM[EEPROM_SIGNATURE + 2] == 'A' &&
      EEPROM[EEPROM_SIGNATURE + 3] == 'T' &&
      EEPROM[EEPROM_SIGNATURE + 4] == 'P' &&
      EEPROM[EEPROM_SIGNATURE + 5] == 'A' &&
      EEPROM[EEPROM_SIGNATURE + 6] == 'D' &&
      EEPROM[EEPROM_SIGNATURE + 7] == 'S')
  {
    Setpoint[0] = readSetpointFromEEPROM(EEPROM_SETPOINT_1);
    Setpoint[1] = readSetpointFromEEPROM(EEPROM_SETPOINT_2);
    Setpoint[2] = readSetpointFromEEPROM(EEPROM_SETPOINT_3);

    DisabledHeatPads[0] = EEPROM[EEPROM_DISABLE_1] - '0';
    DisabledHeatPads[1] = EEPROM[EEPROM_DISABLE_2] - '0';
    DisabledHeatPads[2] = EEPROM[EEPROM_DISABLE_3] - '0';

    if (EEPROM[EEPROM_RUNNING] == '1')
    {
        iState = STATE_AUTO_RUN_AT_STARTUP;

        DisplaySetpoints();

        windowStartTime[0] = millis();
        windowStartTime[1] = millis();
        windowStartTime[2] = millis();
    }
  }
  else
  {
#if DEBUG_OUTPUT
    Serial.println(F("EEPROM HEATPADS not found"));
#endif
    SetupEEPROM();
  }
#if DEBUG_OUTPUT
  Serial.println(F("EEPROM HEATPADS found"));
#endif  
}

void SetupEEPROM()
{
  EEPROM[EEPROM_SIGNATURE + 0] = 'H';
  EEPROM[EEPROM_SIGNATURE + 1] = 'E';
  EEPROM[EEPROM_SIGNATURE + 2] = 'A';
  EEPROM[EEPROM_SIGNATURE + 3] = 'T';
  EEPROM[EEPROM_SIGNATURE + 4] = 'P';
  EEPROM[EEPROM_SIGNATURE + 5] = 'A';
  EEPROM[EEPROM_SIGNATURE + 6] = 'D';
  EEPROM[EEPROM_SIGNATURE + 7] = 'S';

  writeSetpointToEEPROM(EEPROM_SETPOINT_1, Setpoint[0]);
  writeSetpointToEEPROM(EEPROM_SETPOINT_2, Setpoint[1]);
  writeSetpointToEEPROM(EEPROM_SETPOINT_3, Setpoint[2]);

  EEPROM[EEPROM_DISABLE_1] = DisabledHeatPads[0] + '0';
  EEPROM[EEPROM_DISABLE_2] = DisabledHeatPads[1] + '0';
  EEPROM[EEPROM_DISABLE_3] = DisabledHeatPads[2] + '0';

  EEPROM[EEPROM_RUNNING] = '0';
  
#if DEBUG_OUTPUT
  Serial.println(F("EEPROM HEATPADS initialized"));
#endif

}


void loop()
{
#if SD_DATA_LOGGING
  now = rtc.now();
  currentYear = now.year();
  currentMonth = now.month();
  currentDay  = now.day();
  currentHour = now.hour();
  currentMin  = now.minute();
#endif

  if (iState == STATE_AUTO_RUN_AT_STARTUP)
  {
    iState = STATE_RUNNING_CHECK_FOR_STOP;
  }
  if (iState == STATE_START_MSG)
  {
    ResetRelaysAndDisplay();
    lcd.setCursor(0,0);
    lcd.print("HeatPad PID Ctrl");
    lcd.setCursor(0,1);
    lcd.print("Btn Push START  ");
    iState = STATE_WAIT_FOR_START;
    return;
  }
  if (iState == STATE_SETTING_MSG)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("HeatPad PID Ctrl");
    lcd.setCursor(0,1);
    lcd.print("Btn Push SETTING");
    iState = STATE_WAIT_FOR_SETTING;
    return;
  }
  if (iState == STATE_SELECT_SETTING_MSG)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Select Setting  ");
    lcd.setCursor(0,1);
    iSettingSelect = SELECT_SETTING_1;
    lcd.print(sSettingSelect[iSettingSelect-1]);
    iState = STATE_WAIT_FOR_SELECT_SETTING;
    return;
  }
  if (iState == STATE_WAIT_FOR_SELECT_SETTING)
  {
    int iRetCode = CheckRotate();
    if (iRetCode != 0)
    {
      if (iRetCode == 1)
      {
        if (iSettingSelect == SELECT_SETTING_3)
        {
          iSettingSelect = SELECT_SETTING_1;
        }
        else
        {
          iSettingSelect++;
        }
      }
      else
      {
        if (iSettingSelect == SELECT_SETTING_1)
        {
          iSettingSelect = SELECT_SETTING_3;
        }
        else
        {
          iSettingSelect--;
        }
      }
      lcd.setCursor(0,1);
      lcd.print(sSettingSelect[iSettingSelect-1]);
    }
    if (ButtonPushed())
    {
      if (iSettingSelect == SELECT_SETTING_1)
      {
        iState = STATE_SELECT_HEATING_PAD_MSG;
        bChangeSetpoint = true;
      }
      if (iSettingSelect == SELECT_SETTING_2)
      {
        iState = STATE_SELECT_HEATING_PAD_MSG;
        bChangeSetpoint = false;
      }
      if (iSettingSelect == SELECT_SETTING_3)
      {
        iState = STATE_START_MSG;
      }
    }
    return;
  }
  if (iState == STATE_UPDATE_DISABLED_MSG)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(sHeatingPad[iSelectHeadingPad-1]);
    lcd.setCursor(0,1);

    if (DisabledHeatPads[iSelectHeadingPad-1])
    {
      bOK = false;
      lcd.print(" OFF");
    }
    else
    {
      bOK = true;
      lcd.print(" ON ");
    }
    iState = STATE_WAIT_FOR_SELECT_DISABLED;
    return;
  }
  if (iState == STATE_WAIT_FOR_SELECT_DISABLED)
  {
    int iRetCode = CheckRotate();
    if (iRetCode != 0)
    {
      if (bOK)
      {
        bOK = false;
        lcd.setCursor(1, 3);
        lcd.print("OFF");
      }
      else
      {
        bOK = true;
        lcd.setCursor(1, 3);
        lcd.print("ON ");
      }
      return;
    }
    if (ButtonPushed())
    {
      if (bOK)
      {
        DisabledHeatPads[iSelectHeadingPad-1] = 0;
        EEPROM[EEPROM_DISABLE_1+iSelectHeadingPad-1] = '0';

//        writeSetpointToEEPROM(EEPROM_SETPOINT_1+((iSelectHeadingPad-1)*3), Setpoint[iSelectHeadingPad-1]);
      }
      else
      {
        DisabledHeatPads[iSelectHeadingPad-1] = 1;
        EEPROM[EEPROM_DISABLE_1+iSelectHeadingPad-1] = '1';
      }
      iState = STATE_SELECT_HEATING_PAD_MSG; //STATE_SELECT_SETTING_MSG;
    }
    return;
  }
  if (iState == STATE_SELECT_HEATING_PAD_MSG)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("HeatPad Settings");
    lcd.setCursor(0,1);
    iSelectHeadingPad = SELECT_HEATING_PAD_1;
    lcd.print(sHeatingPad[iSelectHeadingPad-1]);
    iState = STATE_WAIT_FOR_SELECT_HEATING_PAD;
    return;
  }
  if (iState == STATE_UPDATE_SETPOINT_MSG)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(sHeatingPad[iSelectHeadingPad-1]);
    lcd.setCursor(0,1);
    SetpointNew = Setpoint[iSelectHeadingPad-1];
    showValue(1, 3,SetpointNew);
    iState = STATE_WAIT_FOR_SELECT_SETPOINT;
    return;
  }

  if (iState == STATE_WAIT_FOR_SELECT_SETPOINT)
  {
    int iRetCode = CheckRotate();
    if (iRetCode != 0)
    {
      if (iRetCode == 1)
      {
        SetpointNew += 0.1;
      }
      else
      {
        SetpointNew -= 0.1;
      }
      showValue(1, 3,SetpointNew);
      return;
    }
    if (ButtonPushed())
    {
      showValue(0, 3,SetpointNew);
      lcd.print(" [");
      showValue(0,9,Setpoint[iSelectHeadingPad-1]);
      lcd.print("]");
      lcd.setCursor(1, 3);
      lcd.print("OK    ");
      iState = STATE_GET_OK_OR_CANCEL;
    }
    return;
  }

  if (iState == STATE_GET_OK_OR_CANCEL)
  {
    int iRetCode = CheckRotate();
    if (iRetCode != 0)
    {
      if (bOK)
      {
        bOK = false;
        lcd.setCursor(1, 3);
        lcd.print("CANCEL");
      }
      else
      {
        bOK = true;
        lcd.setCursor(1, 3);
        lcd.print("OK");
      }
      return;
    }
    if (ButtonPushed())
    {
      if (bOK)
      {
        Setpoint[iSelectHeadingPad-1] = SetpointNew;
        writeSetpointToEEPROM(EEPROM_SETPOINT_1+((iSelectHeadingPad-1)*3), Setpoint[iSelectHeadingPad-1]);
      }
      iState = STATE_SELECT_HEATING_PAD_MSG;
    }
    return;
  }

  if (iState == STATE_WAIT_FOR_SELECT_HEATING_PAD)
  {
    int iRetCode = CheckRotate();
    if (iRetCode != 0)
    {
      if (iRetCode == 1)
      {
        if (iSelectHeadingPad == SELECT_CANCEL)
          iSelectHeadingPad = SELECT_HEATING_PAD_1;
        else
          iSelectHeadingPad++;
      }
      else
      {
        // Encoder is rotating CW so increment
        if (iSelectHeadingPad == SELECT_HEATING_PAD_1)
          iSelectHeadingPad = SELECT_CANCEL;
        else
          iSelectHeadingPad--;
      }
      lcd.setCursor(0,1);
      lcd.print(sHeatingPad[iSelectHeadingPad-1]);
    }

    if (ButtonPushed())
    {
      if (iSelectHeadingPad == SELECT_CANCEL)
         iState = STATE_START_MSG;
      else
      {
        if (bChangeSetpoint)
          iState = STATE_UPDATE_SETPOINT_MSG;
        else
          iState = STATE_UPDATE_DISABLED_MSG;
      }
    }
    return;
  }

  if (iState == STATE_WAIT_FOR_START ||
      iState == STATE_WAIT_FOR_SETTING)
  {
    int iRetCode = CheckRotate();
    if (iRetCode != 0)
    {
      if (iState == STATE_WAIT_FOR_START)
        iState = STATE_SETTING_MSG;
      else
        iState = STATE_START_MSG;
    }
  }

  if (iState == STATE_WAIT_FOR_START ||
      iState == STATE_WAIT_FOR_SETTING ||
      iState == STATE_RUNNING_CHECK_FOR_STOP)
  {
    if (ButtonPushed())
    {
      ResetRelaysAndDisplay();

      if (iState == STATE_WAIT_FOR_SETTING)
        iState = STATE_SELECT_SETTING_MSG;
      else
      if (iState == STATE_RUNNING_CHECK_FOR_STOP)
      {
        iState = STATE_START_MSG;
        EEPROM[EEPROM_RUNNING] = '0';        
      }
      else
      {
        iState = STATE_RUNNING_CHECK_FOR_STOP;

        EEPROM[EEPROM_RUNNING] = '1';

        DisplaySetpoints();

        windowStartTime[0] = millis();
        windowStartTime[1] = millis();
        windowStartTime[2] = millis();
      }
      return;
    }
  }

  if (iState != STATE_RUNNING_CHECK_FOR_STOP)
  {
    return;
  }

  for (int i=0; i<3; i++)
  {    
    if (ErrHeatPads[i])
      continue;

    if (DisabledHeatPads[i])
      continue;

    float fThermocoupleTemperature = max[i].readThermocoupleTemperature();
    if (fThermocoupleTemperature < 10.0 ||
        fThermocoupleTemperature > 35.0)
    {
      ErrHeatPads[i] = -3;
      continue;
    }

    // Add Offset for the thermocouple
    fThermocoupleTemperature = fThermocoupleTemperature + Offsets[i];

    switch (i)
    {
      case 0:
        myRA_1.addValue(fThermocoupleTemperature);
        Input[i] = myRA_1.getAverage();
        break;
      case 1:
        myRA_2.addValue(fThermocoupleTemperature);
        Input[i] = myRA_2.getAverage();
        break;
      case 2:
        myRA_3.addValue(fThermocoupleTemperature);
        Input[i] = myRA_3.getAverage();
        break;
    }
    Average[i] = Input[i];

    // Check and print any faults
    uint8_t fault = max[i].readFault();
    if (fault)
    {
      ErrHeatPads[i] = -2;
      if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
      if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
      if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
      if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
      if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
      if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
      if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
      if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
    }  

    myPID[i].Compute();

    unsigned long now = millis();  //get now time

    /************************************************
    * turn the output pin on/off based on pid output
    ************************************************/
      if (now - windowStartTime[i] > WindowSize[i])
      { //time to shift the Relay Window
        windowStartTime[i] += WindowSize[i];
      }

    if(Output[i] > now - windowStartTime[i]) 
    {
      digitalWrite(Relay_Pins[i],LOW);   //ON
    }  
    else
    {
      digitalWrite(Relay_Pins[i],HIGH);    //OFF
    }

  }

  refresh++;
  if ((refresh % iRefresh) == 0)
  {
    lcd.begin(16,2);
    lcd.clear();
    DisplaySetpoints();
  }

  for (int i=0; i<3; i++)
  {
    int iOffset = 1+(i*5);
    if (DisabledHeatPads[i] || (ErrHeatPads[i] != 0))
    {
      digitalWrite(Relay_Pins[i],HIGH);    //OFF
      lcd.setCursor(iOffset, 1);
      lcd.print("    ");
    }
    else
      showValue(1, iOffset,Average[i]);
  }

  samples++;
  if ((samples % iNmbSamples) == 0)
  {

#if SD_DATA_LOGGING
  SDLogging(currentYear, currentMonth, currentDay, currentHour, currentMin, currentSec, szOperation, Setpoint[0], Average[0], Setpoint[1], Average[1], Setpoint[2], Average[2]);
  strcpy(szOperation, "TempReadings");
#endif

    for (int i=0; i<3; i++)
    {
      Serial.print("AVG_");
      Serial.print(i+1);
      Serial.print(":");
      if (DisabledHeatPads[i] == 1)
        Serial.print("OFF  ");
      else
      if (ErrHeatPads[i] != 0)
      {
        Serial.print("ERR");
        Serial.print(ErrHeatPads[i]);
      }
      else
        Serial.print(Average[i]);
      Serial.print(",");
    }     
    Serial.print("Upper:"); Serial.print(Setpoint[0]+0.25); Serial.print(",");
    Serial.print("Lower:"); Serial.print(Setpoint[0]-0.25); Serial.print(",");
    Serial.print("Setpoint:"); Serial.print(Setpoint[0]); //Serial.print(",");

    Serial.println("");
  }

  //iResetCounter++;
  if (iResetCounter> RESET_AT_COUNT)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RECALIBRATING");
    lcd.setCursor(0,1);
    lcd.print("CONTROLLER");
    delay(2500);
    resetFunc();
  }

}

void showValue(int iRow, int iCol, float fValue)
{
  lcd.setCursor(iCol, iRow);
  lcd.print(int(fValue)/10);
  lcd.print(int(fValue)%10);
  lcd.print(".");
  lcd.print(int(fValue*10)%10);
}

int CheckRotate()
{
  int iRetCode = 0;

  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1)
  {
    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK)
    {
      iRetCode = -1;
    }
    else
    {
      // Encoder is rotating CW so increment
      iRetCode = 1;
    }
  }
  // Remember last CLK state
  lastStateCLK = currentStateCLK;
  return (iRetCode);
}

bool ButtonPushed()
{
  bool  bRetCode = false;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW)
  {
    bRetCode = true;

    while (digitalRead(SW) == LOW)
      ;
  }
  return (bRetCode);
}

void ResetRelaysAndDisplay()
{
  lcd.clear();

  for (int i=0; i<3; i++)
    digitalWrite(Relay_Pins[i],HIGH);    //OFF
}

void DisplaySetpoints()
{
  lcd.clear();
  for (int i=0; i<3; i++)
  {
    int iOffset = 1+(i*5);
    
    if (ErrHeatPads[i] != 0)
    {
      lcd.setCursor(iOffset, 0);
      lcd.print("ERR ");
    }
    else
    if (DisabledHeatPads[i] == 1)
    {
      lcd.setCursor(iOffset, 0);
      lcd.print("OFF ");
    }
    else
      showValue(0, iOffset,Setpoint[i]);
  }
}

#if SD_DATA_LOGGING

void SerialPrintTwoDigits(int iVal)
{
  if (iVal < 10)
    Serial.print("0");
  Serial.print(iVal, DEC);
}

void SDCardPrintTwoDigits(int iVal)
{
  if (iVal < 10)
    fileSDCard.print("0");
  fileSDCard.print(iVal, DEC);
}

void LCD_DisplayErrorAndHALT(char *errorMessage)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("*** ERROR ***");
  lcd.setCursor(0, 1);
  lcd.print(errorMessage);
  while (1);
}

// Initialize the SD for operations
// If the LOGGINGx.CSV file is not present create the file with the first line of column headings
void SetupSDCardOperations()
{
  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!card.init(SPI_HALF_SPEED, chipSelectSDCard)) {
    Serial.println("initialization failed.");
    Serial.println(ERROR_NO_SDCARD);
    LCD_DisplayErrorAndHALT(ERROR_NO_SDCARD);
  } else {
    Serial.println("Wiring is correct and a card is present.");
  }

  // print the type of card
  Serial.println();
  Serial.print("Card type:         ");
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println("SD1");
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println("SD2");
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1);
  }

  Serial.print("Clusters:          ");
  Serial.println(volume.clusterCount());
  Serial.print("Blocks x Cluster:  ");
  Serial.println(volume.blocksPerCluster());

  Serial.print("Total Blocks:      ");
  Serial.println(volume.blocksPerCluster() * volume.clusterCount());
  Serial.println();

  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print("Volume type is:    FAT");
  Serial.println(volume.fatType(), DEC);

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
  Serial.print("Volume size (Kb):  ");
  Serial.println(volumesize);
  Serial.print("Volume size (Mb):  ");
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print("Volume size (Gb):  ");
  Serial.println((float)volumesize / 1024.0);

  Serial.println("\nFiles found on the card (name, date and size in bytes): ");
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);

  Serial.println("");
  Serial.println("*** STATUS ***  ");
  Serial.println("SD Init Start   ");

  if (!SD.begin(chipSelectSDCard)) {
    Serial.println("*** ERROR ***   ");
    Serial.println("SD Init Failed  ");
    Serial.println("System HALTED!");
    while (1);
  }

  Serial.println("* Test CLOCK.CSV");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("* Test CLOCK.CSV");
  lcd.setCursor(0, 1);

  if (SD.exists("CLOCK.CSV"))
  {
    fileSDCard = SD.open("CLOCK.CSV", FILE_READ);
    if (fileSDCard)
    {
      char *ptr3 = 0;

      if (fileSDCard.available())
      {
        char strClockSetting[256];
        char strClockSettingCopy[256];
        fileSDCard.read(strClockSetting, sizeof(strClockSetting));
        fileSDCard.close();
        strClockSetting[sizeof(strClockSetting) - 1] = '\0';
        char *ptr1 = strchr(&strClockSetting[0], '\n');
        if (ptr1 != 0)
        {
          *ptr1++ = '\0';
        }
        Serial.println("Set Clock:");
        Serial.println(strClockSetting);

        ptr3 = strchr(ptr1, '\n');
        if (ptr3 != 0)
        {
          *ptr3++ = '\0';
          strcpy(strClockSettingCopy, ptr1);
        }
        Serial.println(ptr1);

        int iDateTime[6] = {0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 6; i++)
        {
          char *ptr2 = strchr(ptr1, ',');
          if (ptr2 != 0)
          {
            *ptr2 = '\0';
            iDateTime[i] = atoi(ptr1);
            ptr1 = &ptr2[1];
          }
          else
          {
            if (i == 5)
              iDateTime[5] = atoi(ptr1);
            break;
          }
        }

        rtc.adjust(DateTime(iDateTime[0], iDateTime[1], iDateTime[2], iDateTime[3], iDateTime[4], iDateTime[5]));

        if (SD.exists("PRVCLOCK.CSV"))
        {
          SD.remove("PRVCLOCK.CSV");
        }
        SD.remove("CLOCK.CSV");

        fileSDCard = SD.open("PRVCLOCK.CSV", FILE_WRITE);
        if (fileSDCard != 0)
        {
          fileSDCard.println("\"Year\",\"Month\",\"Day\",\"Hour\",\"Minute\",\"Second\"");
          fileSDCard.println(strClockSettingCopy);
          fileSDCard.close();
        }

        Serial.println("* processed *");
        lcd.print("* processed *");
      }
      else
      {
        fileSDCard.close();
      }
    }

  }
  else
  {
    Serial.println("* does not exist");
    lcd.print("* does not exist");
  }

//  delay(STARTUP_PAUSE_DELAY);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // LOGGING.CSV file processing
  {
    char szLOGGING[32] = "LOGGING.CSV";


    Serial.print("* Test ");
    Serial.println(szLOGGING);

    fileSDCard = SD.open(szLOGGING);
    if (fileSDCard)
    {
      if (fileSDCard.available())
      {
      }
      fileSDCard.close();
    }
    else
    {
      fileSDCard = SD.open(szLOGGING, FILE_WRITE);
      if (fileSDCard)
      {
        fileSDCard.println("\"DateTime\",\"Operation\",\"SetPoint1\",\"Temp1\",\"SetPoint2\",\"Temp2\",\"SetPoint3\",\"Temp3\"");
        fileSDCard.close();
      }
      else
      {
        Serial.println("*** ERROR ***   ");
        Serial.println("SD Write Failed ");
        while (1);
      }
    }
    Serial.println("* processed *");
  }

  SD.end();

  Serial.println("*** STATUS ***  ");
  Serial.println("SD Init Finish  ");

  //SDLogging(currentYear, currentMonth, currentDay, currentHour, currentMin, currentSec, "StartUp", 0.1, 0.1, 0.2, 0.2, 0.3, 0.3);
}

void SDLogging(int iYear, int iMonth, int iDay, int iHour, int iMinute, int iSecond, char *szOp, float fSetPoint1, float fTemp1, float fSetPoint2, float fTemp2, float fSetPoint3, float fTemp3)
{
  if (!SD.begin(chipSelectSDCard))
  {
    Serial.println("SD LogFail");
    return;
  }
  bSDLogFail = false;


  {
    char szLOGGING[32] = "LOGGING.CSV";

    fileSDCard = SD.open(szLOGGING, FILE_WRITE);

    // if the file opened okay, write to it:
    if (fileSDCard)
    {
      SDCardPrintTwoDigits(iYear);
      fileSDCard.print("-");
      SDCardPrintTwoDigits(iMonth);
      fileSDCard.print("-");
      SDCardPrintTwoDigits(iDay);
      fileSDCard.print("T");
      SDCardPrintTwoDigits(iHour);
      fileSDCard.print(":");
      SDCardPrintTwoDigits(iMinute);
      fileSDCard.print(":");
      SDCardPrintTwoDigits(iSecond);
      fileSDCard.print(",");
      fileSDCard.print(szOp);
      fileSDCard.print(",");
      fileSDCard.print(fSetPoint1);
      fileSDCard.print(",");
      fileSDCard.print(fTemp1);
      fileSDCard.print(",");
      fileSDCard.print(fSetPoint2);
      fileSDCard.print(",");
      fileSDCard.print(fTemp2);
      fileSDCard.print(",");
      fileSDCard.print(fSetPoint3);
      fileSDCard.print(",");
      fileSDCard.print(fTemp3);
      fileSDCard.println("");
      fileSDCard.close();
    }
    else
    {
      // if the file didn't open, display an error:
      Serial.println("*** ERROR ***   ");
      Serial.println("Open LOGGING.CSV");
    }
  }

  SD.end();

}


#endif
