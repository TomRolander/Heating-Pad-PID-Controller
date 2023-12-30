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

#define DEBUG_OUTPUT  1

void(* resetFunc) (void) = 0;

// Rotary Encoder Inputs
#define CLK 2
#define DT 3
#define SW 4

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
long samples = 0;
int iNmbSamples = 50; //50; //25; //50;

#include <Adafruit_MAX31856.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 max[3] = {
  Adafruit_MAX31856(10, 11, 12, 13),
  Adafruit_MAX31856( 9, 11, 12, 13),
  Adafruit_MAX31856( 8, 11, 12, 13)
};

#include <PID_v1.h>

#define RELAY_1 21
#define RELAY_2 20
#define RELAY_3 19

int Relay_Pins[3] = {RELAY_1,RELAY_2,RELAY_3};
int iRelay = 0;

  //initialize the variables we're linked to
double Setpoint[3] = {28.0, 28.0, 28.0};
double SetpointNew = 0;

int DisabledHeatPads[3] = {0,0,0};

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
      default: Serial.println("Unknown"); break;
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

  DisplaySetpoints();

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
    if (DisabledHeatPads[i])
      continue;

    switch (i)
    {
      case 0:
        myRA_1.addValue(max[i].readThermocoupleTemperature());
        Input[i] = myRA_1.getAverage();
        break;
      case 1:
        myRA_2.addValue(max[i].readThermocoupleTemperature());
        Input[i] = myRA_2.getAverage();
        break;
      case 2:
        myRA_3.addValue(max[i].readThermocoupleTemperature());
        Input[i] = myRA_3.getAverage();
        break;
    }
    Average[i] = Input[i];

    // Check and print any faults
    uint8_t fault = max[i].readFault();
    if (fault) {
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
    if (DisabledHeatPads[i])
    {
      lcd.setCursor(iOffset, 1);
      lcd.print("    ");
    }
    else
      showValue(1, iOffset,Average[i]);
  }

  samples++;
  if ((samples % iNmbSamples) == 0)
  {
    Serial.print("Average_1:");
    Serial.print(Average[0]);
    Serial.print(",");
    Serial.print("Average_2:");
    Serial.print(Average[1]);
    Serial.print(",");
    Serial.print("Average_3:");
    Serial.print(Average[2]);
    Serial.print(",");
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
    if (DisabledHeatPads[i] == 1)
    {
      lcd.setCursor(iOffset, 0);
      lcd.print("OFF ");
    }
    else
      showValue(0, iOffset,Setpoint[i]);
  }
}