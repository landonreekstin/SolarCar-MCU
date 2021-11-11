// Author: Landon Reekstin
// Contributors: Delwys Glokpor, Brian Kamusinga

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <canAddresses.h>
#include <SD.h>
#include <SPI.h>
#include <TimeLib.h>
#include <Time.h>
#include <string>
#include <sstream>
#include <cstdarg>
#include <avr/wdt.h>
using namespace std;

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3; // can3 port 
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2; // can2 port VFM channel
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1; // can1 port Main CAN channel

IntervalTimer timer;

// Pins
#define BKP_DIAG    2
#define BKP_ON_OFF  3   //turns the back up supply on or off
#define SW3         4
#define SW2         5         // Emergency disconnect
#define CHGP_SIG    6   //detects the charge power signal, high when charge power is enabled
#define RDP_SIG     7   //detects the ready power signal, high whe ready power is enabled
#define MP_E_SIG    8   //detects the MPE- enable signal, low when enabled
#define PWR5_STAT   9   
#define PWR5_ON_OFF 10  // controls pwr 5
#define PWR4_STAT   11
#define PWR4_ON_OFF 12  // controls pwr 4 
#define led         13
#define HAZARDS     14  //Input for the harzard signal, ACTIVE LOW when on, needs a pullup
#define DRIVER_FAN  15  //Input for Driver Fan, ACTIVE LOW when on, needs a pullup
#define DTLIGHTS    16  //Input for Daytime running lights signal, ACTIVE LOW when on, needs a pullup
#define CAR_RESET   17   
#define FAN_STAT    20
#define FAN_ON_OFF  21  //Input for the harzard signal, ACTIVE LOW when on, needs a pullup
#define PWR3_STAT   22
#define PWR3_ON_OFF 23  // controls pwr 3
#define PWR2_STAT   24
#define PWR2_ON_OFF 25  // controls pwr 2
#define PWR1_STAT   26
#define PWR1_ON_OFF 27  // controls pwr 1
#define MOTORON_SIG 34  // signals that the motor has been turned on, HIGH when on
#define CHG_E_SIG   35  //Charge enable signal
#define TOGGLE_12V  36         // 12v_ON/OFF
#define DIAG_12V    37         // 12V_DIAG
#define CSENSE_12V  38         // 12V_CSENSE
#define DSG_E_SIG   39   //Discharge enable signal
#define BKP_CSENSE  40
#define ACTIVECOOLING 41  //Active cooling button, only comes one when pack temperature is about 40C

//These allow for the mcu to be restarted
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

//Conversion Factors
#define MPH_MS_CONV 2.237
#define KPH_MS_CONV 3.6
#define METERS_TO_MILES_CONV 0.000621371
#define METERS_TO_KM_CONV 0.001
#define TIRE_DIAMETER_IN_METERS 0.62

// Pedal Constants
#define PEDAL_SLACK 2
#define REGEN_PEDAL_MAX 397
#define ACCELERATOR_PEDAL_MAX 287
#define REGEN_PEDAL_MIN 300
#define ACCELERATOR_PEDAL_MIN 186

// AC Constant
#define AC_MAX_TEMP 40

// for 4D display screen
#define CUR_MULT 0.1

int speedSelect = 0;   // 0 for MPH | 1 for KPH
int currentSelect = 0; // 0 for net | 1 for MC

#define DUAL_MOTOR true
boolean bShowCells = false;

//Variables for VFM
unsigned int VFM_GEAR_INDICATOR[8] = {0x00, 0x0E, 0x1C, 0x2A, 0x38, 0x46, 0x54, 0x64}; //percentage out
//VFM errors
String VFM_ERRORS[] = {"Sensor Error", "Power Error", "Motor Error"};
String errorMessage = "";

String MC_ERRORS[] = {"HWOC", "SWOC", "DC BUS OV", "HALL", "WATCHDOG", "CONFIG ERR", "15V RAIL UV", "DESAT....BAD!!"};
//motor status data
int limitFlags, errorFlags, activeMotor, canSendErr, canRecErr;
//motor status data
int limitFlags1, errorFlags1, activeMotor1, canSendErr1, canRecErr1;

union fVals
{
  byte b[4];
  float fVal;
} motorCurrent, BusCurrent, MotorRPM, busVolt, busCur, vel, mcCur, mcTemp, mcOdo, mcAh, dcCommand, mcVolt, mcRPM, bus1Volt, bus1Cur, vel1, mc1Cur, mc1Temp, mc1Odo, mc1Ah, dc1Command, mc1Volt, mc1RPM;

union inVals
{
  byte b[2];
  int inVal;
} BATDISCurrent, BATCHGCurrent, ChargerCurrent, ChargerVoltage, BrakesFront, BrakesRear;

//universal write variables
int uni_Vel, uni_Amp;

int iCount = 0;
int iCellCtr = 0;

//KEYvariables
int mcSpeed = 0;
int Voltage = 0;
int Current = 0;

String currentMC0Errors = "";
String currentMC1Errors = "";
int currentMC0Limits = 0;
int currentMC1Limits = 0;

int CurrentDriveMode = 0;

int CurrentBATDISCurrent = 0;
int CurrentBATCHGCurrent = 0;

int currentOdometer = 0;
int currentPowerUse = 0;
int currentGear = 0;

// Switch Panel Variables
bool motorSig = 0;
bool chgPSig = 0;
bool rdpSig = 0;
bool activeCooling = 0;
bool hazards = 0;
bool driverFan = 0;
bool dtLights = 0;
bool carReset = 0;
bool hornSig = 0;   // not part of switch panel but checked with them

bool switchPanelArr[8] = {motorSig, chgPSig, rdpSig, activeCooling, hazards, driverFan, dtLights, carReset};

bool strobeOn = true; // on by default

// battery pack data 1
int packVolt, packCur, packAHr;

// battery pack data 2
byte relayStatus, currentLimitStatus1, currentLimitStatus2;
int dischargeRelay, chargeRelay, iBPSCounter, dischargeLimit, chargeLimit, currentHold,    BMSmalfunction, batSOC;

// battery pack data 3
int H_Volt, L_Volt, A_Volt;
#define CELL_MULTI_VOLT .0001

//battery pack data 4
int H_Temp, L_Temp, A_Temp;

void hornTest() {
  digitalWrite(PWR4_ON_OFF, HIGH);
  digitalWrite(PWR5_ON_OFF, HIGH);
  delay(2000);
  digitalWrite(PWR4_ON_OFF, LOW);
  digitalWrite(PWR5_ON_OFF, LOW);
}

/**
 * @brief 
 * Broadcast message that the emergency disconect button has been push.
 * 
 */
void CarEmergencyBroadcast() {
  CAN_message_t EMG;
  EMG.id = EMERGENCY;
  for (size_t i = 0; i < 7; i++)
  {
    EMG.buf[i] = 0x32;
  }
  strobeOn = true;
  can1.write(EMG);
  //can2.write(EMG);
  can3.write(EMG);
  can1.events();
  //can2.events();
  can3.events();
  Serial.println("Emergency Disconect button has been pushed");
}

/**
 * @brief 
 * What happens when the emergency disconect button has been push.
 * checks for the switches state before doing anything else because
 * the pins will be on the wrong state until the array is plugged
 * in.
 */
void emergencyDisconnect() {
  //int read = digitalRead(SW3);
  // debug block
  Serial.println("SW2 now is " + String(digitalRead(SW2)));
  Serial.println("SW3 now is " + String(digitalRead(SW3)));
  // end of debug block
  //delay(2000);
  //if (digitalRead(SW3) == read && read == HIGH){
    //if( !digitalRead(SW3)){
      // switches are pulled LOW so array is connected
      digitalWrite(TOGGLE_12V, LOW);

      while(true){ // broadcast EMGCY message
        CarEmergencyBroadcast();
        delay(200);
      }
    //}
 // }
}

void toggleMotorSig() {
  motorSig = !motorSig;
}

void toggleChgpSig() {
  chgPSig = !chgPSig;
}

void toggleRdpSig() {
  rdpSig = !rdpSig;
}

/**
 * When Activecooling HI, checks highest pack cell temperature when activeCooling is HI. If it exceeds 40 C, spare 12V is activated.
*/
void toggleActiveCooling() {
  activeCooling = !activeCooling;
}

// Checks the state of active low switches on the switch panel and sets their respective pins high or sends a can message with the state
void activeLowCheck() {
  //When DTLights is LO, sends DTLights cmd to DT_LIGHTS msg.
  dtLights = !digitalRead(DTLIGHTS);
  //When DriversFAN is LO, turns on the fan.
  digitalWrite(FAN_ON_OFF, !digitalRead(DRIVER_FAN));
  // Activecooling
  if (H_Temp > AC_MAX_TEMP) {
    digitalWrite(PWR1_ON_OFF, !digitalRead(ACTIVECOOLING));
    digitalWrite(PWR2_ON_OFF, !digitalRead(ACTIVECOOLING));
  }
  // Hazards
  hazards = !digitalRead(HAZARDS);
}

// Checks the state of the switches on the panel and executes coresponding functionality.
void switchPanelCheck() {

  // reset
  if (carReset == 1) {
    CPU_RESTART;
  }

  // horn
  if (hornSig) {
    digitalWrite(PWR4_ON_OFF, HIGH);
    digitalWrite(PWR5_ON_OFF, HIGH);
  }
  else {
    digitalWrite(PWR4_ON_OFF, LOW);
    digitalWrite(PWR5_ON_OFF, LOW);
  }
}

void calculateCurrent()
{

  if (DUAL_MOTOR)
  {
    if (currentSelect == 0)
      uni_Amp = (mcCur.fVal + mc1Cur.fVal) * 10.0;
    else
      uni_Amp = busCur.fVal * 10.0;
  }
  else
  {
    if (currentSelect == 0)
      uni_Amp = mcCur.fVal * 10.0;
    else
      uni_Amp = busCur.fVal * 10.0;
  }

  // write pack Current
  if ((uni_Amp * CUR_MULT) < 0)
  {
    //genie.WriteObject(GENIE_OBJ_USER_LED, 4, 1);
  } // led is turned on when pack current is negative
  else
  {
    //genie.WriteObject(GENIE_OBJ_USER_LED, 4, 0);
  } // led is off when current is positive
  Serial.print("Current sent to display:");
  Serial.println(abs(uni_Amp));
  //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 0, abs(uni_Amp));
}

void calculateSpeed() {
  if (DUAL_MOTOR)
  {
    if (speedSelect == 0)
      uni_Vel = (mc1RPM.fVal + mcRPM.fVal) / 2.0 * TIRE_DIAMETER_IN_METERS * M_PI * 1 / 60.0 * MPH_MS_CONV;
    else
      uni_Vel = (mc1RPM.fVal + mcRPM.fVal) / 2.0 * TIRE_DIAMETER_IN_METERS * M_PI * 1 / 60.0 * KPH_MS_CONV;
  }
  else
  {
    if (speedSelect == 0)
      uni_Vel = mcRPM.fVal * 0.7 * TIRE_DIAMETER_IN_METERS * 1 / 60 * MPH_MS_CONV;
    else
      uni_Vel = mcRPM.fVal * 0.7 * TIRE_DIAMETER_IN_METERS * 1 / 60 * KPH_MS_CONV;
  }

  if (uni_Vel < 0)
    Serial.println("Velocity is negative");
    //genie.WriteObject(GENIE_OBJ_USER_LED, 2, 1); // led is turned on when velocity is negative
  else
  {
    // write velocity
    Serial.println("Velocity is positive");
  }
  Serial.print("SPEED : ");
  Serial.println(uni_Vel);
}

void calculate_odo_and_poweruse()
{
  if (DUAL_MOTOR)
  {
    currentPowerUse = int((mc1Ah.fVal + mcAh.fVal) * 100);
    if (speedSelect == 0)
    {
      currentOdometer = int((mc1Odo.fVal + mcOdo.fVal) / 2 * METERS_TO_MILES_CONV * 10);
    }
    else
    {
      currentOdometer = int((mc1Odo.fVal + mcOdo.fVal) / 2 * METERS_TO_KM_CONV * 10);
    }
  }
  else
  {
    currentPowerUse = int(mcAh.fVal * 100);

    if (speedSelect == 0)
    {
      currentOdometer = (mcOdo.fVal * METERS_TO_MILES_CONV * 10);
    }
    else
    {
      currentOdometer = (mcOdo.fVal * METERS_TO_KM_CONV * 10);
    }
  }

  //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 8, currentOdometer);
  //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 9, currentPowerUse);
  Serial.print("Odometer in 0.1  : ");
  Serial.println(currentOdometer);
  Serial.print("Power use in 0.01 Amp Hours : ");
  Serial.println(currentPowerUse);
}

// Send status of switches on switch panel to CAN to be logged.
void sendSwitchPanel() {
  CAN_message_t SwitchPanelMSG;
  SwitchPanelMSG.id = SWITCH_PANEL;
  bitWrite(SwitchPanelMSG.buf[0],0, motorSig);
  bitWrite(SwitchPanelMSG.buf[0],1, chgPSig);
  bitWrite(SwitchPanelMSG.buf[0],2, rdpSig);
  bitWrite(SwitchPanelMSG.buf[0],3, activeCooling);
  bitWrite(SwitchPanelMSG.buf[0],4, hazards);
  bitWrite(SwitchPanelMSG.buf[0],5, driverFan);
  bitWrite(SwitchPanelMSG.buf[0],6, dtLights);
  bitWrite(SwitchPanelMSG.buf[0],7, carReset);

  for (size_t i = 1; i < 8; i++) {
    SwitchPanelMSG.buf[i] = 0x00;
  }

  can1.write(SwitchPanelMSG);
}

void sendStrobe() {
  CAN_message_t StrobeMSG;
  StrobeMSG.id = STROBE;
  const int strobeCMD = 0x77;
  if (strobeOn) {
    StrobeMSG.buf[0] = strobeCMD;
  }
  else {
    StrobeMSG.buf[0] = 0x00;
  }
  for (size_t i = 1; i < 7; i++) {
    StrobeMSG.buf[i] = 0x00;
  }
  can1.write(StrobeMSG);
}

// Sends data to CAN Bus
void sendframe() {
  sendSwitchPanel();
  switchPanelCheck();
  activeLowCheck();
  sendStrobe();
}

// Reads all CAN messages to log on SD
void ReadCanBus(const CAN_message_t &msg) {
  //Serial.println("Inside readcanbus");
  switch (msg.id) {
    case DC_RESET:
      CPU_RESTART;
    break;

    case SWITCH_PANEL:
    {
      motorSig = bitRead(msg.buf[0], 0);
      chgPSig = bitRead(msg.buf[0], 1);
      rdpSig = bitRead(msg.buf[0], 2);
      activeCooling = bitRead(msg.buf[0], 3);
      hazards = bitRead(msg.buf[0], 4);
      driverFan = bitRead(msg.buf[0], 5);
      dtLights = bitRead(msg.buf[0], 6);
      carReset = bitRead(msg.buf[0], 7);
    }
    break;

    case DC_SWITCH:
    {
      int driveMode = 1;
      int regenSignal = 0;
      bool brakeSignal = 0;
      //string driveModeStr;

      if (bitRead(msg.buf[0], 0)) // reverse
      {
        driveMode = 1;
        //driveModeStr = "driveMode=R";
      }
      else if (bitRead(msg.buf[0], 1)) // neutral
      {
        driveMode = 0;
        //driveModeStr = "driveMode=N";
      }
      else if (bitRead(msg.buf[0], 3)) // forward
      {
        driveMode = 2;
        //driveModeStr = "driveMode=F";
      }

      if (driveMode != CurrentDriveMode)
      {
        CurrentDriveMode = driveMode;
      }

      if (bitRead(msg.buf[0], 2))   // regen
      {
        regenSignal = 1;
      }
      else 
      {
        regenSignal = 0;
      }

       if (bitRead(msg.buf[0], 7))   // brakes, ommit brakeSensor code will be used insted
      {
        brakeSignal = 1;
      }
      else 
      {
        brakeSignal = 0;
      }

      if (msg.buf[2] == 0x32) {
        Serial.println("Horn signal received");
        hornSig = true;
      }
      else if (msg.buf[2] == 0x00) {
        hornSig = false;
      }
      
    }
    break;

    case DC_DRIVE:
      for (uint8_t i = 0; i < 4; i++)
      {
        MotorRPM.b[i] = msg.buf[i];
      }
      int motorRPM = (int)MotorRPM.fVal;

      for (uint8_t i = 0; i < 4; i++)
      {
        motorCurrent.b[i] = msg.buf[i + 4];
      }
      int MotorCurrent = (int)motorCurrent.fVal;

      // calc pedal position
      int acceleratorValue = motorCurrent.fVal * (ACCELERATOR_PEDAL_MAX - ACCELERATOR_PEDAL_MIN);
      int regenValue = motorCurrent.fVal * (REGEN_PEDAL_MAX - REGEN_PEDAL_MIN);  
    break;

    case DC_POWER:
      for (uint8_t i = 0; i < 4; i++)
      {
        BusCurrent.b[i] = msg.buf[i + 4];
      }
      int busCurrent = (int)BusCurrent.fVal;
    break;

    case DC_VFM:
      can3.write(msg);
    break;

    case VFM:
    {
      can1.write(msg);
      unsigned int VFM_Stator_displacement_percentage = 0x00;
      int gear = 0;
      VFM_Stator_displacement_percentage = msg.buf[0];
      Serial.print("Current % stator displacement:");
      Serial.println(VFM_Stator_displacement_percentage);

      for (int i = 0; i < 8; i++)
      {
        if (VFM_Stator_displacement_percentage == VFM_GEAR_INDICATOR[i])
        {
          gear = i + 1;
          if (currentGear != gear)
          {
            Serial.println(gear);
            currentGear = gear;
            break;
          }
        }
      }
      Serial.print("VFM Errors Present:");

      int errorCount = 0;

      for (int i = 1; i < 4; i++)
      {
        if (!bitRead(msg.buf[1], i))
        {
          errorMessage = errorMessage + VFM_ERRORS[i - 1] + ": ";
          errorCount++;
        }
      }
      if (errorCount == 0)
      {
        Serial.println("None");
        {
          Serial.println(errorMessage);
        }
      }
    }
    break;

    case MC_VELOCITY:
      for (int i = 0; i < 4; i++)
      {
        mcRPM.b[i] = msg.buf[i];
        vel.b[i] = msg.buf[i + 4];
      }

      calculateSpeed();
    break;

    case MC1_VELOCITY:
      for (int i = 0; i < 4; i++)
      {
        mc1RPM.b[i] = msg.buf[i];
        vel1.b[i] = msg.buf[i + 4];
      }

      calculateSpeed();
    break;
    //-------------------------------------------------------------------------------
    // Tritium motor controller voltage data
    // 0-3: motor bus voltage
    // 4-7: motor bus amperage

    case MC_BUS:
      for (int i = 0; i < 4; i++)
      {
        mcVolt.b[i] = msg.buf[i];
        mcCur.b[i] = msg.buf[i + 4];
      }
      Serial.print("Motor Controller 0 Voltage :");
      Serial.println(mcVolt.fVal);
      Serial.print("Motor Controller 0 Current :");
      Serial.println(mcCur.fVal);
      calculateCurrent();
    break;

    case MC1_BUS:
      for (int i = 0; i < 4; i++)
      {
        mc1Volt.b[i] = msg.buf[i];
        mc1Cur.b[i] = msg.buf[i + 4];
      }
      Serial.print("Motor Controller 1 Voltage :");
      Serial.println(mc1Volt.fVal);
      Serial.print("Motor Controller 1 Current :");
      Serial.println(mc1Cur.fVal);
      calculateCurrent();
    break;
    //-------------------------------------------------------------------------------
    // wavesculpt leddigit14
    // 0-3: motor temp
    // 4-7: heatsink temp
    case MC_TEMP1:
      for (int i = 0; i < 4; i++)
      {
        mcTemp.b[i] = msg.buf[i + 4];
      }
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 14, (int)(mcTemp.fVal * 10));
      Serial.print("Motor Controller 1 IPM Temperature :");
      Serial.println(mcTemp.fVal);
    break;

    //-------------------------------------------------------------------------------
    // wavesculpt leddigit15
    // 0-3: motor temp
    // 4-7: heatsink temp
    case MC1_TEMP1:
      for (int i = 0; i < 4; i++)
      {
        mc1Temp.b[i] = msg.buf[i + 4];
      }
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 15, (int)(mc1Temp.fVal * 10));
      Serial.print("Motor Controller 1 IPM Temperature :");
      Serial.println(mc1Temp.fVal);
    break;

    //-------------------------------------------------------------------------------
    // Tritium motor controller status data
    // 7: Can receive error
    // 6: Can send error
    // 4-5: Active Motor
    // 2-3: Error Flags
    // 0-1: Limit Flags

    case MC_STATUS:
    {

      limitFlags = msg.buf[0];  //
      errorFlags = msg.buf[2];  //
      activeMotor = msg.buf[4]; //
      canSendErr = msg.buf[6];
      canRecErr = msg.buf[7];
      String mc0Errors = " ";
      for (size_t i = 0; i < 7; i++)
      {
        if (bitRead(errorFlags, i))
        {
          mc0Errors = mc0Errors + MC_ERRORS[i] + ": ";
        }
      }

      if (currentMC0Errors != mc0Errors)
      {
        //genie.WriteStr(2, mc0Errors);
      }
      currentMC0Errors = mc0Errors;

      if (limitFlags != currentMC0Limits)
      {

        for (size_t i = 0; i < 7; i++)
        {
          if (bitRead(limitFlags, i))
          {
            //genie.WriteObject(GENIE_OBJ_STRINGS, 3, i + 1);
            break;
          }
        }
        currentMC0Limits = limitFlags;
      }
      else if (limitFlags == 0)
      {
        //genie.WriteObject(GENIE_OBJ_STRINGS, 3, 0);
      }
    }
    break;

    case MC1_STATUS:
    {
      limitFlags1 = msg.buf[0];  //
      errorFlags1 = msg.buf[2];  //
      activeMotor1 = msg.buf[4]; //
      canSendErr1 = msg.buf[6];
      canRecErr1 = msg.buf[7];

      String mc1Errors = " ";
      for (size_t i = 0; i < 7; i++)
      {
        if (bitRead(errorFlags1, i))
        {
          mc1Errors = mc1Errors + MC_ERRORS[i] + ":";
        }
      }

      if (currentMC1Errors != mc1Errors)
      {
        //genie.WriteStr(2, mc1Errors);
      }
      currentMC1Errors = mc1Errors;
      if (limitFlags != currentMC1Limits)
      {

        for (size_t i = 0; i < 7; i++)
        {
          if (bitRead(limitFlags1, i))
          {
            //genie.WriteObject(GENIE_OBJ_STRINGS, 5, i + 1);
            break;
          }
        }
      }
      else if (limitFlags1 == 0)
      {
        //genie.WriteObject(GENIE_OBJ_STRINGS, 5, 0);
      }
      currentMC1Limits = limitFlags;
    }
    break;
    // Tritium motor controller Odo and Ah measurements - base address + 14
    // 22-25: Distance the vehicle has travelled since reset in m
    // 26-29: Charge flow into the controller DC bus from the time of reset in Ah

    case MC1_DIST:
      for (int i = 0; i < 4; i++)
      {
        mc1Odo.b[i] = msg.buf[i];
        mc1Ah.b[i] = msg.buf[i + 4];
      }
      calculate_odo_and_poweruse();
    break;

    case MC_DIST:

      // read data bytes
      for (int i = 0; i < 4; i++)
      {
        mcOdo.b[i] = msg.buf[i];
        mcAh.b[i] = msg.buf[i + 4];
      }
      calculate_odo_and_poweruse();
    break;

    case BMS_PACK_1:

      // convert byte[2] to short int
      packCur = ((int)msg.buf[0] << 8) | (int)msg.buf[1];
      // write pack voltage
      packVolt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
      Serial.print("Battery Pack Current:");
      Serial.println(packCur);
      Serial.print("Battery Pack Voltage");
      Serial.println(packVolt);
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 1, packVolt);
      calculateCurrent();
      
      //SOC as calculated by BMS
 
      if(batSOC!=msg.buf[4])
      {
      batSOC = msg.buf[4];
      //genie.WriteObject(GENIE_OBJ_GAUGE, 0, int(batSOC*0.5));
      //genie.WriteStr(10, String(round(batSOC*0.5))+"%");
      }
      Serial.print("Battery Pack SOC:");
      Serial.println(String(batSOC*0.5) +"%");
      
      //Relay Status
      relayStatus = msg.buf[6];

      dischargeRelay = bitRead(relayStatus, 0);
      Serial.println("relay status: " + String(dischargeRelay));
      if (dischargeRelay == 1)
      { //genie.WriteObject(GENIE_OBJ_USER_LED, 0, 1); // led is turned on when discharge is on
        Serial.print("discharge is on:");
        strobeOn = false;
      }
      else
      { //genie.WriteObject(GENIE_OBJ_USER_LED, 0, 0); // led is off when discharge is off
        Serial.print("discharge is off:");
        strobeOn = true;
      }
      
      // charge enable
      chargeRelay = bitRead(relayStatus, 1);
      if (chargeRelay == 1)
        {//genie.WriteObject(GENIE_OBJ_USER_LED, 1, 1); // led is turned on when charge is on
          Serial.print("charge is on:");
        }
      else
        {//genie.WriteObject(GENIE_OBJ_USER_LED, 1, 0); // led is off when charge is off
          Serial.print("charge is off:");
        }
    
      //if (BMSmalfunction == 1)
        //genie.WriteObject(GENIE_OBJ_USER_LED, 3, 1); // led is turned on when charge is on
      //else
        //genie.WriteObject(GENIE_OBJ_USER_LED, 3, 0); // led is off when charge is off
    break;

    //-------------------------------------------------------------------------------
    // Orion BMS data pack 2/C
    // 0: 
    case BMS_PACK_2:
      packAHr = ((unsigned int)msg.buf[4] << 8) | (unsigned int)msg.buf[5];
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS,13, (int)(packAHr * 10));
    break;

    //-------------------------------------------------------------------------------
    // Orion BMS data pack 3/D
    //
    case BMS_PACK_3:
      H_Volt = ((unsigned int)msg.buf[0] << 8) | (unsigned int)msg.buf[1];
      L_Volt = ((unsigned int)msg.buf[2] << 8) | (unsigned int)msg.buf[3];
      A_Volt = ((unsigned int)msg.buf[4] << 8) | (unsigned int)msg.buf[5];
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 5, (int)(H_Volt / 10));
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 7, (int)(L_Volt / 10));
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 6, (int)(A_Volt / 10));

      Serial.print("Highest Cell volatge in 0.1mv: ");
      Serial.println(H_Volt);
      Serial.print("Lowest Cell Voltage in 0.1mv: ");
      Serial.println(L_Volt);
      Serial.print("Average Cell Voltage in 0.1mv: ");
      Serial.println(A_Volt);
    break;

    //-------------------------------------------------------------------------------
    // Orion BMS data pack 4/E
    //
    case BMS_PACK_4:
      H_Temp = (unsigned int)msg.buf[2];
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 3, H_Temp * 10);
      L_Temp = (unsigned int)msg.buf[3];
      //genie.WriteObject(GENIE_OBJ_LED_DIGITS, 4, L_Temp);
      Serial.print("Highest Cell Temperature: ");
      Serial.println(H_Temp);
      Serial.print("Lowest Cell Temperature: ");
      Serial.println(L_Temp);
    break;

    case BRAKES_FRONT:
      BrakesFront.b[0] = msg.buf[0];    // MSB
      BrakesFront.b[1] = msg.buf[1];    // LSB
      int FrontPSI = BrakesFront.inVal;
    break;

    case BRAKES_REAR:
      BrakesRear.b[0] = msg.buf[0];    // MSB
      BrakesRear.b[1] = msg.buf[1];    // LSB
      int RearPSI = BrakesRear.inVal;
    break;

    case DC_TURN_SIGNALS:
      
    break;
  }
}


void setup() {
  // Pin Initialization
  // Inputs
//INPUTS ON THE SWITCH PANEL
  //pinMode(SW1, INPUT_PULLUP); // all three switched high while array 
  pinMode(SW3, INPUT_PULLUP); // not connected
  pinMode(SW2, INPUT_PULLUP);
  pinMode(HAZARDS, INPUT_PULLUP);
  pinMode(DRIVER_FAN, INPUT_PULLUP);
  pinMode(DTLIGHTS, INPUT_PULLUP);
  pinMode(CAR_RESET, INPUT_PULLUP);
  pinMode(ACTIVECOOLING, INPUT_PULLUP);
  
  pinMode(MOTORON_SIG, INPUT_PULLUP);
  pinMode(CHGP_SIG, INPUT);
  pinMode(RDP_SIG, INPUT);

//INPUTS FROM THE BMS
  pinMode(DSG_E_SIG, INPUT_PULLUP);   // When high, strobe on
  pinMode(CHG_E_SIG, INPUT_PULLUP);
  pinMode(MP_E_SIG, INPUT_PULLUP);

  pinMode(PWR5_STAT, INPUT);
  pinMode(PWR4_STAT, INPUT);
  pinMode(BKP_DIAG, INPUT);

  pinMode(FAN_STAT, INPUT);
  pinMode(PWR3_STAT, INPUT);
  pinMode(PWR2_STAT, INPUT);
  pinMode(PWR1_STAT, INPUT);
  pinMode(DIAG_12V, INPUT);
  pinMode(CSENSE_12V, INPUT);
  pinMode(BKP_CSENSE, INPUT);

  // Outputs
  pinMode(PWR5_ON_OFF, OUTPUT);
  pinMode(PWR4_ON_OFF, OUTPUT);
  pinMode(FAN_ON_OFF, OUTPUT);
  pinMode(PWR3_ON_OFF, OUTPUT);
  pinMode(PWR2_ON_OFF, OUTPUT);
  pinMode(PWR1_ON_OFF, OUTPUT);
  pinMode(TOGGLE_12V, OUTPUT);
  pinMode(BKP_ON_OFF, OUTPUT);
  pinMode(led, OUTPUT);

  // Car Reset. Prevents the car from starting up until reset is pressed.
  while (digitalRead(CAR_RESET)) {}

  // Startup operations
  digitalWrite(BKP_ON_OFF, HIGH);
  digitalWrite(TOGGLE_12V, HIGH);
  digitalWrite(PWR3_ON_OFF, HIGH);

  // Pin interrupts
  attachInterrupt(MOTORON_SIG, toggleMotorSig, CHANGE);
  attachInterrupt(CHGP_SIG, toggleChgpSig, CHANGE);
  attachInterrupt(RDP_SIG, toggleRdpSig, CHANGE);
  attachInterrupt(ACTIVECOOLING, toggleActiveCooling, CHANGE);
  
  // CAN Initialization
  can1.begin();
  can1.setBaudRate(500000); // 250kbps data rate
  can1.enableFIFO();
  can1.enableFIFOInterrupt();
  can1.onReceive(FIFO, ReadCanBus);
  can1.mailboxStatus();


  can2.begin();
  can2.setBaudRate(500000); // 250kbps data rate VFM channel
  can2.enableFIFO();
  can2.enableFIFOInterrupt();
  can2.onReceive(FIFO, ReadCanBus);
  can2.mailboxStatus();

  can3.begin();
  can3.setBaudRate(250000); // 500kbps data rate
  can3.enableFIFO();
  can3.enableFIFOInterrupt();
  can3.onReceive(FIFO, ReadCanBus);
  can3.mailboxStatus();

  timer.begin(sendframe, 50000); // Send frame every 50ms--100ms

  analogWrite(led, 10);

  Serial.println("Setup run");
  //hornTest();
}

void loop() {
  can2.events();
  can3.events();
  // debug block
  Serial.println("SW2 state is " + String(digitalRead(SW2)));
  Serial.println("SW3 state is " + String(digitalRead(SW3)));
  // end of debug block
  delay(2);
}