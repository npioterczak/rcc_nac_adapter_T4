//
// Created by norbert on 12.08.22.
//

#include "Peugeot_CAN.h"
#include <TimeLib.h>
#include <FlexCAN_T4.h>
#include <EEPROM.h>

#define SERIAL_SPEED 9600

// Use previously declared Teensy CAN Lib variables
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can2004;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2010;

CAN_message_t canMsgSnd;
CAN_message_t canMsgRcv;
bool debugGeneral = false; // Get some debug informations on Serial
bool debugCAN2004 = false; // Read data sent by ECUs from the car to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool debugCAN2010 = false; // Read data sent by the NAC / SMEG to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool EconomyModeEnabled = true; // You can disable economy mode on the Telematic if you want to - Not recommended at all
bool Send_CAN2010_ForgedMessages = false; // Send forged CAN2010 messages to the CAR CAN-BUS Network (useful for testing CAN2010 device(s) from already existent connectors)
bool TemperatureInF = false; // Default Temperature in Celcius
bool mpgMi = false;
bool kmL = false; // km/L statistics instead of L/100
bool noFMUX = false; // If you don't have any useful button on the main panel, turn the SRC button on steering wheel commands into MENU - only works for CAN2010 SMEG / NAC -
byte steeringWheelCommands_Type = 0; // noFMUX extra setting : 0 = Generic, 1 = C4 I / C5 X7 NAV+MUSIC+APPS+PHONE mapping, 2 = C4 I / C5 X7 MENU mapping, 3 = C4 I / C5 X7 MENU mapping + SRC on wiper command button, 4 = C4 I / C5 X7 MENU mapping + TRIP on wiper command button, 5 = C4 I / C5 X7 MENU mapping + SRC on wiper command button + TRIP on ESC button
byte languageID = 0; // Default is FR: 0 - EN: 1 / DE: 2 / ES: 3 / IT: 4 / PT: 5 / NL: 6 / BR: 9 / TR: 12 / RU: 14
bool listenCAN2004Language = false; // Switch language on CAN2010 devices if changed on supported CAN2004 devices, default: no
byte Time_day = 1; // Default day if the RTC module is not configured
byte Time_month = 1; // Default month if the RTC module is not configured
int Time_year = 2022; // Default year if the RTC module is not configured
byte Time_hour = 0; // Default hour if the RTC module is not configured
byte Time_minute = 0; // Default minute if the RTC module is not configured
bool resetEEPROM = false; // Switch to true to reset all EEPROM values
bool CVM_Emul = true; // Send suggested speed from Telematic to fake CVM (Multifunction camera inside the windshield) frame
bool generatePOPups = false; // Generate notifications from alerts journal - useful for C5 (X7)

bool emulateVIN = false; // Replace network VIN by another (donor car for example)
char vinNumber[18] = "VF3XXXXXXXXXXXXXX";

bool hasAnalogicButtons = false; // Analog buttons instead of FMUX
byte menuButton = 4;
byte volDownButton = 5;
byte volUpButton = 6;
byte scrollValue = 0;

// Default variables
bool Ignition = false;
bool SerialEnabled = false;
int Temperature = 0;
bool EconomyMode = false;
bool EngineRunning = false;
byte languageID_CAN2004 = 0;
bool AirConditioningON = false;
byte FanSpeed = 0;
bool FanOff = false;
bool AirRecycle = false;
bool DeMist = false;
bool DeFrost = false;
byte LeftTemp = 0;
byte RightTemp = 0;
bool Mono = false;
bool FootAerator = false;
bool WindShieldAerator = false;
bool CentralAerator = false;
bool AutoFan = false;
byte FanPosition = 0;
bool MaintenanceDisplayed = false;
int buttonState = 0;
int lastButtonState = 0;
long lastDebounceTime = 0;
long buttonPushTime = 0;
long buttonSendTime = 0;
long debounceDelay = 100;
int daysSinceYearStart = 0;
unsigned long customTimeStamp = 0;
double vehicleSpeed = 0;
double engineRPM = 0;
bool darkMode = false;
bool resetTrip1 = false;
bool resetTrip2 = false;
bool pushAAS = false;
bool pushSAM = false;
bool pushDSG = false;
bool pushSTT = false;
bool pushCHECK = false;
bool stopCHECK = false;
bool pushBLACK = false;
bool pushASR = false;
bool pushTRIP = false;
byte personalizationSettings[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte statusCMB[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte statusTRIP[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool TelematicPresent = false;
bool ClusterPresent = false;
bool pushA2 = false;
int alertsCache[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Max 8
byte alertsParametersCache[] = {0, 0, 0, 0, 0, 0, 0, 0}; // Max 8
bool isBVMP = false;
byte statusOpenings = 0;
byte notificationParameters = 0;
int tmpVal = 0;

// Language & Unit CAN2010 value
byte languageAndUnitNum = (languageID * 4) + 128;
void Peugeot_CAN::canSniff(const CAN_message_t &msg) {
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print("  LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
}
void Peugeot_CAN::initPeugeotCan() {
    can2004.begin();
    can2004.setClock(CLK_60MHz);
    // can2004.setMaxMB(16);
    can2004.setBaudRate(125000);
    // can2004.enableFIFO();
    // can2004.onReceive(canSniff);
// init can from nac/rcc to car entertainment
    can2010.begin();
    can2010.setClock(CLK_60MHz);
    // can2010.setMaxMB(16);
    can2010.setBaudRate(125000);
    if (resetEEPROM) {
        EEPROM.update(0, 0);
        EEPROM.update(1, 0);
        EEPROM.update(2, 0);
        EEPROM.update(3, 0);
        EEPROM.update(4, 0);
        EEPROM.update(5, 0);
        EEPROM.update(6, 0);
        EEPROM.update(7, 0);
        EEPROM.update(10, 0);
        EEPROM.update(11, 0);
        EEPROM.update(12, 0);
        EEPROM.update(13, 0);
        EEPROM.update(14, 0);
        EEPROM.update(15, 0);
        EEPROM.update(16, 0);
    }

    if (debugCAN2004 || debugCAN2010 || debugGeneral) {
        SerialEnabled = true;
    }

    // Read data from EEPROM
    tmpVal = EEPROM.read(0);
    if (tmpVal >= 128) {
        languageAndUnitNum = tmpVal;
    }

    if ((languageAndUnitNum % 2) == 0 && kmL) {
        languageAndUnitNum = languageAndUnitNum + 1;
    }

    tmpVal = EEPROM.read(1);
    if (tmpVal <= 32) {
        languageID_CAN2004 = tmpVal;
    }

    tmpVal = EEPROM.read(2);
    if (tmpVal <= 32) {
        languageID = tmpVal;
    }

    tmpVal = EEPROM.read(3);
    if (tmpVal == 1) {
        TemperatureInF = true;
    }

    tmpVal = EEPROM.read(4);
    if (tmpVal == 1) {
        mpgMi = true;
    }

    tmpVal = EEPROM.read(5);
    if (tmpVal <= 31) {
        Time_day = tmpVal;
    }

    tmpVal = EEPROM.read(6);
    if (tmpVal <= 12) {
        Time_month = tmpVal;
    }

    EEPROM.get(7, tmpVal); // int
    if (tmpVal >= 1872 && tmpVal <= 2127) {
        Time_year = tmpVal;
    }

    personalizationSettings[0] = EEPROM.read(10);
    personalizationSettings[1] = EEPROM.read(11);
    personalizationSettings[2] = EEPROM.read(12);
    personalizationSettings[3] = EEPROM.read(13);
    personalizationSettings[4] = EEPROM.read(14);
    personalizationSettings[5] = EEPROM.read(15);
    personalizationSettings[6] = EEPROM.read(16);

    if (hasAnalogicButtons) {
        //Initialize buttons - MENU/VOL+/VOL-
        pinMode(menuButton, INPUT_PULLUP);
        pinMode(volDownButton, INPUT_PULLUP);
        pinMode(volUpButton, INPUT_PULLUP);
    }

    if (SerialEnabled) {
        // Initalize Serial for debug
        Serial.begin(SERIAL_SPEED);
        Serial.print("test start");
    }

    // Set hour on CAN-BUS Clock
    canMsgSnd.buf[0] = hour();
    canMsgSnd.buf[1] = minute();
    canMsgSnd.id = 0x228;
    canMsgSnd.len = 2;
    can2004.write(canMsgSnd);

    // Send fake EMF version
    canMsgSnd.buf[0] = 0x25;
    canMsgSnd.buf[1] = 0x0A;
    canMsgSnd.buf[2] = 0x0B;
    canMsgSnd.buf[3] = 0x04;
    canMsgSnd.buf[4] = 0x0C;
    canMsgSnd.buf[5] = 0x01;
    canMsgSnd.buf[6] = 0x20;
    canMsgSnd.buf[7] = 0x11;
    canMsgSnd.id = 0x5E5;
    canMsgSnd.len = 8;
    can2004.write(canMsgSnd);

    if (SerialEnabled) {
        Serial.print("Current Time: ");
        Serial.print(day());
        Serial.print("/");
        Serial.print(month());
        Serial.print("/");
        Serial.print(year());

        Serial.print(" ");

        Serial.print(hour());
        Serial.print(":");
        Serial.print(minute());

        Serial.println();
    }

    // Init callback for frames coming from entertainment bus
    can2004.onReceive(Peugeot_CAN::canSniff);
    // Init callback for frames coming from nac/rcc
    can2010.onReceive(Peugeot_CAN::nacToCanBus);
}

void Peugeot_CAN::sendPOPup(bool present, int id, byte priority, byte parameters) {
    bool clear = false;
    byte firstEmptyPos = 8;

    for (int i = 0; i < 8; i++) {
        if (alertsCache[i] == id) {
            if (!present) { // Remove from cache and clear popup
                alertsCache[i] = alertsParametersCache[i] = firstEmptyPos = 0;
                clear = true;
                break;
            } else if (parameters == alertsParametersCache[i]) { // Already sent
                return;
            } else {
                return Peugeot_CAN::sendPOPup(false, id, priority, 0x00); // Clear previous popup first
            }
        } else if (alertsCache[i] == 0 && firstEmptyPos >= 8) {
            firstEmptyPos = i;
        }
    }

    if (firstEmptyPos >= 8) {
        return; // Avoid overflow
    }
    if (!present && !clear) {
        return;
    } else if (!clear) {
        alertsCache[firstEmptyPos] = id;
        alertsParametersCache[firstEmptyPos] = parameters;

        if (SerialEnabled && present) {
            Serial.print("Notification sent with message ID: ");
            Serial.println(id);
        }
    }

    if (priority > 14) {
        priority = 14;
    }

    if (present) {
        canMsgSnd.buf[0] = highByte(id);
        canMsgSnd.buf[1] = lowByte(id);
        bitWrite(canMsgSnd.buf[0], 7, present); // New message
    } else { // Close Popup
        canMsgSnd.buf[0] = 0x7F;
        canMsgSnd.buf[1] = 0xFF;
    }
    canMsgSnd.buf[2] = priority; // Priority (0 > 14)
    bitWrite(canMsgSnd.buf[2], 7, 1); // Destination: NAC / EMF / MATT
    bitWrite(canMsgSnd.buf[2], 6, 1); // Destination: CMB
    canMsgSnd.buf[3] = parameters; // Parameters
    canMsgSnd.buf[4] = 0x00; // Parameters
    canMsgSnd.buf[5] = 0x00; // Parameters
    canMsgSnd.buf[6] = 0x00; // Parameters
    canMsgSnd.buf[7] = 0x00; // Parameters
    canMsgSnd.id = 0x1A1;
    canMsgSnd.len = 8;
    can2010.write(canMsgSnd);
}

void Peugeot_CAN::canBusToNac(const CAN_message_t &msg) {
    int id = msg.id;
    int len = msg.len;

    if (debugCAN2004) {
        Serial.print("FRAME:ID=");
        Serial.print(id);
        Serial.print(":LEN=");
        Serial.print(len);

        char tmp[3];
        for (int i = 0; i < len; i++) {
            Serial.print(":");

            snprintf(tmp, 3, "%02X", msg.buf[i]);

            Serial.print(tmp);
        }

        Serial.println();

        can2010.write(msg);
    } else if (!debugCAN2010) {
        if (id == 0x15B) {
            // Do not send back converted frames between networks
        } else if (id == 0x36 && len == 8) { // Economy Mode detection
            if (bitRead(msg.buf[2], 7) == 1) {
                if (!EconomyMode && SerialEnabled) {
                    Serial.println("Economy mode ON");
                }

                EconomyMode = true;
            } else {
                if (EconomyMode && SerialEnabled) {
                    Serial.println("Economy mode OFF");
                }

                EconomyMode = false;
            }

            tmpVal = msg.buf[3];

            can2010.write(msg);
        } else if (id == 0xB6 && len == 8) {
            engineRPM = ((msg.buf[0] << 8) | msg.buf[1]) * 0.125;
            if (engineRPM > 0) {
                EngineRunning = true;
            } else {
                EngineRunning = false;
            }
            vehicleSpeed = ((msg.buf[2] << 8) | msg.buf[3]) * 0.01;
            can2010.write(msg);
        } else if (id == 0x336 && len == 3 && emulateVIN) { // ASCII coded first 3 letters of VIN
            canMsgSnd.buf[0] = vinNumber[0]; //V
            canMsgSnd.buf[1] = vinNumber[1]; //F
            canMsgSnd.buf[2] = vinNumber[2]; //3
            canMsgSnd.id = 0x336;
            canMsgSnd.len = 3;
            can2010.write(canMsgSnd);
        } else if (id == 0x3B6 && len == 6 && emulateVIN) { // ASCII coded 4-9 letters of VIN
            canMsgSnd.buf[0] = vinNumber[3]; //X
            canMsgSnd.buf[1] = vinNumber[4]; //X
            canMsgSnd.buf[2] = vinNumber[5]; //X
            canMsgSnd.buf[3] = vinNumber[6]; //X
            canMsgSnd.buf[4] = vinNumber[7]; //X
            canMsgSnd.buf[5] = vinNumber[8]; //X
            canMsgSnd.id = 0x3B6;
            canMsgSnd.len = 6;
            can2010.write(canMsgSnd);
        } else if (id == 0x2B6 && len == 8 && emulateVIN) { // ASCII coded 10-17 letters (last 8) of VIN
            canMsgSnd.buf[0] = vinNumber[9]; //X
            canMsgSnd.buf[1] = vinNumber[10]; //X
            canMsgSnd.buf[2] = vinNumber[11]; //X
            canMsgSnd.buf[3] = vinNumber[12]; //X
            canMsgSnd.buf[4] = vinNumber[13]; //X
            canMsgSnd.buf[5] = vinNumber[14]; //X
            canMsgSnd.buf[6] = vinNumber[15]; //X
            canMsgSnd.buf[7] = vinNumber[16]; //X
            canMsgSnd.id = 0x2B6;
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
        } else if (id == 0xE6 && len < 8) { // ABS status frame, increase length
            canMsgSnd.buf[0] = msg.buf[0]; // Status lights / Alerts
            canMsgSnd.buf[1] = msg.buf[1]; // Rear left rotations
            canMsgSnd.buf[2] = msg.buf[2]; // Rear left rotations
            canMsgSnd.buf[3] = msg.buf[3]; // Rear right rotations
            canMsgSnd.buf[4] = msg.buf[4]; // Rear right rotations
            canMsgSnd.buf[5] = msg.buf[5]; // Battery Voltage measured by ABS
            canMsgSnd.buf[6] = msg.buf[6]; // STT / Slope / Emergency Braking
            canMsgSnd.buf[7] = 0x00; // Checksum / Counter : WIP
            canMsgSnd.id = 0xE6;
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
        } else if (id == 0x21F && len == 3) { // Steering wheel commands - Generic
            tmpVal = msg.buf[0];
            scrollValue = msg.buf[1];

            if (bitRead(msg.buf[0], 1) && noFMUX && steeringWheelCommands_Type ==
                                                    0) { // Replace MODE/SRC by MENU (Valid for 208, C-Elysee calibrations for example)
                canMsgSnd.buf[0] = 0x80; // MENU button
                canMsgSnd.buf[1] = 0x00;
                canMsgSnd.buf[2] = 0x00;
                canMsgSnd.buf[3] = 0x00;
                canMsgSnd.buf[4] = 0x00;
                canMsgSnd.buf[5] = 0x02;
                canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                canMsgSnd.buf[7] = 0x00;
                canMsgSnd.id = 0x122;
                canMsgSnd.len = 8;
                can2010.write(canMsgSnd);
                if (Send_CAN2010_ForgedMessages) {
                    can2004.write(canMsgSnd);
                }
            } else {
                can2010.write(msg);

                if (noFMUX || hasAnalogicButtons) { // Fake FMUX Buttons in the car
                    canMsgSnd.buf[0] = 0x00;
                    canMsgSnd.buf[1] = 0x00;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    canMsgSnd.id = 0x122;
                    canMsgSnd.len = 8;
                    can2010.write(canMsgSnd);
                    if (Send_CAN2010_ForgedMessages) {
                        can2004.write(canMsgSnd);
                    }
                }
            }
        } else if (id == 0xA2 && noFMUX && steeringWheelCommands_Type == 1) { // Steering wheel commands - C4 I / C5 X7
            // Fake FMUX Buttons in the car
            canMsgSnd.buf[0] = 0x00;
            canMsgSnd.buf[1] = 0x00;
            canMsgSnd.buf[2] = 0x00;
            canMsgSnd.buf[3] = 0x00;
            canMsgSnd.buf[4] = 0x00;
            canMsgSnd.buf[5] = 0x02;
            canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
            canMsgSnd.buf[7] = 0x00;

            if (bitRead(msg.buf[1], 3)) { // MENU button pushed > MUSIC
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x00;
                    canMsgSnd.buf[1] = 0x20;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 2)) { // MODE button pushed > NAV
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x00;
                    canMsgSnd.buf[1] = 0x08;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 4)) { // ESC button pushed > APPS
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x00;
                    canMsgSnd.buf[1] = 0x40;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 5)) { // OK button pushed > PHONE
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x00;
                    canMsgSnd.buf[1] = 0x04;
                    canMsgSnd.buf[2] = 0x08;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else {
                pushA2 = false;
                can2010.write(msg);
            }
            canMsgSnd.id = 0x122;
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }
        } else if (id == 0xA2 && noFMUX && (steeringWheelCommands_Type == 2 || steeringWheelCommands_Type == 3 ||
                                            steeringWheelCommands_Type == 4 || steeringWheelCommands_Type ==
                                                                               5)) { // Steering wheel commands - C4 I / C5 X7
            // Fake FMUX Buttons in the car
            canMsgSnd.buf[0] = 0x00;
            canMsgSnd.buf[1] = 0x00;
            canMsgSnd.buf[2] = 0x00;
            canMsgSnd.buf[3] = 0x00;
            canMsgSnd.buf[4] = 0x00;
            canMsgSnd.buf[5] = 0x02;
            canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
            canMsgSnd.buf[7] = 0x00;

            if (bitRead(msg.buf[1], 3)) { // MENU button pushed > MENU
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x80;
                    canMsgSnd.buf[1] = 0x00;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 2) && (steeringWheelCommands_Type == 3 || steeringWheelCommands_Type ==
                                                                                     5)) { // Right push button / MODE/SRC > SRC
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x40;
                    canMsgSnd.buf[1] = 0x00;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 4) && steeringWheelCommands_Type == 4) { // ESC button pushed > SRC
                if (!pushA2) {
                    canMsgSnd.buf[0] = 0x40;
                    canMsgSnd.buf[1] = 0x00;
                    canMsgSnd.buf[2] = 0x00;
                    canMsgSnd.buf[3] = 0x00;
                    canMsgSnd.buf[4] = 0x00;
                    canMsgSnd.buf[5] = 0x02;
                    canMsgSnd.buf[6] = 0x00; // Volume potentiometer button
                    canMsgSnd.buf[7] = 0x00;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 4) && steeringWheelCommands_Type == 5) { // ESC button pushed > TRIP
                if (!pushA2) {
                    pushTRIP = true;
                    pushA2 = true;
                }
            } else if (bitRead(msg.buf[1], 2) &&
                       steeringWheelCommands_Type == 4) { // Right push button / MODE/SRC > TRIP
                if (!pushA2) {
                    pushTRIP = true;
                    pushA2 = true;
                }
            } else {
                pushA2 = false;
                can2010.write(msg);
            }
            canMsgSnd.id = 0x122;
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }

            if (pushTRIP) {
                pushTRIP = false;

                canMsgSnd.buf[0] = statusTRIP[0];
                bitWrite(canMsgSnd.buf[0], 3, 1);
                canMsgSnd.buf[1] = statusTRIP[1];
                canMsgSnd.buf[2] = statusTRIP[2];
                canMsgSnd.buf[3] = statusTRIP[3];
                canMsgSnd.buf[4] = statusTRIP[4];
                canMsgSnd.buf[5] = statusTRIP[5];
                canMsgSnd.buf[6] = statusTRIP[6];
                canMsgSnd.buf[7] = statusTRIP[7];
                canMsgSnd.id = 0x221;
                canMsgSnd.len = 8;
                can2010.write(canMsgSnd);
                if (Send_CAN2010_ForgedMessages) {
                    can2004.write(canMsgSnd);
                }
            }
        } else if (id == 0x217 && len == 8) { // Cache cluster status (CMB)
            statusCMB[0] = msg.buf[0];
            statusCMB[1] = msg.buf[1];
            statusCMB[2] = msg.buf[2];
            statusCMB[3] = msg.buf[3];
            statusCMB[4] = msg.buf[4];
            statusCMB[5] = msg.buf[5];
            statusCMB[6] = msg.buf[6];
            statusCMB[7] = msg.buf[7];

            can2010.write(msg);
        } else if (id == 0x1D0 && len == 7 && EngineRunning) { // No fan activated if the engine is not ON on old models
            LeftTemp = msg.buf[5];
            RightTemp = msg.buf[6];
            if (LeftTemp == RightTemp) { // No other way to detect MONO mode
                Mono = true;
                LeftTemp = LeftTemp + 64;
            } else {
                Mono = false;
            }

            FanOff = false;
            // Fan Speed BSI_2010 = "41" (Off) > "49" (Full speed)
            tmpVal = msg.buf[2];
            if (tmpVal == 15) {
                FanOff = true;
                FanSpeed = 0x41;
            } else {
                FanSpeed = (tmpVal + 66);
            }

            // Position Fan
            tmpVal = msg.buf[3];

            if (tmpVal == 0x40) {
                FootAerator = false;
                WindShieldAerator = true;
                CentralAerator = false;
            } else if (tmpVal == 0x30) {
                FootAerator = false;
                WindShieldAerator = false;
                CentralAerator = true;
            } else if (tmpVal == 0x20) {
                FootAerator = true;
                WindShieldAerator = false;
                CentralAerator = false;
            } else if (tmpVal == 0x70) {
                FootAerator = false;
                WindShieldAerator = true;
                CentralAerator = true;
            } else if (tmpVal == 0x80) {
                FootAerator = true;
                WindShieldAerator = true;
                CentralAerator = true;
            } else if (tmpVal == 0x50) {
                FootAerator = true;
                WindShieldAerator = false;
                CentralAerator = true;
            } else if (tmpVal == 0x10) {
                FootAerator = false;
                WindShieldAerator = false;
                CentralAerator = false;
            } else if (tmpVal == 0x60) {
                FootAerator = true;
                WindShieldAerator = true;
                CentralAerator = false;
            } else {
                FootAerator = false;
                WindShieldAerator = false;
                CentralAerator = false;
            }

            tmpVal = msg.buf[4];
            if (tmpVal == 0x10) {
                DeMist = true;
                AirRecycle = false;
            } else if (tmpVal == 0x30) {
                AirRecycle = true;
            } else {
                AirRecycle = false;
            }

            AutoFan = false;
            DeMist = false;

            tmpVal = msg.buf[0];
            if (tmpVal == 0x11) {
                DeMist = true;
                AirConditioningON = true;
                FanOff = false;
            } else if (tmpVal == 0x12) {
                DeMist = true;
                AirConditioningON = false;
                FanOff = false;
            } else if (tmpVal == 0x21) {
                DeMist = true;
                AirConditioningON = true;
                FanOff = false;
            } else if (tmpVal == 0xA2) {
                FanOff = true;
                AirConditioningON = false;
            } else if (tmpVal == 0x22) {
                AirConditioningON = false;
            } else if (tmpVal == 0x20) {
                AirConditioningON = true;
            } else if (tmpVal == 0x02) {
                AirConditioningON = false;
                AutoFan = false;
            } else if (tmpVal == 0x00) {
                AirConditioningON = true;
                AutoFan = true;
            }

            if (!FootAerator && !WindShieldAerator && CentralAerator) {
                FanPosition = 0x34;
            } else if (FootAerator && WindShieldAerator && CentralAerator) {
                FanPosition = 0x84;
            } else if (!FootAerator && WindShieldAerator && CentralAerator) {
                FanPosition = 0x74;
            } else if (FootAerator && !WindShieldAerator && CentralAerator) {
                FanPosition = 0x54;
            } else if (FootAerator && !WindShieldAerator && !CentralAerator) {
                FanPosition = 0x24;
            } else if (!FootAerator && WindShieldAerator && !CentralAerator) {
                FanPosition = 0x44;
            } else if (FootAerator && WindShieldAerator && !CentralAerator) {
                FanPosition = 0x64;
            } else {
                FanPosition = 0x04; // Nothing
            }

            if (DeMist) {
                FanSpeed = 0x10;
                FanPosition = FanPosition + 16;
            } else if (AutoFan) {
                FanSpeed = 0x10;
            }

            if (FanOff) {
                AirConditioningON = false;
                FanSpeed = 0x41;
                LeftTemp = 0x00;
                RightTemp = 0x00;
                FanPosition = 0x04;
            }

            if (AirConditioningON) {
                canMsgSnd.buf[0] = 0x01; // A/C ON - Auto Soft : "00" / Auto Normal "01" / Auto Fast "02"
            } else {
                canMsgSnd.buf[0] = 0x09; // A/C OFF - Auto Soft : "08" / Auto Normal "09" / Auto Fast "0A"
            }

            canMsgSnd.buf[1] = 0x00;
            canMsgSnd.buf[2] = 0x00;
            canMsgSnd.buf[3] = LeftTemp;
            canMsgSnd.buf[4] = RightTemp;
            canMsgSnd.buf[5] = FanSpeed;
            canMsgSnd.buf[6] = FanPosition;
            canMsgSnd.buf[7] = 0x00;
            canMsgSnd.id = 0x350;
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }
        } else if (id == 0xF6 && len == 8) {
            tmpVal = msg.buf[0];
            if (tmpVal > 128) {
                if (!Ignition && SerialEnabled) {
                    Serial.println("Ignition ON");
                }

                Ignition = true;
            } else {
                if (Ignition && SerialEnabled) {
                    Serial.println("Ignition OFF");
                }

                Ignition = false;
            }

            tmpVal = ceil(msg.buf[5] / 2.0) -
                     40; // Temperatures can be negative but we only have 0 > 255, the new range is starting from -40°C
            if (Temperature != tmpVal) {
                Temperature = tmpVal;

                if (SerialEnabled) {
                    Serial.print("Ext. Temperature: ");
                    Serial.print(tmpVal);
                    Serial.println("°C");
                }
            }

            can2010.write(msg);
        } else if (id == 0x168 && len == 8) { // Instrument Panel - WIP
            canMsgSnd.buf[0] = msg.buf[0]; // Alerts
            canMsgSnd.buf[1] = msg.buf[1];
            canMsgSnd.buf[2] = msg.buf[2];
            canMsgSnd.buf[3] = msg.buf[3];
            canMsgSnd.buf[4] = msg.buf[4];
            canMsgSnd.buf[5] = msg.buf[5];
            bitWrite(canMsgSnd.buf[6], 7, 0);
            bitWrite(canMsgSnd.buf[6], 6, 1); // Ambiance
            bitWrite(canMsgSnd.buf[6], 5, 1); // EMF availability
            bitWrite(canMsgSnd.buf[6], 4, bitRead(msg.buf[5], 0)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[6], 3, bitRead(msg.buf[6], 7)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[6], 2, bitRead(msg.buf[6], 6)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[6], 1, bitRead(msg.buf[6], 5)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[6], 0, 0);
            canMsgSnd.buf[7] = msg.buf[7];
            canMsgSnd.id = 0x168;
            canMsgSnd.len = 8;

            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) { // Will generate some light issues on the instrument panel
                can2004.write(canMsgSnd);
            }
        } else if (id == 0x120 &&
                   generatePOPups) { // Alerts journal / Diagnostic > Popup notifications - Work in progress
            // C5 (X7) Cluster is connected to CAN High Speed, no notifications are sent on CAN Low Speed, let's rebuild alerts from the journal (slighly slower than original alerts)
            // Bloc 1
            if (bitRead(msg.buf[0], 7) == 0 && bitRead(msg.buf[0], 6) == 1) {
                sendPOPup(bitRead(msg.buf[1], 7), 5, 1, 0x00); // Engine oil pressure fault: stop the vehicle (STOP)
                sendPOPup(bitRead(msg.buf[1], 6), 1, 1, 0x00); // Engine temperature fault: stop the vehicle (STOP)
                sendPOPup(bitRead(msg.buf[1], 5), 138, 6, 0x00); // Charging system fault: repair needed (WARNING)
                sendPOPup(bitRead(msg.buf[1], 4), 106, 1, 0x00); // Braking system fault: stop the vehicle (STOP)
                // bitRead(msg.buf[1], 3); // N/A
                sendPOPup(bitRead(msg.buf[1], 2), 109, 2, 0x00); // Power steering fault: stop the vehicle (STOP)
                sendPOPup(bitRead(msg.buf[1], 1), 3, 4, 0x00); // Top up coolant level (WARNING)
                // bitRead(msg.buf[1], 0); // Fault with LKA (WARNING)
                sendPOPup(bitRead(msg.buf[2], 7), 4, 4, 0x00); // Top up engine oil level (WARNING)
                // bitRead(msg.buf[2], 6); // N/A
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[2], 5)); // Front right door
                bitWrite(notificationParameters, 6, bitRead(msg.buf[2], 4)); // Front left door
                bitWrite(notificationParameters, 5, bitRead(msg.buf[2], 3)); // Rear right door
                bitWrite(notificationParameters, 4, bitRead(msg.buf[2], 2)); // Rear left door
                bitWrite(notificationParameters, 3, bitRead(msg.buf[2], 0)); // Boot open
                // bitWrite(notificationParameters, 2, ?); // Hood open
                bitWrite(notificationParameters, 1, bitRead(msg.buf[3], 7)); // Rear Screen open
                // bitWrite(notificationParameters, 0, ?); // Fuel door open
                sendPOPup((bitRead(msg.buf[2], 5) || bitRead(msg.buf[2], 4) || bitRead(msg.buf[2], 3) ||
                           bitRead(msg.buf[2], 2) || bitRead(msg.buf[2], 0) || bitRead(msg.buf[3], 7)), 8, 8,
                          notificationParameters); // Left hand front door opened (WARNING) || Right hand front door opened (WARNING) || Left hand rear door opened (WARNING) || Right hand rear door opened (WARNING) || Boot open (WARNING) || Rear screen open (WARNING)
                // bitRead(msg.buf[2], 1); // N/A
                sendPOPup(bitRead(msg.buf[3], 6), 107, 2, 0x00); // ESP/ASR system fault, repair the vehicle (WARNING)
                // bitRead(msg.buf[3], 5); // Battery charge fault, stop the vehicle (WARNING)
                // bitRead(msg.buf[3], 4); // N/A
                sendPOPup(bitRead(msg.buf[3], 3), 125, 6, 0x00); // Water in diesel fuel filter (WARNING)
                sendPOPup(bitRead(msg.buf[3], 2), 103, 6, 0x00); // Have brake pads replaced (WARNING)
                sendPOPup(bitRead(msg.buf[3], 1), 224, 10, 0x00); // Fuel level low (INFO)
                sendPOPup(bitRead(msg.buf[3], 0), 120, 6,
                          0x00); // Airbag(s) or seatbelt(s) pretensioner fault(s) (WARNING)
                // bitRead(msg.buf[4], 7); // N/A
                // bitRead(msg.buf[4], 6); // Engine fault, repair the vehicle (WARNING)
                sendPOPup(bitRead(msg.buf[4], 5), 106, 2,
                          0x00); // ABS braking system fault, repair the vehicle (WARNING)
                sendPOPup(bitRead(msg.buf[4], 4), 15, 4,
                          0x00); // Particle filter is full, please drive 20min to clean it (WARNING)
                // bitRead(msg.buf[4], 3); // N/A
                sendPOPup(bitRead(msg.buf[4], 2), 129, 6, 0x00); // Particle filter additive level low (WARNING)
                // bitRead(msg.buf[4], 1); // N/A
                sendPOPup(bitRead(msg.buf[4], 0), 17, 4, 0x00); // Suspension fault, repair the vehicle (WARNING)
                // bitRead(msg.buf[5], 7); // Preheating deactivated, battery charge too low (INFO)
                // bitRead(msg.buf[5], 6); // Preheating deactivated, fuel level too low (INFO)
                // bitRead(msg.buf[5], 5); // Check the centre brake lamp (WARNING)
                // bitRead(msg.buf[5], 4); // Retractable roof mechanism fault (WARNING)
                // sendPOPup(bitRead(msg.buf[5], 3), ?, 8, 0x00); // Steering lock fault, repair the vehicle (WARNING)
                sendPOPup(bitRead(msg.buf[5], 2), 131, 6, 0x00); // Electronic immobiliser fault (WARNING)
                // bitRead(msg.buf[5], 1); // N/A
                // bitRead(msg.buf[5], 0); // Roof operation not possible, system temperature too high (WARNING)
                // bitRead(msg.buf[6], 7); // Roof operation not possible, start the engine (WARNING)
                // bitRead(msg.buf[6], 6); // Roof operation not possible, apply parking brake (WARNING)
                // bitRead(msg.buf[6], 5); // Hybrid system fault (STOP)
                // bitRead(msg.buf[6], 4); // Automatic headlamp adjustment fault (WARNING)
                // bitRead(msg.buf[6], 3); // Hybrid system fault (WARNING)
                // bitRead(msg.buf[6], 2); // Hybrid system fault: speed restricted (WARNING)
                sendPOPup(bitRead(msg.buf[6], 1), 223, 10, 0x00); // Top Up screenwash fluid level (INFO)
                sendPOPup(bitRead(msg.buf[6], 0), 227, 14, 0x00); // Replace remote control battery (INFO)
                // bitRead(msg.buf[7], 7); // N/A
                // bitRead(msg.buf[7], 6); // Preheating deactivated, set the clock (INFO)
                // bitRead(msg.buf[7], 5); // Trailer connection fault (WARNING)
                // bitRead(msg.buf[7], 4); // N/A
                // bitRead(msg.buf[7], 3); // Tyre under-inflation (WARNING)
                // bitRead(msg.buf[7], 2); // Driving aid camera limited visibility (INFO)
                // bitRead(msg.buf[7], 1); // N/A
                // bitRead(msg.buf[7], 0); // N/A
            }

            // Bloc 2
            if (bitRead(msg.buf[0], 7) == 1 && bitRead(msg.buf[0], 6) == 0) {
                // bitRead(msg.buf[1], 7); // N/A
                // bitRead(msg.buf[1], 6); // Electric mode not available : Particle filter regenerating (INFO)
                // bitRead(msg.buf[1], 5); // N/A
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[1], 4)); // Front left tyre
                bitWrite(notificationParameters, 6, bitRead(msg.buf[1], 3)); // Front right tyre
                bitWrite(notificationParameters, 5, bitRead(msg.buf[1], 2)); // Rear right tyre
                bitWrite(notificationParameters, 4, bitRead(msg.buf[1], 1)); // Rear left tyre
                sendPOPup((bitRead(msg.buf[1], 4) || bitRead(msg.buf[1], 3) || bitRead(msg.buf[1], 2) ||
                           bitRead(msg.buf[1], 1)), 13, 6,
                          notificationParameters); // Puncture: Replace or repair the wheel (STOP)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[1], 0)); // Front right sidelamp
                bitWrite(notificationParameters, 6, bitRead(msg.buf[2], 7)); // Front left sidelamp
                bitWrite(notificationParameters, 5, bitRead(msg.buf[2], 6)); // Rear right sidelamp
                bitWrite(notificationParameters, 4, bitRead(msg.buf[2], 5)); // Rear left sidelamp
                sendPOPup((bitRead(msg.buf[1], 0) || bitRead(msg.buf[2], 7) || bitRead(msg.buf[2], 6) ||
                           bitRead(msg.buf[2], 5)), 160, 6, notificationParameters); // Check sidelamps (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[2], 4)); // Right dipped beam headlamp
                bitWrite(notificationParameters, 6, bitRead(msg.buf[2], 3)); // Left dipped beam headlamp
                sendPOPup((bitRead(msg.buf[2], 4) || bitRead(msg.buf[2], 3)), 154, 6,
                          notificationParameters); // Check the dipped beam headlamps (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[2], 2)); // Right main beam headlamp
                bitWrite(notificationParameters, 6, bitRead(msg.buf[2], 1)); // Left main beam headlamp
                sendPOPup((bitRead(msg.buf[2], 2) || bitRead(msg.buf[2], 1)), 155, 6,
                          notificationParameters); // Check the main beam headlamps (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[2], 0)); // Right brake lamp
                bitWrite(notificationParameters, 6, bitRead(msg.buf[3], 7)); // Left brake lamp
                sendPOPup((bitRead(msg.buf[2], 0) || bitRead(msg.buf[3], 7)), 156, 6,
                          notificationParameters); // Check the RH brake lamp (WARNING) || Check the LH brake lamp (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[3], 6)); // Front right foglamp
                bitWrite(notificationParameters, 6, bitRead(msg.buf[3], 5)); // Front left foglamp
                bitWrite(notificationParameters, 5, bitRead(msg.buf[3], 4)); // Rear right foglamp
                bitWrite(notificationParameters, 4, bitRead(msg.buf[3], 3)); // Rear left foglamp
                sendPOPup((bitRead(msg.buf[3], 6) || bitRead(msg.buf[3], 5) || bitRead(msg.buf[3], 4) ||
                           bitRead(msg.buf[3], 3)), 157, 6,
                          notificationParameters); // Check the front foglamps (WARNING) || Check the front foglamps (WARNING) || Check the rear foglamps (WARNING) || Check the rear foglamps (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[3], 2)); // Front right direction indicator
                bitWrite(notificationParameters, 6, bitRead(msg.buf[3], 1)); // Front left direction indicator
                bitWrite(notificationParameters, 5, bitRead(msg.buf[3], 0)); // Rear right direction indicator
                bitWrite(notificationParameters, 4, bitRead(msg.buf[4], 7)); // Rear left direction indicator
                sendPOPup((bitRead(msg.buf[3], 2) || bitRead(msg.buf[3], 1) || bitRead(msg.buf[3], 0) ||
                           bitRead(msg.buf[4], 7)), 159, 6,
                          notificationParameters); // Check the direction indicators (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[4], 6)); // Right reversing lamp
                bitWrite(notificationParameters, 6, bitRead(msg.buf[4], 5)); // Left reversing lamp
                sendPOPup((bitRead(msg.buf[4], 6) || bitRead(msg.buf[4], 5)), 159, 6,
                          notificationParameters); // Check the reversing lamp(s) (WARNING)
                // bitRead(msg.buf[4], 4); // N/A
                // bitRead(msg.buf[4], 3); // N/A
                // bitRead(msg.buf[4], 2); // N/A
                // bitRead(msg.buf[4], 1); // N/A
                // bitRead(msg.buf[4], 0); // N/A
                // bitRead(msg.buf[5], 7); // N/A
                // bitRead(msg.buf[5], 6); // N/A
                // bitRead(msg.buf[5], 5); // N/A
                sendPOPup(bitRead(msg.buf[5], 4), 136, 8, 0x00); // Parking assistance system fault (WARNING)
                // bitRead(msg.buf[5], 3); // N/A
                // bitRead(msg.buf[5], 2); // N/A
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[5], 1)); // Front left tyre
                bitWrite(notificationParameters, 6, bitRead(msg.buf[5], 0)); // Front right tyre
                bitWrite(notificationParameters, 5, bitRead(msg.buf[6], 7)); // Rear right tyre
                bitWrite(notificationParameters, 4, bitRead(msg.buf[6], 5)); // Rear left tyre
                sendPOPup((bitRead(msg.buf[5], 1) || bitRead(msg.buf[5], 0) || bitRead(msg.buf[6], 7) ||
                           bitRead(msg.buf[6], 5)), 13, 8, notificationParameters); // Adjust tyre pressures (WARNING)
                // bitRead(msg.buf[6], 5); // Switch off lighting (INFO)
                // bitRead(msg.buf[6], 4); // N/A
                sendPOPup((bitRead(msg.buf[6], 3) || bitRead(msg.buf[6], 1)), 190, 8,
                          0x00); // Emissions fault (WARNING)
                sendPOPup(bitRead(msg.buf[6], 2), 192, 8, 0x00); // Emissions fault: Starting Prevented (WARNING)
                // bitRead(msg.buf[6], 0); // N/A
                // bitRead(msg.buf[7], 7); // N/A
                // bitRead(msg.buf[7], 6); // N/A
                sendPOPup(bitRead(msg.buf[7], 5), 215, 10, 0x00); // "P" (INFO)
                sendPOPup(bitRead(msg.buf[7], 4), 216, 10, 0x00); // Ice warning (INFO)
                bitWrite(statusOpenings, 7, bitRead(msg.buf[7], 3)); // Front right door
                bitWrite(statusOpenings, 6, bitRead(msg.buf[7], 2)); // Front left door
                bitWrite(statusOpenings, 5, bitRead(msg.buf[7], 1)); // Rear right door
                bitWrite(statusOpenings, 4, bitRead(msg.buf[7], 0)); // Rear left door
                sendPOPup((bitRead(msg.buf[7], 3) || bitRead(msg.buf[7], 2) || bitRead(msg.buf[7], 1) ||
                           bitRead(msg.buf[7], 0) || bitRead(statusOpenings, 3) || bitRead(statusOpenings, 1)), 222, 8,
                          statusOpenings); // Front right door opened (INFO) || Front left door opened (INFO) || Rear right door opened (INFO) || Rear left door opened (INFO)
            }

            // Bloc 3
            if (bitRead(msg.buf[0], 7) == 1 && bitRead(msg.buf[0], 6) == 1) {
                bitWrite(statusOpenings, 3, bitRead(msg.buf[1], 7)); // Boot open
                // bitWrite(statusOpenings, 2, ?); // Hood open
                bitWrite(statusOpenings, 1, bitRead(msg.buf[1], 5)); // Rear Screen open
                // bitWrite(statusOpenings, 0, ?); // Fuel door open
                sendPOPup((bitRead(msg.buf[1], 7) || bitRead(msg.buf[1], 5) || bitRead(statusOpenings, 7) ||
                           bitRead(statusOpenings, 6) || bitRead(statusOpenings, 5) || bitRead(statusOpenings, 4)), 222,
                          8, statusOpenings); // Boot open (INFO) || Rear Screen open (INFO)
                // bitRead(msg.buf[1], 6); // Collision detection risk system fault (INFO)
                // bitRead(msg.buf[1], 4); // N/A
                // bitRead(msg.buf[1], 3); // N/A
                // bitRead(msg.buf[1], 2); // N/A
                // bitRead(msg.buf[1], 1); // N/A
                // bitRead(msg.buf[1], 0); // N/A
                // bitRead(msg.buf[2], 7); // N/A
                // bitRead(msg.buf[2], 6); // N/A
                // bitRead(msg.buf[2], 5); // N/A
                sendPOPup(bitRead(msg.buf[2], 4), 100, 6, 0x00); // Parking brake fault (WARNING)
                // bitRead(msg.buf[2], 3); // Active spoiler fault: speed restricted (WARNING)
                // bitRead(msg.buf[2], 2); // Automatic braking system fault (INFO)
                // bitRead(msg.buf[2], 1); // Directional headlamps fault (WARNING)
                // bitRead(msg.buf[2], 0); // N/A
                // bitRead(msg.buf[3], 7); // N/A
                // bitRead(msg.buf[3], 6); // N/A
                // bitRead(msg.buf[3], 5); // N/A
                // bitRead(msg.buf[3], 4); // N/A
                // bitRead(msg.buf[3], 3); // N/A
                if (isBVMP) {
                    sendPOPup(bitRead(msg.buf[3], 2), 122, 4, 0x00); // Gearbox fault (WARNING)
                } else {
                    sendPOPup(bitRead(msg.buf[3], 2), 110, 4, 0x00); // Gearbox fault (WARNING)
                }
                // bitRead(msg.buf[3], 1); // N/A
                // bitRead(msg.buf[3], 0); // N/A
                // bitRead(msg.buf[4], 7); // N/A
                // bitRead(msg.buf[4], 6); // N/A
                // bitRead(msg.buf[4], 5); // N/A
                // bitRead(msg.buf[4], 4); // N/A
                // bitRead(msg.buf[4], 3); // N/A
                // bitRead(msg.buf[4], 2); // Engine fault (WARNING)
                sendPOPup(bitRead(msg.buf[4], 1), 17, 3,
                          0x00); // Suspension fault: limit your speed to 90km/h (WARNING)
                // bitRead(msg.buf[4], 0); // N/A
                // bitRead(msg.buf[5], 7); // N/A
                // bitRead(msg.buf[5], 6); // N/A
                // bitRead(msg.buf[5], 5); // N/A
                // bitRead(msg.buf[5], 4); // N/A
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[5], 3)); // Front left tyre
                bitWrite(notificationParameters, 6, bitRead(msg.buf[5], 2)); // Front right tyre
                bitWrite(notificationParameters, 5, bitRead(msg.buf[5], 1)); // Rear right tyre
                bitWrite(notificationParameters, 4, bitRead(msg.buf[5], 0)); // Rear left tyre
                sendPOPup((bitRead(msg.buf[5], 3) || bitRead(msg.buf[5], 2) || bitRead(msg.buf[5], 1) ||
                           bitRead(msg.buf[5], 0)), 229, 10,
                          notificationParameters); // Sensor fault: Left hand front tyre pressure not monitored (INFO)
                sendPOPup(bitRead(msg.buf[6], 7), 18, 4, 0x00); // Suspension fault: repair the vehicle (WARNING)
                sendPOPup(bitRead(msg.buf[6], 6), 109, 4, 0x00); // Power steering fault: repair the vehicle (WARNING)
                // bitRead(msg.buf[6], 5); // N/A
                // bitRead(msg.buf[6], 4); // N/A
                // bitRead(msg.buf[6], 3); // Inter-vehicle time measurement fault (WARNING)
                // bitRead(msg.buf[6], 2); // Engine fault, stop the vehicle (STOP)
                // bitRead(msg.buf[6], 1); // Fault with LKA (INFO)
                // bitRead(msg.buf[6], 0); // Tyre under-inflation detection system fault (WARNING)
                notificationParameters = 0x00;
                bitWrite(notificationParameters, 7, bitRead(msg.buf[7], 7)); // Front left tyre
                bitWrite(notificationParameters, 6, bitRead(msg.buf[7], 6)); // Front right tyre
                bitWrite(notificationParameters, 5, bitRead(msg.buf[7], 5)); // Rear right tyre
                //bitWrite(notificationParameters, 4, ?); // Rear left tyre
                sendPOPup((bitRead(msg.buf[7], 7) || bitRead(msg.buf[7], 6) || bitRead(msg.buf[7], 5)), 183, 8,
                          notificationParameters); // Underinflated wheel, ajust pressure and reset (INFO)
                // bitRead(msg.buf[7], 4); // Spare wheel fitted: driving aids deactivated (INFO)
                // bitRead(msg.buf[7], 3); // Automatic braking disabled (INFO)
                sendPOPup(bitRead(msg.buf[7], 2), 188, 6, 0x00); // Refill AdBlue (WARNING)
                sendPOPup(bitRead(msg.buf[7], 1), 187, 10, 0x00); // Refill AdBlue (INFO)
                sendPOPup(bitRead(msg.buf[7], 0), 189, 4, 0x00); // Impossible engine start, refill AdBlue (WARNING)
            }

            can2010.write(msg); // Forward original frame
        } else if (id == 0x221) { // Trip info
            statusTRIP[0] = msg.buf[0];
            statusTRIP[1] = msg.buf[1];
            statusTRIP[2] = msg.buf[2];
            statusTRIP[3] = msg.buf[3];
            statusTRIP[4] = msg.buf[4];
            statusTRIP[5] = msg.buf[5];
            statusTRIP[6] = msg.buf[6];
            statusTRIP[7] = msg.buf[7];
            can2010.write(msg); // Forward original frame

            customTimeStamp = (long) hour() * (long) 3600 + minute() * 60 + second();
            daysSinceYearStart = daysSinceYearStartFct();

            canMsgSnd.buf[0] = (((1 << 8) - 1) & (customTimeStamp >> (12)));
            canMsgSnd.buf[1] = (((1 << 8) - 1) & (customTimeStamp >> (4)));
            canMsgSnd.buf[2] =
                    (((((1 << 4) - 1) & (customTimeStamp)) << 4)) + (((1 << 4) - 1) & (daysSinceYearStart >> (8)));
            canMsgSnd.buf[3] = (((1 << 8) - 1) & (daysSinceYearStart));
            canMsgSnd.buf[4] = 0x00;
            canMsgSnd.buf[5] = 0xC0;
            canMsgSnd.buf[6] = languageID;
            canMsgSnd.id = 0x3F6; // Fake EMF Time frame
            canMsgSnd.len = 7;

            can2004.write(canMsgSnd);
        } else if (id == 0x128 && len == 8) { // Instrument Panel
            canMsgSnd.buf[0] = msg.buf[4]; // Main driving lights
            bitWrite(canMsgSnd.buf[1], 7, bitRead(msg.buf[6], 7)); // Gearbox report
            bitWrite(canMsgSnd.buf[1], 6, bitRead(msg.buf[6], 6)); // Gearbox report
            bitWrite(canMsgSnd.buf[1], 5, bitRead(msg.buf[6], 5)); // Gearbox report
            bitWrite(canMsgSnd.buf[1], 4, bitRead(msg.buf[6], 4)); // Gearbox report
            bitWrite(canMsgSnd.buf[1], 3, bitRead(msg.buf[6], 3)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[1], 2, bitRead(msg.buf[6], 2)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[1], 1, bitRead(msg.buf[6], 1)); // Gearbox report while driving
            bitWrite(canMsgSnd.buf[1], 0, bitRead(msg.buf[6], 0)); // Gearbox report blinking
            bitWrite(canMsgSnd.buf[2], 7, bitRead(msg.buf[7], 7)); // Arrow blinking
            bitWrite(canMsgSnd.buf[2], 6, bitRead(msg.buf[7], 6)); // BVA mode
            bitWrite(canMsgSnd.buf[2], 5, bitRead(msg.buf[7], 5)); // BVA mode
            bitWrite(canMsgSnd.buf[2], 4, bitRead(msg.buf[7], 4)); // BVA mode
            bitWrite(canMsgSnd.buf[2], 3, bitRead(msg.buf[7], 3)); // Arrow type
            bitWrite(canMsgSnd.buf[2], 2, bitRead(msg.buf[7], 2)); // Arrow type
            if (bitRead(msg.buf[7], 1) == 1 && bitRead(msg.buf[7], 0) == 0) { // BVMP to BVA
                isBVMP = true;
                bitWrite(canMsgSnd.buf[2], 1, 0); // Gearbox type
                bitWrite(canMsgSnd.buf[2], 0, 0); // Gearbox type
            } else {
                bitWrite(canMsgSnd.buf[2], 1, bitRead(msg.buf[7], 1)); // Gearbox type
                bitWrite(canMsgSnd.buf[2], 0, bitRead(msg.buf[7], 0)); // Gearbox type
            }
            bitWrite(canMsgSnd.buf[3], 7, bitRead(msg.buf[1], 7)); // Service
            bitWrite(canMsgSnd.buf[3], 6, bitRead(msg.buf[1], 6)); // STOP
            bitWrite(canMsgSnd.buf[3], 5, bitRead(msg.buf[2], 5)); // Child security
            bitWrite(canMsgSnd.buf[3], 4, bitRead(msg.buf[0], 7)); // Passenger Airbag
            bitWrite(canMsgSnd.buf[3], 3, bitRead(msg.buf[3], 2)); // Foot on brake
            bitWrite(canMsgSnd.buf[3], 2, bitRead(msg.buf[3], 1)); // Foot on brake
            bitWrite(canMsgSnd.buf[3], 1, bitRead(msg.buf[0], 5)); // Parking brake
            bitWrite(canMsgSnd.buf[3], 0, 0); // Electric parking brake
            bitWrite(canMsgSnd.buf[4], 7, bitRead(msg.buf[0], 2)); // Diesel pre-heating
            bitWrite(canMsgSnd.buf[4], 6, bitRead(msg.buf[1], 4)); // Opening open
            bitWrite(canMsgSnd.buf[4], 5, bitRead(msg.buf[3], 4)); // Automatic parking
            bitWrite(canMsgSnd.buf[4], 4, bitRead(msg.buf[3], 3)); // Automatic parking blinking
            bitWrite(canMsgSnd.buf[4], 3, 0); // Automatic high beam
            bitWrite(canMsgSnd.buf[4], 2, bitRead(msg.buf[2], 4)); // ESP Disabled
            bitWrite(canMsgSnd.buf[4], 1, bitRead(msg.buf[2], 3)); // ESP active
            bitWrite(canMsgSnd.buf[4], 0, bitRead(msg.buf[2], 2)); // Active suspension
            bitWrite(canMsgSnd.buf[5], 7, bitRead(msg.buf[0], 4)); // Low fuel
            bitWrite(canMsgSnd.buf[5], 6, bitRead(msg.buf[0], 6)); // Driver seatbelt
            bitWrite(canMsgSnd.buf[5], 5, bitRead(msg.buf[3], 7)); // Driver seatbelt blinking
            bitWrite(canMsgSnd.buf[5], 4, bitRead(msg.buf[0], 1)); // Passenger seatbelt
            bitWrite(canMsgSnd.buf[5], 3, bitRead(msg.buf[3], 6)); // Passenger seatbelt Blinking
            bitWrite(canMsgSnd.buf[5], 2, 0); // SCR
            bitWrite(canMsgSnd.buf[5], 1, 0); // SCR
            bitWrite(canMsgSnd.buf[5], 0, bitRead(msg.buf[5], 6)); // Rear left seatbelt
            bitWrite(canMsgSnd.buf[6], 7, bitRead(msg.buf[5], 5)); // Rear seatbelt left blinking
            bitWrite(canMsgSnd.buf[6], 6, bitRead(msg.buf[5], 2)); // Rear right seatbelt
            bitWrite(canMsgSnd.buf[6], 5, bitRead(msg.buf[5], 1)); // Rear right seatbelt blinking
            bitWrite(canMsgSnd.buf[6], 4, bitRead(msg.buf[5], 4)); // Rear middle seatbelt
            bitWrite(canMsgSnd.buf[6], 3, bitRead(msg.buf[5], 3)); // Rear middle seatbelt blinking
            bitWrite(canMsgSnd.buf[6], 2, bitRead(msg.buf[5], 7)); // Instrument Panel ON
            bitWrite(canMsgSnd.buf[6], 1, bitRead(msg.buf[2], 1)); // Warnings
            bitWrite(canMsgSnd.buf[6], 0, 0); // Passenger protection
            canMsgSnd.buf[7] = 0x00;
            canMsgSnd.id = 0x128;
            canMsgSnd.len = 8;

            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) { // Will generate some light issues on the instrument panel
                can2004.write(canMsgSnd);
            }
        } else if (id == 0x3A7 && len == 8) { // Maintenance
            canMsgSnd.buf[0] = 0x40;
            canMsgSnd.buf[1] = msg.buf[5]; // Value x255 +
            canMsgSnd.buf[2] = msg.buf[6]; // Value x1 = Number of days till maintenance (FF FF if disabled)
            canMsgSnd.buf[3] = msg.buf[3]; // Value x5120 +
            canMsgSnd.buf[4] = msg.buf[4]; // Value x20 = km left till maintenance
            canMsgSnd.id = 0x3E7; // New maintenance frame ID
            canMsgSnd.len = 5;

            if (SerialEnabled && !MaintenanceDisplayed) {
                Serial.print("Next maintenance in: ");
                if (msg.buf[3] != 0xFF && msg.buf[4] != 0xFF) {
                    tmpVal = (msg.buf[3] * 5120) + (msg.buf[4] * 20);
                    Serial.print(tmpVal);
                    Serial.println(" km");
                }
                if (msg.buf[5] != 0xFF && msg.buf[6] != 0xFF) {
                    tmpVal = (msg.buf[5] * 255) + msg.buf[6];
                    Serial.print(tmpVal);
                    Serial.println(" days");
                }
                MaintenanceDisplayed = true;
            }

            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }
        } else if (id == 0x1A8 && len == 8) { // Cruise control
            can2010.write(msg);

            canMsgSnd.buf[0] = msg.buf[1];
            canMsgSnd.buf[1] = msg.buf[2];
            canMsgSnd.buf[2] = msg.buf[0];
            canMsgSnd.buf[3] = 0x80;
            canMsgSnd.buf[4] = 0x14;
            canMsgSnd.buf[5] = 0x7F;
            canMsgSnd.buf[6] = 0xFF;
            canMsgSnd.buf[7] = 0x98;
            canMsgSnd.id = 0x228; // New cruise control frame ID
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }
        } else if (id == 0x2D7 && len == 5 && listenCAN2004Language) { // CAN2004 Matrix
            tmpVal = msg.buf[0];
            if (tmpVal > 32) {
                kmL = true;
                tmpVal = tmpVal - 32;
            }

            if (tmpVal <= 32 && languageID_CAN2004 != tmpVal) {
                languageID_CAN2004 = tmpVal;
                EEPROM.update(1, languageID_CAN2004);

                // Change language and unit on ID 608 for CAN2010 Telematic language change
                languageAndUnitNum = (languageID_CAN2004 * 4) + 128;
                if (kmL) {
                    languageAndUnitNum = languageAndUnitNum + 1;
                }
                EEPROM.update(0, languageAndUnitNum);

                if (SerialEnabled) {
                    Serial.print("CAN2004 Matrix - Change Language: ");
                    Serial.print(tmpVal);
                    Serial.println();
                }
            } else {
                Serial.print("CAN2004 Matrix - Unsupported language ID: ");
                Serial.print(tmpVal);
                Serial.println();
            }
        } else if (id == 0x361) { // Personalization menus availability
            bitWrite(canMsgSnd.buf[0], 7, 1); // Parameters availability
            bitWrite(canMsgSnd.buf[0], 6, bitRead(msg.buf[2], 3)); // Beam
            bitWrite(canMsgSnd.buf[0], 5, 0); // Lighting
            bitWrite(canMsgSnd.buf[0], 4, bitRead(msg.buf[3], 7)); // Adaptative lighting
            bitWrite(canMsgSnd.buf[0], 3, bitRead(msg.buf[4], 1)); // SAM
            bitWrite(canMsgSnd.buf[0], 2, bitRead(msg.buf[4], 2)); // Ambiance lighting
            bitWrite(canMsgSnd.buf[0], 1, bitRead(msg.buf[2], 0)); // Automatic headlights
            bitWrite(canMsgSnd.buf[0], 0, bitRead(msg.buf[3], 6)); // Daytime running lights
            bitWrite(canMsgSnd.buf[1], 7, bitRead(msg.buf[5], 5)); // AAS
            bitWrite(canMsgSnd.buf[1], 6, bitRead(msg.buf[3], 5)); // Wiper in reverse
            bitWrite(canMsgSnd.buf[1], 5, bitRead(msg.buf[2], 4)); // Guide-me home lighting
            bitWrite(canMsgSnd.buf[1], 4, bitRead(msg.buf[1], 2)); // Driver welcome
            bitWrite(canMsgSnd.buf[1], 3, bitRead(msg.buf[2], 6)); // Motorized tailgate
            bitWrite(canMsgSnd.buf[1], 2, bitRead(msg.buf[2], 0)); // Selective openings - Rear
            bitWrite(canMsgSnd.buf[1], 1, bitRead(msg.buf[2], 7)); // Selective openings - Key
            bitWrite(canMsgSnd.buf[1], 0, 0); // Selective openings
            bitWrite(canMsgSnd.buf[2], 7, 1); // TNB - Seatbelt indicator
            bitWrite(canMsgSnd.buf[2], 6, 1); // XVV - Custom cruise limits
            bitWrite(canMsgSnd.buf[2], 5, bitRead(msg.buf[1], 4)); // Configurable button
            bitWrite(canMsgSnd.buf[2], 4, bitRead(msg.buf[2], 2)); // Automatic parking brake
            bitWrite(canMsgSnd.buf[2], 3, 0); // Sound Harmony
            bitWrite(canMsgSnd.buf[2], 2, 0); // Rear mirror index
            bitWrite(canMsgSnd.buf[2], 1, 0);
            bitWrite(canMsgSnd.buf[2], 0, 0);
            bitWrite(canMsgSnd.buf[3], 7, 1); // DSG Reset
            bitWrite(canMsgSnd.buf[3], 6, 0); // Front Collision Warning
            bitWrite(canMsgSnd.buf[3], 5, 0);
            bitWrite(canMsgSnd.buf[3], 4, 1); // XVV - Custom cruise limits Menu
            bitWrite(canMsgSnd.buf[3], 3, 1); // Recommended speed indicator
            bitWrite(canMsgSnd.buf[3], 2, bitRead(msg.buf[5], 6)); // DSG - Underinflating (3b)
            bitWrite(canMsgSnd.buf[3], 1, bitRead(msg.buf[5], 5)); // DSG - Underinflating (3b)
            bitWrite(canMsgSnd.buf[3], 0, bitRead(msg.buf[5], 4)); // DSG - Underinflating (3b)
            canMsgSnd.buf[4] = 0x00;
            canMsgSnd.buf[5] = 0x00;
            canMsgSnd.id = 0x361;
            canMsgSnd.len = 6;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }
        } else if (id == 0x260 && len == 8) { // Personalization settings status
            // Do not forward original message, it has been completely redesigned on CAN2010
            // Also forge missing messages from CAN2004

            if (msg.buf[0] == 0x01) { // User profile 1
                canMsgSnd.buf[0] = languageAndUnitNum;
                bitWrite(canMsgSnd.buf[1], 7, (mpgMi) ? 1 : 0);
                bitWrite(canMsgSnd.buf[1], 6, (TemperatureInF) ? 1 : 0);
                bitWrite(canMsgSnd.buf[1], 5, 0); // Ambiance level
                bitWrite(canMsgSnd.buf[1], 4, 1); // Ambiance level
                bitWrite(canMsgSnd.buf[1], 3, 1); // Ambiance level
                bitWrite(canMsgSnd.buf[1], 2, 1); // Parameters availability
                bitWrite(canMsgSnd.buf[1], 1, 0); // Sound Harmony
                bitWrite(canMsgSnd.buf[1], 0, 0); // Sound Harmony
                bitWrite(canMsgSnd.buf[2], 7, bitRead(msg.buf[1], 0)); // Automatic parking brake
                bitWrite(canMsgSnd.buf[2], 6, bitRead(msg.buf[1], 7)); // Selective openings - Key
                bitWrite(canMsgSnd.buf[2], 5, bitRead(msg.buf[1], 4)); // Selective openings
                bitWrite(canMsgSnd.buf[2], 4, bitRead(msg.buf[1], 5)); // Selective openings - Rear
                bitWrite(canMsgSnd.buf[2], 3, bitRead(msg.buf[1], 1)); // Driver Welcome
                bitWrite(canMsgSnd.buf[2], 2, bitRead(msg.buf[2], 7)); // Adaptative lighting
                bitWrite(canMsgSnd.buf[2], 1, bitRead(msg.buf[3], 6)); // Daytime running lights
                bitWrite(canMsgSnd.buf[2], 0, bitRead(msg.buf[3], 7)); // Ambiance lighting
                bitWrite(canMsgSnd.buf[3], 7, bitRead(msg.buf[2], 5)); // Guide-me home lighting
                bitWrite(canMsgSnd.buf[3], 6, bitRead(msg.buf[2], 1)); // Duration Guide-me home lighting (2b)
                bitWrite(canMsgSnd.buf[3], 5, bitRead(msg.buf[2], 0)); // Duration Guide-me home lighting (2b)
                bitWrite(canMsgSnd.buf[3], 4, bitRead(msg.buf[2], 6)); // Beam
                bitWrite(canMsgSnd.buf[3], 3, 0); // Lighting ?
                bitWrite(canMsgSnd.buf[3], 2, 0); // Duration Lighting (2b) ?
                bitWrite(canMsgSnd.buf[3], 1, 0); // Duration Lighting (2b) ?
                bitWrite(canMsgSnd.buf[3], 0, bitRead(msg.buf[2], 4)); // Automatic headlights
                bitWrite(canMsgSnd.buf[4], 7, bitRead(msg.buf[5], 6)); // AAS
                bitWrite(canMsgSnd.buf[4], 6, bitRead(msg.buf[6], 5)); // SAM
                bitWrite(canMsgSnd.buf[4], 5, bitRead(msg.buf[5], 4)); // Wiper in reverse
                bitWrite(canMsgSnd.buf[4], 4, 0); // Motorized tailgate
                bitWrite(canMsgSnd.buf[4], 3, bitRead(msg.buf[7], 7)); // Configurable button
                bitWrite(canMsgSnd.buf[4], 2, bitRead(msg.buf[7], 6)); // Configurable button
                bitWrite(canMsgSnd.buf[4], 1, bitRead(msg.buf[7], 5)); // Configurable button
                bitWrite(canMsgSnd.buf[4], 0, bitRead(msg.buf[7], 4)); // Configurable button

                personalizationSettings[7] = canMsgSnd.buf[1];
                personalizationSettings[8] = canMsgSnd.buf[2];
                personalizationSettings[9] = canMsgSnd.buf[3];
                personalizationSettings[10] = canMsgSnd.buf[4];
            } else { // Cached information if any other profile
                canMsgSnd.buf[0] = languageAndUnitNum;
                canMsgSnd.buf[1] = personalizationSettings[7];
                canMsgSnd.buf[2] = personalizationSettings[8];
                canMsgSnd.buf[3] = personalizationSettings[9];
                canMsgSnd.buf[4] = personalizationSettings[10];
            }
            canMsgSnd.buf[5] = 0x00;
            canMsgSnd.buf[6] = 0x00;
            canMsgSnd.id = 0x260;
            canMsgSnd.len = 7;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }

            bitWrite(canMsgSnd.buf[0], 7, 0);
            bitWrite(canMsgSnd.buf[0], 6, 0);
            bitWrite(canMsgSnd.buf[0], 5, 0);
            bitWrite(canMsgSnd.buf[0], 4, 0);
            bitWrite(canMsgSnd.buf[0], 3, 0);
            bitWrite(canMsgSnd.buf[0], 2, 1); // Parameters validity
            bitWrite(canMsgSnd.buf[0], 1, 0); // User profile
            bitWrite(canMsgSnd.buf[0], 0, 1); // User profile = 1
            canMsgSnd.buf[1] = personalizationSettings[0];
            canMsgSnd.buf[2] = personalizationSettings[1];
            canMsgSnd.buf[3] = personalizationSettings[2];
            canMsgSnd.buf[4] = personalizationSettings[3];
            canMsgSnd.buf[5] = personalizationSettings[4];
            canMsgSnd.buf[6] = personalizationSettings[5];
            canMsgSnd.buf[7] = personalizationSettings[6];
            canMsgSnd.id = 0x15B; // Personalization frame status
            canMsgSnd.len = 8;
            can2004.write(canMsgSnd);

            if (!TelematicPresent && Ignition) {
                canMsgSnd.buf[0] = 0x00;
                canMsgSnd.buf[1] = 0x10;
                canMsgSnd.buf[2] = 0xFF;
                canMsgSnd.buf[3] = 0xFF;
                canMsgSnd.buf[4] = 0x7F;
                canMsgSnd.buf[5] = 0xFF;
                canMsgSnd.buf[6] = 0x00;
                canMsgSnd.buf[7] = 0x00;
                canMsgSnd.id = 0x167; // Fake EMF status frame
                canMsgSnd.len = 8;
                can2004.write(canMsgSnd);
            }

            // Economy mode simulation
            if (EconomyMode && EconomyModeEnabled) {
                canMsgSnd.buf[0] = 0x14;
                if (Ignition) {
                    canMsgSnd.buf[5] = 0x0E;
                } else {
                    canMsgSnd.buf[5] = 0x0C;
                }
            } else {
                if (EngineRunning) {
                    canMsgSnd.buf[0] = 0x54;
                } else {
                    canMsgSnd.buf[0] = 0x04;
                }
                canMsgSnd.buf[5] = 0x0F;
            }
            canMsgSnd.buf[1] = 0x03;
            canMsgSnd.buf[2] = 0xDE;

            canMsgSnd.buf[3] = 0x00; // Increasing value,
            canMsgSnd.buf[4] = 0x00; // counter ?

            canMsgSnd.buf[6] = 0xFE;
            canMsgSnd.buf[7] = 0x00;
            canMsgSnd.id = 0x236;
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }

            // Current Time
            // If time is synced
            if (timeStatus() != timeNotSet) {
                canMsgSnd.buf[0] = (year() -
                                    1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
                canMsgSnd.buf[1] = month();
                canMsgSnd.buf[2] = day();
                canMsgSnd.buf[3] = hour();
                canMsgSnd.buf[4] = minute();
                canMsgSnd.buf[5] = 0x3F;
                canMsgSnd.buf[6] = 0xFE;
            } else {
                canMsgSnd.buf[0] = (Time_year -
                                    1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
                canMsgSnd.buf[1] = Time_month;
                canMsgSnd.buf[2] = Time_day;
                canMsgSnd.buf[3] = Time_hour;
                canMsgSnd.buf[4] = Time_minute;
                canMsgSnd.buf[5] = 0x3F;
                canMsgSnd.buf[6] = 0xFE;
            }
            canMsgSnd.id = 0x276;
            canMsgSnd.len = 7;
            can2010.write(canMsgSnd);
            if (Send_CAN2010_ForgedMessages) {
                can2004.write(canMsgSnd);
            }

            if (!EngineRunning) {
                AirConditioningON = false;
                FanSpeed = 0x41;
                LeftTemp = 0x00;
                RightTemp = 0x00;
                FanPosition = 0x04;

                canMsgSnd.buf[0] = 0x09;
                canMsgSnd.buf[1] = 0x00;
                canMsgSnd.buf[2] = 0x00;
                canMsgSnd.buf[3] = LeftTemp;
                canMsgSnd.buf[4] = RightTemp;
                canMsgSnd.buf[5] = FanSpeed;
                canMsgSnd.buf[6] = FanPosition;
                canMsgSnd.buf[7] = 0x00;
                canMsgSnd.id = 0x350;
                canMsgSnd.len = 8;
                can2010.write(canMsgSnd);
                if (Send_CAN2010_ForgedMessages) {
                    can2004.write(canMsgSnd);
                }
            }
        } else {
            can2010.write(msg);
        }
    } else {
        can2010.write(msg);
    }
}

void Peugeot_CAN::nacToCanBus(const CAN_message_t &msg) {
    int id = msg.id;
    int len = msg.len;

    if (debugCAN2010) {
        Serial.print("FRAME:ID=");
        Serial.print(id);
        Serial.print(":LEN=");
        Serial.print(len);

        char tmp[3];
        for (int i = 0; i < len; i++) {
            Serial.print(":");

            snprintf(tmp, 3, "%02X", msg.buf[i]);

            Serial.print(tmp);
        }

        Serial.println();

        can2004.write(msg);
    } else if (!debugCAN2004) {
        if (id == 0x260 || id == 0x361) {
            // Do not send back converted frames between networks
        } else if (id == 0x39B && len == 5) {
            Time_year = msg.buf[0] + 1872; // Year would not fit inside one byte (0 > 255), add 1872 and you get this new range (1872 > 2127)
            Time_month = msg.buf[1];
            Time_day = msg.buf[2];
            Time_hour = msg.buf[3];
            Time_minute = msg.buf[4];

            setTime(Time_hour, Time_minute, 0, Time_day, Time_month, Time_year);
            EEPROM.update(5, Time_day);
            EEPROM.update(6, Time_month);
            EEPROM.put(7, Time_year);

            // Set hour on CAN-BUS Clock
            canMsgSnd.buf[0] = hour();
            canMsgSnd.buf[1] = minute();
            canMsgSnd.id = 0x228;
            canMsgSnd.len = 1;
            can2004.write(canMsgSnd);

            if (SerialEnabled) {
                Serial.print("Change Hour/Date: ");
                Serial.print(day());
                Serial.print("/");
                Serial.print(month());
                Serial.print("/");
                Serial.print(year());

                Serial.print(" ");

                Serial.print(hour());
                Serial.print(":");
                Serial.print(minute());

                Serial.println();
            }
        } else if (id == 0x1A9 && len == 8) { // Telematic commands
            TelematicPresent = true;

            darkMode = bitRead(msg.buf[0], 7); // Dark mode
            resetTrip1 = bitRead(msg.buf[0], 1); // Reset Trip 1
            resetTrip2 = bitRead(msg.buf[0], 0); // Reset Trip 2
            pushAAS = bitRead(msg.buf[3], 2); // AAS
            pushSAM = bitRead(msg.buf[3], 2); // SAM
            pushDSG = bitRead(msg.buf[5], 0); // Indirect DSG reset
            pushSTT = bitRead(msg.buf[6], 7); // Start&Stop
            pushCHECK = bitRead(msg.buf[6], 0); // Check
            stopCHECK = bitRead(msg.buf[1], 7); // Stop Check
            pushBLACK = bitRead(msg.buf[5], 0); // Black Panel

            if (Ignition) {
                canMsgSnd.buf[0] = 0x00;
                bitWrite(canMsgSnd.buf[0], 7, resetTrip1); // Reset Trip 1
                bitWrite(canMsgSnd.buf[0], 6, resetTrip2); // Reset Trip 2
                canMsgSnd.buf[1] = 0x10;
                bitWrite(canMsgSnd.buf[1], 5, darkMode); // Dark mode
                canMsgSnd.buf[2] = 0xFF;
                canMsgSnd.buf[3] = 0xFF;
                canMsgSnd.buf[4] = 0x7F;
                canMsgSnd.buf[5] = 0xFF;
                canMsgSnd.buf[6] = 0x00;
                canMsgSnd.buf[7] = 0x00;
                canMsgSnd.id = 0x167; // Fake EMF Status frame
                canMsgSnd.len = 8;
                can2004.write(canMsgSnd);
            }

            if (!ClusterPresent && Ignition && (resetTrip1 || resetTrip2 || pushAAS || pushSAM || pushDSG || pushSTT || pushCHECK)) {
                canMsgSnd.buf[0] = statusCMB[0];
                canMsgSnd.buf[1] = statusCMB[1];
                bitWrite(canMsgSnd.buf[1], 4, pushCHECK);
                bitWrite(canMsgSnd.buf[1], 2, resetTrip1);
                canMsgSnd.buf[2] = statusCMB[2];
                bitWrite(canMsgSnd.buf[2], 7, pushAAS);
                bitWrite(canMsgSnd.buf[2], 6, pushASR);
                canMsgSnd.buf[3] = statusCMB[3];
                bitWrite(canMsgSnd.buf[3], 3, pushSAM);
                bitWrite(canMsgSnd.buf[3], 0, resetTrip2);
                canMsgSnd.buf[4] = statusCMB[4];
                bitWrite(canMsgSnd.buf[4], 7, pushDSG);
                canMsgSnd.buf[5] = statusCMB[5];
                canMsgSnd.buf[6] = statusCMB[6];
                bitWrite(canMsgSnd.buf[6], 7, pushSTT);
                canMsgSnd.buf[7] = statusCMB[7];
                canMsgSnd.id = 0x217;
                canMsgSnd.len = 8;
                can2004.write(canMsgSnd);
            }
        } else if (id == 0x329 && len == 8) {
            pushASR = bitRead(msg.buf[3], 0); // ESP
        } else if (id == 0x31C && len == 5) { // MATT status
            canMsgSnd.buf[0] = msg.buf[0];
            // Rewrite if necessary to make BTEL commands working
            if (resetTrip1) { // Reset Trip 1
                bitWrite(canMsgSnd.buf[0], 3, 1);
            }
            if (resetTrip2) { // Reset Trip 2
                bitWrite(canMsgSnd.buf[0], 2, 1);
            }
            canMsgSnd.buf[1] = msg.buf[1];
            canMsgSnd.buf[2] = msg.buf[2];
            canMsgSnd.buf[3] = msg.buf[3];
            canMsgSnd.buf[4] = msg.buf[4];
            canMsgSnd.id = 0x31C;
            canMsgSnd.len = 5;
            can2004.write(canMsgSnd);
        } else if (id == 0x217 && len == 8) { // Rewrite Cluster status (CIROCCO for example) for tactile touch buttons (telematic) because it is not listened by BSI
            ClusterPresent = true;

            canMsgSnd.buf[0] = msg.buf[0];
            canMsgSnd.buf[1] = msg.buf[1];
            bitWrite(canMsgSnd.buf[1], 4, pushCHECK);
            bitWrite(canMsgSnd.buf[1], 2, resetTrip1);
            canMsgSnd.buf[2] = msg.buf[2];
            bitWrite(canMsgSnd.buf[2], 7, pushAAS);
            bitWrite(canMsgSnd.buf[2], 6, pushASR);
            canMsgSnd.buf[3] = msg.buf[3];
            bitWrite(canMsgSnd.buf[3], 3, pushSAM);
            bitWrite(canMsgSnd.buf[3], 0, resetTrip2);
            canMsgSnd.buf[4] = msg.buf[4];
            bitWrite(canMsgSnd.buf[4], 7, pushDSG);
            canMsgSnd.buf[5] = msg.buf[5];
            canMsgSnd.buf[6] = msg.buf[6];
            bitWrite(canMsgSnd.buf[6], 7, pushSTT);
            canMsgSnd.buf[7] = msg.buf[7];
            canMsgSnd.id = 0x217;
            canMsgSnd.len = 8;
            can2004.write(canMsgSnd);
        } else if (id == 0x15B && len == 8) {
            if (bitRead(msg.buf[1], 2)) { // Parameters validity
                tmpVal = msg.buf[0];
                if (tmpVal >= 128) {
                    languageAndUnitNum = tmpVal;
                    EEPROM.update(0, languageAndUnitNum);

                    if (SerialEnabled) {
                        Serial.print("Telematic - Change Language and Unit (Number): ");
                        Serial.print(tmpVal);
                        Serial.println();
                    }

                    tmpVal = msg.buf[1];
                    if (tmpVal >= 128) {
                        mpgMi = true;
                        EEPROM.update(4, 1);

                        tmpVal = tmpVal - 128;
                    } else {
                        mpgMi = false;
                        EEPROM.update(4, 0);
                    }

                    if (tmpVal >= 64) {
                        TemperatureInF = true;
                        EEPROM.update(3, 1);

                        if (SerialEnabled) {
                            Serial.print("Telematic - Change Temperature Type: Fahrenheit");
                            Serial.println();
                        }
                    } else if (tmpVal >= 0) {
                        TemperatureInF = false;
                        EEPROM.update(3, 0);

                        if (SerialEnabled) {
                            Serial.print("Telematic - Change Temperature Type: Celcius");
                            Serial.println();
                        }
                    }
                } else {
                    tmpVal = ceil(tmpVal / 4.0);
                    if (msg.buf[1] >= 128) {
                        tmpVal--;
                    }
                    languageID = tmpVal;

                    // CAN2004 Head-up panel is only one-way talking, we can't change the language on it from the CAN2010 Telematic :-(

                    if (SerialEnabled) {
                        Serial.print("Telematic - Change Language (ID): ");
                        Serial.print(tmpVal);
                        Serial.println();
                    }
                }

                // Personalization settings change
                bitWrite(canMsgSnd.buf[0], 7, 0);
                bitWrite(canMsgSnd.buf[0], 6, 0);
                bitWrite(canMsgSnd.buf[0], 5, 0);
                bitWrite(canMsgSnd.buf[0], 4, 0);
                bitWrite(canMsgSnd.buf[0], 3, 0);
                bitWrite(canMsgSnd.buf[0], 2, 0); // Parameters validity, 0 = Changed parameter(s) the BSI must take into account
                bitWrite(canMsgSnd.buf[0], 1, 0); // User profile
                bitWrite(canMsgSnd.buf[0], 0, 1); // User profile = 1
                bitWrite(canMsgSnd.buf[1], 7, bitRead(msg.buf[2], 6)); // Selective openings
                bitWrite(canMsgSnd.buf[1], 6, 1);
                bitWrite(canMsgSnd.buf[1], 5, bitRead(msg.buf[2], 4)); // Selective rear openings
                bitWrite(canMsgSnd.buf[1], 4, bitRead(msg.buf[2], 5)); // Selective openings
                bitWrite(canMsgSnd.buf[1], 3, 0);
                bitWrite(canMsgSnd.buf[1], 2, 0);
                bitWrite(canMsgSnd.buf[1], 1, bitRead(msg.buf[2], 3)); // Driver welcome
                bitWrite(canMsgSnd.buf[1], 0, bitRead(msg.buf[2], 7)); // Parking brake
                bitWrite(canMsgSnd.buf[2], 7, bitRead(msg.buf[2], 2)); // Adaptative lighting
                bitWrite(canMsgSnd.buf[2], 6, bitRead(msg.buf[3], 4)); // Beam
                bitWrite(canMsgSnd.buf[2], 5, bitRead(msg.buf[3], 7)); // Guide-me home lighting
                bitWrite(canMsgSnd.buf[2], 4, bitRead(msg.buf[3], 0)); // Automatic headlights
                bitWrite(canMsgSnd.buf[2], 3, 0);
                bitWrite(canMsgSnd.buf[2], 2, 0);
                bitWrite(canMsgSnd.buf[2], 1, bitRead(msg.buf[3], 6)); // Duration Guide-me home lighting (2b)
                bitWrite(canMsgSnd.buf[2], 0, bitRead(msg.buf[3], 5)); // Duration Guide-me home lighting (2b)
                bitWrite(canMsgSnd.buf[3], 7, bitRead(msg.buf[2], 0)); // Ambiance lighting
                bitWrite(canMsgSnd.buf[3], 6, bitRead(msg.buf[2], 1)); // Daytime running lights
                bitWrite(canMsgSnd.buf[3], 5, 0);
                bitWrite(canMsgSnd.buf[3], 4, 0);
                bitWrite(canMsgSnd.buf[3], 3, 0);
                bitWrite(canMsgSnd.buf[3], 2, 0);
                bitWrite(canMsgSnd.buf[3], 1, 0);
                bitWrite(canMsgSnd.buf[3], 0, 0);
                canMsgSnd.buf[4] = 0x00;
                bitWrite(canMsgSnd.buf[5], 7, bitRead(msg.buf[4], 7)); // AAS
                bitWrite(canMsgSnd.buf[5], 6, bitRead(msg.buf[4], 7)); // AAS
                bitWrite(canMsgSnd.buf[5], 5, 0);
                bitWrite(canMsgSnd.buf[5], 4, bitRead(msg.buf[4], 5)); // Wiper in reverse
                bitWrite(canMsgSnd.buf[5], 3, 0);
                bitWrite(canMsgSnd.buf[5], 2, 0);
                bitWrite(canMsgSnd.buf[5], 1, 0);
                bitWrite(canMsgSnd.buf[5], 0, 0);
                bitWrite(canMsgSnd.buf[6], 7, 0);
                bitWrite(canMsgSnd.buf[6], 6, bitRead(msg.buf[4], 6)); // SAM
                bitWrite(canMsgSnd.buf[6], 5, bitRead(msg.buf[4], 6)); // SAM
                bitWrite(canMsgSnd.buf[6], 4, 0);
                bitWrite(canMsgSnd.buf[6], 3, 0);
                bitWrite(canMsgSnd.buf[6], 2, 0);
                bitWrite(canMsgSnd.buf[6], 1, 0);
                bitWrite(canMsgSnd.buf[6], 0, 0);
                bitWrite(canMsgSnd.buf[7], 7, bitRead(msg.buf[4], 3)); // Configurable button
                bitWrite(canMsgSnd.buf[7], 6, bitRead(msg.buf[4], 2)); // Configurable button
                bitWrite(canMsgSnd.buf[7], 5, bitRead(msg.buf[4], 1)); // Configurable button
                bitWrite(canMsgSnd.buf[7], 4, bitRead(msg.buf[4], 0)); // Configurable button
                bitWrite(canMsgSnd.buf[7], 3, 0);
                bitWrite(canMsgSnd.buf[7], 2, 0);
                bitWrite(canMsgSnd.buf[7], 1, 0);
                bitWrite(canMsgSnd.buf[7], 0, 0);
                canMsgSnd.id = 0x15B;
                canMsgSnd.len = 8;
                can2004.write(canMsgSnd);

                // Store personalization settings for the recurring frame
                personalizationSettings[0] = canMsgSnd.buf[1];
                personalizationSettings[1] = canMsgSnd.buf[2];
                personalizationSettings[2] = canMsgSnd.buf[3];
                personalizationSettings[3] = canMsgSnd.buf[4];
                personalizationSettings[4] = canMsgSnd.buf[5];
                personalizationSettings[5] = canMsgSnd.buf[6];
                personalizationSettings[6] = canMsgSnd.buf[7];
                EEPROM.update(10, personalizationSettings[0]);
                EEPROM.update(11, personalizationSettings[1]);
                EEPROM.update(12, personalizationSettings[2]);
                EEPROM.update(13, personalizationSettings[3]);
                EEPROM.update(14, personalizationSettings[4]);
                EEPROM.update(15, personalizationSettings[5]);
                EEPROM.update(16, personalizationSettings[6]);
            }
        } else if (id == 0x1E9 && len >= 2 && CVM_Emul) { // Telematic suggested speed to fake CVM frame
            can2004.write(msg);

            tmpVal = (msg.buf[3] >> 2); // POI type - Gen2 (6b)

            canMsgSnd.buf[0] = msg.buf[1];
            canMsgSnd.buf[1] = ((tmpVal > 0 && vehicleSpeed > msg.buf[0]) ? 0x30 : 0x10); // POI Over-speed, make speed limit blink
            canMsgSnd.buf[2] = 0x00;
            canMsgSnd.buf[3] = 0x00;
            canMsgSnd.buf[4] = 0x7C;
            canMsgSnd.buf[5] = 0xF8;
            canMsgSnd.buf[6] = 0x00;
            canMsgSnd.buf[7] = 0x00;
            canMsgSnd.id = 0x268; // CVM Frame ID
            canMsgSnd.len = 8;
            can2010.write(canMsgSnd);
        } else if (id == 0x1E5 && len == 7) {
            canMsgRcv = msg;
            // Ambience mapping
            tmpVal = canMsgRcv.buf[5];
            if (tmpVal == 0x00) { // User
                canMsgRcv.buf[6] = 0x40;
            } else if (tmpVal == 0x08) { // Classical
                canMsgRcv.buf[6] = 0x44;
            } else if (tmpVal == 0x10) { // Jazz
                canMsgRcv.buf[6] = 0x48;
            } else if (tmpVal == 0x18) { // Pop-Rock
                canMsgRcv.buf[6] = 0x4C;
            } else if (tmpVal == 0x28) { // Techno
                canMsgRcv.buf[6] = 0x54;
            } else if (tmpVal == 0x20) { // Vocal
                canMsgRcv.buf[6] = 0x50;
            } else { // Default : User
                canMsgRcv.buf[6] = 0x40;
            }

            // Loudness / Volume linked to speed
            tmpVal = msg.buf[4];
            if (tmpVal == 0x10) { // Loudness / not linked to speed
                canMsgRcv.buf[5] = 0x40;
            } else if (tmpVal == 0x14) { // Loudness / Volume linked to speed
                canMsgRcv.buf[5] = 0x47;
            } else if (tmpVal == 0x04) { // No Loudness / Volume linked to speed
                canMsgRcv.buf[5] = 0x07;
            } else if (tmpVal == 0x00) { // No Loudness / not linked to speed
                canMsgRcv.buf[5] = 0x00;
            } else { // Default : No Loudness / not linked to speed
                canMsgRcv.buf[5] = 0x00;
            }

            // Bass
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = msg.buf[2];
            canMsgRcv.buf[2] = ((tmpVal - 32) / 4) + 57; // Converted value

            // Treble
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = msg.buf[3];
            canMsgRcv.buf[4] = ((tmpVal - 32) / 4) + 57; // Converted value on position 4 (while it's on 3 on a old amplifier)

            // Balance - Left / Right
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = msg.buf[1];
            canMsgRcv.buf[1] = ((tmpVal - 32) / 4) + 57; // Converted value

            // Balance - Front / Back
            // CAN2004 Telematic Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
            // CAN2010 Telematic Range: "32" > "88" ("60" = 0)
            tmpVal = msg.buf[0];
            canMsgRcv.buf[0] = ((tmpVal - 32) / 4) + 57; // Converted value

            // Mediums ?
            canMsgRcv.buf[3] = 63; // 0x3F = 63

            can2004.write(canMsgRcv);
        } else {
            can2004.write(msg);
        }
    } else {
        can2004.write(msg);
    }
}

int Peugeot_CAN::daysSinceYearStartFct() {
    // Given a day, month, and year (4 digit), returns
    // the day of year. Errors return 999.
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    // Check if it is a leap year, this is confusing business
    // See: https://support.microsoft.com/en-us/kb/214019
    if (year() % 4 == 0) {
        if (year() % 100 != 0) {
            daysInMonth[1] = 29;
        } else {
            if (year() % 400 == 0) {
                daysInMonth[1] = 29;
            }
        }
    }

    int doy = 0;
    for (int i = 0; i < month() - 1; i++) {
        doy += daysInMonth[i];
    }

    doy += day();
    return doy;
}
