#include "OneWire.h"

#define DEBUG           ->comment this line to prevent code from writing debug messages to serial port
#include "DebugUtils.h"

#if defined(CONTROLLINO_MAXI) //Controllino Maxi board specifics

#include <Controllino.h>

//output relays pin definitions
#define FILTRATION_PUMP CONTROLLINO_R4  //CONTROLLINO_RELAY_4
#define PH_PUMP    CONTROLLINO_R3       //CONTROLLINO_RELAY_3
#define CHL_PUMP   CONTROLLINO_R5       //CONTROLLINO_RELAY_5
#define HEAT_ON    CONTROLLINO_R0       //CONTROLLINO_RELAY_0

#define RELAY_R1   CONTROLLINO_R1       //CONTROLLINO_RELAY_1
#define RELAY_R2   CONTROLLINO_R2       //CONTROLLINO_RELAY_2
#define RELAY_R6   CONTROLLINO_R6       //CONTROLLINO_RELAY_6
#define RELAY_R7   CONTROLLINO_R7       //CONTROLLINO_RELAY_7
#define RELAY_R8   CONTROLLINO_R8       //CONTROLLINO_RELAY_8
#define RELAY_R9   CONTROLLINO_R9       //CONTROLLINO_RELAY_9

//Digital input pins connected to Acid and Chl tank level reed switches
#define CHL_LEVEL  CONTROLLINO_D1       //CONTROLLINO_D1 pin 3
#define PH_LEVEL   CONTROLLINO_D3       //CONTROLLINO_D3 pin 5

//Analog input pins connected to Phidgets 1130_0 pH/ORP Adapters.
//Galvanic isolation circuitry between Adapters and Arduino required!
#define ORP_MEASURE CONTROLLINO_A2      //CONTROLLINO_A2 pin A2 on pin header connector, not on screw terminal (/!\)
#define PH_MEASURE  CONTROLLINO_A4      //CONTROLLINO_A4 pin A4 on pin header connector, not on screw terminal (/!\)

//Analog input pin connected to pressure sensor
#define PSI_MEASURE CONTROLLINO_A9      //CONTROLLINO_A9 pin A9 on pin header connector, not on screw terminal (/!\)

//Front panel push button switch
#define PUSH_BUTTON_PIN  CONTROLLINO_A5   //CONTROLLINO_A5 pin A5. Connect a button switch from this pin to ground
#define GREEN_LED_PIN    CONTROLLINO_D0  //CONTROLLINO_D0). Digital output pin to switch ON/OFF Green LED of push button
#define RED_LED_PIN      CONTROLLINO_D2  //CONTROLLINO_D2). Digital output pin to switch ON/OFF Red LED of push button

#else //Mega2560 board specifics

#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;

#if !( defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) )
#error This code is intended to run only on the Arduino Mega 1280/2560 boards ! Please check your Tools->Board setting.
#endif
// #define EspSerial Serial3
#define EEPROM_START      512
// #include <Esp8266_AT_WM_Lite.h>

#define FILTRATION_PUMP A14
#define PH_PUMP         36
#define CHL_PUMP        42
#define HEAT_ON         58

#define RELAY_R1   37
#define RELAY_R2   31
#define RELAY_R6   32
#define RELAY_R7   33
#define RELAY_R8   34
#define RELAY_R9   35


//Digital input pins connected to Acid and Chl tank level reed switches
#define CHL_LEVEL       28
#define PH_LEVEL        30

//Analog input pins connected to Phidgets 1130_0 pH/ORP Adapters.
//Galvanic isolation circuitry between Adapters and Arduino required!
#define ORP_MEASURE     A2
#define PH_MEASURE      A0

//Analog input pin connected to pressure sensor
#define PSI_MEASURE     A10

//Front panel push button switch
#define PUSH_BUTTON_PIN  A12   //Connect a button switch from this pin to ground
#define GREEN_LED_PIN    2  //Digital output pin to switch ON/OFF Green LED of push button
#define RED_LED_PIN      5  //Digital output pin to switch ON/OFF Red LED of push button

#endif

//One wire bus for the water temperature measurement
//Data wire is connected to input digital pin 6 on the Arduino
#define ONE_WIRE_BUS_A 6

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire_A(ONE_WIRE_BUS_A);
#include <DallasTemperature.h>
#include <MQTT.h>

// Pass our oneWire reference to Dallas Temperature library instance
DallasTemperature sensors_A(&oneWire_A);

//12bits (0,06°C) temperature sensor resolution
#define TEMPERATURE_RESOLUTION 12

//MAC Address of DS18b20 water temperature sensor
DeviceAddress DS18b20_0 = { 0x28, 0x92, 0x25, 0x41, 0x0A, 0x00, 0x00, 0xEE };
String sDS18b20_0;

String sArduinoMac;
//IPAddress ip(192, 168, 0, 188);  //IP address, needs to be adapted depending on local network topology

//Version of config stored in Eeprom
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION 121

//interval (in miilisec) between MQTT publishes of measurement data
#define PublishInterval 30000

//Settings structure and its default values
struct StoreStruct
{
  uint8_t ConfigVersion;   // This is for testing if first time using eeprom or not
  bool Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode, WaterHeat;
  uint8_t FiltrationStart, FiltrationDuration, FiltrationStopMax, FiltrationStop, DelayPIDs;
  unsigned long PhPumpUpTimeLimit, ChlPumpUpTimeLimit;
  unsigned long PhPIDWindowSize, OrpPIDWindowSize, PhPIDwindowStartTime, OrpPIDwindowStartTime;
  double Ph_SetPoint, Orp_SetPoint, PSI_HighThreshold, PSI_MedThreshold, WaterTempLowThreshold, WaterTemp_SetPoint, TempExternal, pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1, PSICalibCoeffs0, PSICalibCoeffs1;
  double Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, PhPIDOutput, OrpPIDOutput, TempValue, PhValue, OrpValue, PSIValue;
  double AcidFill, ChlFill, pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR;
  byte ip[4], subnet[4], gateway[4], dnsserver[4], mac[6];
  bool ipConfiged;
} storage =
{ //default values. Change the value of CONFIG_VERSION in order to restore the default values
  CONFIG_VERSION,
  0, 0, 1, 0,
  8, 12, 20, 20, 120,
  900, 2500,
  3000000, 3600000, 0, 0,
  7.4, 750.0, 0.5, 0.25, 10.0, 27.0, 3.0, 4.3, -2.63, -1189, 2564, 1.11, 0.0,
  2000000.0, 0.0, 0.0, 2500.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4,
  100.0, 100.0, 20.0, 20.0, 1.5, 3.0,
  {192, 168, 0, 188}, {255, 255, 255, 0}, {192, 168, 0, 254}, {8, 8, 8, 8}, {0xA8, 0x61, 0x0A, 0xAE, 0x2C, 0x68},
  0
};

  // MAC address of Ethernet shield (in case of Controllino board, set an arbitrary MAC address)
  byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0x65, 0x04}; //-> Mega2560 dev setup with Ethernet shield
/*
  //MQTT stuff including local broker/server IP address, login and pwd
  MQTTClient MQTTClient;
  const char* MqttServerIP = "192.168.0.38";
  //const char* MqttServerIP = "broker.mqttdashboard.com";//cloud-based MQTT broker to test when node-red and MQTT broker are not installed locally (/!\ public and unsecure!)
  const char* MqttServerClientID = "ArduinoPoolTest"; // /!\ choose a client ID which is unique to this Arduino board
  const char* MqttServerLogin = nullptr;  //replace by const char* MqttServerLogin = nullptr; in case broker does not require a login/pwd
  const char* MqttServerPwd = nullptr; //replace by const char* MqttServerPwd = nullptr; in case broker does not require a login/pwd
  const char* PoolTopicMeas1 = "Home/PoolTest/Meas1";
  const char* PoolTopicMeas2 = "Home/PoolTest/Meas2";
  const char* PoolTopicSet1 = "Home/PoolTest/Set1";
  const char* PoolTopicSet2 = "Home/PoolTest/Set2";
  const char* PoolTopicSet3 = "Home/PoolTest/Set3";
  const char* PoolTopicSet4 = "Home/PoolTest/Set4";
  const char* PoolTopicSet5 = "Home/PoolTest/Set5";
  const char* PoolTopicAPI = "Home/PoolTest/API";
  const char* PoolTopicStatus = "Home/PoolTest/status";
  const char* PoolTopicError = "Home/PoolTest/Err";

*/
//MQTT stuff including local broker/server IP address, login and pwd
MQTTClient MQTTClient;
const char* MqttServerIP = "192.168.0.38";
//const char* MqttServerIP = "broker.mqttdashboard.com";//cloud-based MQTT broker to test when node-red and MQTT broker are not installed locally (/!\ public and unsecure!)
const char* MqttServerClientID = "ArduinoPool2"; // /!\ choose a client ID which is unique to this Arduino board
const char* MqttServerLogin = nullptr;  //replace by const char* MqttServerLogin = nullptr; in case broker does not require a login/pwd
const char* MqttServerPwd = nullptr; //replace by const char* MqttServerPwd = nullptr; in case broker does not require a login/pwd
const char* PoolTopicMeas1 = "Home/Pool/Meas1";
const char* PoolTopicMeas2 = "Home/Pool/Meas2";
const char* PoolTopicSet1 = "Home/Pool/Set1";
const char* PoolTopicSet2 = "Home/Pool/Set2";
const char* PoolTopicSet3 = "Home/Pool/Set3";
const char* PoolTopicSet4 = "Home/Pool/Set4";
const char* PoolTopicSet5 = "Home/Pool/Set5";
const char* PoolTopicAPI = "Home/Pool/API";
const char* PoolTopicStatus = "Home/Pool/status";
const char* PoolTopicError = "Home/Pool/Err";
