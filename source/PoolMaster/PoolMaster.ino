/*

  Arduino/Controllino-Maxi/ATmega2560 based Ph/ORP regulator for home pool sysem
  (c) Loic74 <loic74650@gmail.com> 2018-2023

***how to compile***
  - select the target board type in the Arduino IDE (either "Arduino Mega 2560" or "Controllino Maxi")
  - in the Config.h file:
      - uncomment "#define DEBUG" if you wish debug info on the serial interface
      - if using a MEGA 2560 board with an Ethernet shield, update the MAC address of the Ethernet shield in order to reflect yours
      - update the LAN IP address of the MQTT broker ("MqttServerIP") as well as the credentials if any
      - update address of DS18b20 water temperature sensor ("DS18b20_0")
      - comment the "#define pHOrpBoard" if you are NOT using the I2C pH_Orp_Board as interface to the pH and Orp probes but rather the analog Phidget boards (PoolMaster V5.0 and earlier)


***Compatibility***
  For this sketch to work on your setup you must change the following in the code:
  - possibly the pinout definitions in case you are not using a CONTROLLINO MAXI board
  - MAC address of DS18b20 water temperature sensor
  - MAC and IP address of the Ethernet shield
  - MQTT broker IP address and login credentials
  - possibly the topic names on the MQTT broker to subscribe and publish to
  - the Kp,Ki,Kd parameters for both PID loops in case your peristaltic pumps have a different throughput than 1.5Liters/hour for the pH pump and 3.0Liters/hour for the Chlorine pump.
  Also the default Kp values were adjusted for a 50m3 pool volume. You might have to adjust the Kp values in case of a different pool volume and/or peristaltic pumps throughput
  (start by adjusting it proportionally). In any case these parameters are likely to require adjustments for every pool

***Brief description:***
  Four main metrics are measured and periodically reported over MQTT: water temperature and pressure, PH and ORP values
  Pumps states, tank-level estimates and other parmaters are also periodically reported
  Two PID regulation loops are running in parallel: one for PH, one for ORP
  An additional simple on/off regulation loop is handling the water temperature (starts/stops the heating system circulator)
  PH is regulated by injecting Acid from a tank into the pool water (a relay starts/stops the Acid peristaltic pump)
  ORP is regulated by injecting Chlorine from a tank into the pool water (a relay starts/stops the Chlorine peristaltic pump)
  Defined time-slots and water temperature are used to start/stop the filtration pump for a daily given amount of time (a relay starts/stops the filtration pump)
  A lightweight webserver provides a simple dynamic webpage with a summary of all system parameters
  Communication with the system is performed using the MQTT protocol over an Ethernet connection to the local network/MQTT broker.
  Every 30 seconds (by default), the system will publish on the "PoolTopicMeas1" and "PoolTopicMeas2"(see in code below) the following payloads in Json format:
  {"Tmp":818,"pH":321,"PSI":56,"Orp":583,"FilUpT":8995,"PhUpT":0,"ChlUpT":0}
  {"AcidF":100,"ChlF":100,"IO":11,"IO2":0}
  Tmp: measured Water temperature value in °C x100 (8.18°C in the above example payload)
  pH: measured pH value x100 (3.21 in the above example payload)
  Orp: measured Orp (aka Redox) value in mV (583mV in the above example payload)
  PSI: measured Water pressure value in bar x100 (0.56bar in the above example payload)
  FiltUpT: current running time of Filtration pump in seconds (reset every 24h. 8995secs in the above example payload)
  PhUpT: current running time of Ph pump in seconds (reset every 24h. 0secs in the above example payload)
  ChlUpT: current running time of Chl pump in seconds (reset every 24h. 0secs in the above example payload)
  AcidF: percentage fill estimate of acid tank ("pHTank" command must have been called when a new acid tank was set in place in order to have accurate value)
  ChlF: percentage fill estimate of Chlorine tank ("ChlTank" command must have been called when a new Chlorine tank was set in place in order to have accurate value)
  IO: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are :
    FiltPump: current state of Filtration Pump (0=on, 1=off)
    PhPump: current state of Ph Pump (0=on, 1=off)
    ChlPump: current state of Chl Pump (0=on, 1=off)
    PhlLevel: current state of Acid tank level (0=empty, 1=ok)
    ChlLevel: current state of Chl tank level (0=empty, 1=ok)
    PSIError: over-pressure error
    pHErr: pH pump overtime error flag
    ChlErr: Chl pump overtime error flag
  IO2: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are :
    pHPID: current state of pH PID regulation loop (1=on, 0=off)
    OrpPID: current state of Orp PID regulation loop (1=on, 0=off)
    Mode: state of pH and Orp regulation mode (0=manual, 1=auto)
    Heat: state of water heat command (0=off, 1=on)
    R1: state of Relay1
    R2: state of Relay2
    R6: state of Relay6
    R7: state of Relay7

***MQTT API***
  Below are the Payloads/commands to publish on the "PoolTopicAPI" topic (see in code below) in Json format in order to launch actions on the Arduino:
  {"Mode":1} or {"Mode":0}         -> set "Mode" to manual (0) or Auto (1). In Auto, filtration starts/stops at set times of the day and PID's are enabled/disabled
  {"Heat":1} or {"Heat":0}         -> start/stop the regulation of the pool water temperature
  {"FiltPump":1} or {"FiltPump":0} -> manually start/stop the filtration pump.
  {"ChlPump":1} or {"ChlPump":0}   -> manually start/stop the Chl pump to add more Chlorine
  {"PhPump":1} or {"PhPump":0}     -> manually start/stop the Acid pump to lower the Ph
  {"PhPID":1} or {"PhPID":0}       -> start/stop the Ph PID regulation loop
  {"OrpPID":1} or {"OrpPID":0}     -> start/stop the Orp PID regulation loop
  {"PhCalib":[4.02,3.8,9.0,9.11]}  -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
  {"OrpCalib":[450,465,750,784]}   -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
  {"PSICalib":[0,0,0.71,0.6]}      -> multi-point linear regression calibration (minimum 2 point-couple, 6 max.) in the form [ElectronicPressureSensorReading_0, MechanicalPressureSensorReading_0, xx, xx, ElectronicPressureSensorReading_n, MechanicalPressureSensorReading_n]. Mechanical pressure sensor is typically located on the sand filter
  {"PhSetPoint":7.4}               -> set the Ph setpoint, 7.4 in this example
  {"OrpSetPoint":750.0}            -> set the Orp setpoint, 750mV in this example
  {"WSetPoint":27.0}               -> set the water temperature setpoint, 27.0deg in this example
  {"WTempLow":10.0}                -> set the water low-temperature threshold below which there is no need to regulate Orp and Ph (ie. in winter)
  {"OrpPIDParams":[4000,0,0]}      -> respectively set Kp,Ki,Kd parameters of the Orp PID loop. In this example they are set to 2857, 0 and 0
  {"PhPIDParams":[2000000,0,0.0]}  -> respectively set Kp,Ki,Kd parameters of the Ph PID loop. In this example they are set to 1330000, 0 and 0.0
  {"OrpPIDWSize":3600000}          -> set the window size of the Orp PID loop (in msec), 60mins in this example
  {"PhPIDWSize":3600000}           -> set the window size of the Ph PID loop (in msec), 60mins in this example
  {"Date":[1,1,1,18,13,32,0]}      -> set date/time of RTC module in the following format: (Day of the month, Day of the week, Month, Year, Hour, Minute, Seconds), in this example: Monday 1st January 2018 - 13h32mn00secs
  {"URTC":1}                       -> Launch automatic update of Date/Time of RTC by sending an NTP request to a time server (requires an internet connection)
  {"FiltT0":9}                     -> set the earliest hour (9:00 in this example) to run filtration pump. Filtration pump will not run beofre that hour
  {"FiltT1":20}                    -> set the latest hour (20:00 in this example) to run filtration pump. Filtration pump will not run after that hour
  {"PubPeriod":30}                 -> set the periodicity (in seconds) at which the system info (pumps states, tank levels states, measured values, etc) will be published to the MQTT broker
  {"PumpsMaxUp":1800}              -> set the Max Uptime (in secs) for the Ph and Chl pumps over a 24h period. If over, PID regulation is stopped and a warning flag is raised
  {"Clear":1}                      -> reset the pH and Orp pumps overtime error flags in order to let the regulation loops continue. "Mode", "PhPID" and "OrpPID" commands need to be switched back On (1) after an error flag was raised
  {"DelayPID":60}                  -> Delay (in mins) after FiltT0 before the PID regulation loops will start. This is to let the Orp and pH readings stabilize first. 30mins in this example. Should not be > 59mins
  {"TempExt":4.2}                  -> Provide the external temperature. Should be updated regularly and will be used to start filtration when outside air temperature is <-2.0deg. 4.2deg in this example
  {"PSIHigh":1.0}                  -> set the water high-pressure threshold (1.0bar in this example). When water pressure is over that threshold, an error flag is set.
  {"pHTank":[20,100]}              -> call this command when the Acid tank is replaced or refilled. First parameter is the tank volume in Liters, second parameter is its percentage fill (100% when full)
  {"ChlTank":[20,100]}             -> call this command when the Chlorine tank is replaced or refilled. First parameter is the tank volume in Liters, second parameter is its percentage fill (100% when full)
  {"Relay":[1,1]}                  -> call this generic command to actuate spare relays. Parameter 1 is the relay number (R1 in this example), parameter 2 is the relay state (ON in this example). This command is useful to use spare relays for additional features (lighting, etc). Available relay numbers are 1,2,6,7,8,9
  {"Reboot":1}                     -> call this command to reboot the controller (after 8 seconds from calling this command)
  {"pHPumpFR":1.5}                 -> call this command to set pH pump flow rate un L/h. In this example 1.5L/h
  {"ChlPumpFR":3}                  -> call this command to set Chl pump flow rate un L/h. In this example 3L/h
  {"RstpHCal":1}                   -> call this command to reset the calibration coefficients of the pH probe
  {"RstOrpCal":1}                   -> call this command to reset the calibration coefficients of the Orp probe
  {"RstPSICal":1}                   -> call this command to reset the calibration coefficients of the pressure sensor

***Dependencies and respective revisions used to compile this project***
  https://github.com/256dpi/arduino-mqtt/releases (rev 2.5.2)
  https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library (rev 3.0.10)
  https://github.com/PaulStoffregen/OneWire (rev 2.3.8)
  https://github.com/milesburton/Arduino-Temperature-Control-Library (rev 3.9.1)
  https://github.com/RobTillaart/RunningMedian (rev 0.3.9)
  https://github.com/prampec/arduino-softtimer (rev 3.3.0)
  https://github.com/prampec/arduino-pcimanager/tree/master (rev 2.1.4)
  https://github.com/bricofoy/yasm (rev 1.1.0)
  https://github.com/br3ttb/Arduino-PID-Library (rev 1.2.1)
  https://github.com/bblanchon/ArduinoJson (rev 7.3.0)
  https://github.com/thijse/Arduino-EEPROMEx (rev 1.0.0)
  https://github.com/EinarArnason/ArduinoQueue (rev 1.2.5)
  https://github.com/PaulStoffregen/Time (rev 1.6.1) 
  https://github.com/adafruit/RTClib (rev 2.1.4)
  https://github.com/TrippyLighting/EthernetBonjour
  https://github.com/Seithan/EasyNextionLibrary (rev 1.0.6)
  http://arduiniana.org/libraries/streaming/ (rev 5)
  https://github.com/tardate/TextFinder
  https://lygte-info.dk/pic/Projects/ADS1115Library/ADS1115.zip (rev 1.0.0)  -> documentation here: https://lygte-info.dk/project/ADS1115Library%20UK.html

*/
#include "Config.h"
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetBonjour.h>
#include <SD.h>
#include <TimeLib.h>
#include <RunningMedian.h>
#include <SoftTimer.h>
#include <yasm.h>
#include <PID_v1.h>
#include <Streaming.h>
#include <Wire.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <ArduinoJson.h>
#include <EEPROMex.h>
#include "ArduinoQueue.h"
#include "Pump.h"
#include "EasyNextionLibrary.h"  // Include EasyNextionLibrary
#include <ADS1115.h>

// Firmware revision
String Firmw = "7.0.0";

//Starting point address where to store the config data in EEPROM
#define memoryBase 32
int configAdress = 0;
const int maxAllowedWrites = 200;//not sure what this is for

//Queue object to store incoming JSON commands (up to 10)
#define QUEUE_SIZE_ITEMS 10
#define QUEUE_SIZE_BYTES 1000
ArduinoQueue<String> queueIn(QUEUE_SIZE_ITEMS, QUEUE_SIZE_BYTES);

//Queue object to store outgoing JSON messages (up to 7)
//buffers for MQTT string payload
#define PayloadBufferLength 150
char Payload[PayloadBufferLength];

//Nextion TFT object. Choose which ever Serial port
//you wish to connect to (not "Serial" which is used for debug), here Serial2 UART
EasyNex myNex(Serial2);

bool EmergencyStopFiltPump = false;             // flag will be (re)set by double-tapp button

//buffer used to capture HTTP requests
String readString;

bool PSIError = 0;

//The four pumps of the system (instanciate the Pump class)
//In this case, all pumps start/Stop are managed by the Arduino relays
//In the case of the filtration pump not being managed by the Arduino, the two first pin parameters "FILTRATION_PUMP" might differ (see Pump class for more details)
Pump HeatCirculatorPump(HEAT_ON, HEAT_ON, NO_TANK, FILTRATION_PUMP, 0.0, 0.0);
Pump FiltrationPump(FILTRATION_PUMP, FILTRATION_PUMP, NO_TANK, NO_INTERLOCK, 0.0, 0.0);
Pump PhPump(PH_PUMP, PH_PUMP, PH_LEVEL, FILTRATION_PUMP, storage.pHPumpFR, storage.pHTankVol);
Pump ChlPump(CHL_PUMP, CHL_PUMP, CHL_LEVEL, FILTRATION_PUMP, storage.ChlPumpFR, storage.ChlTankVol);

//Tank level error flags
bool PhLevelError = 0;
bool ChlLevelError = 0;

//Status of connection to broker
bool MQTTConnection = false;

//PIDs instances
//Specify the links and initial tuning parameters
PID PhPID(&storage.PhValue, &storage.PhPIDOutput, &storage.Ph_SetPoint, storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd, REVERSE);
PID OrpPID(&storage.OrpValue, &storage.OrpPIDOutput, &storage.Orp_SetPoint, storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd, DIRECT);

//Filtration anti freeze mode
bool AntiFreezeFiltering = false;

//BitMaps with GPIO states
uint8_t BitMap = 0;
uint8_t BitMap2 = 0;

//MQTT publishing periodicity of system info, in msecs
unsigned long PublishPeriod = 30000;

#if defined(pHOrpBoard) //using the I2C pHOrpBoard as interface to the pH and Orp probes
//Instance of ADC library to measure pH and Orp using the pH_Orp_Board
ADS1115 adc(ADS1115ADDRESS+1);
#endif

//Signal filtering library. Only used in this case to compute the average
//over multiple measurements but offers other filtering functions such as median, etc.
RunningMedian samples_Temp = RunningMedian(10);
RunningMedian samples_Ph = RunningMedian(10);
RunningMedian samples_Orp = RunningMedian(10);
RunningMedian samples_PSI = RunningMedian(3);
RunningMedian samples_Ph2 = RunningMedian(10);//temporary, for testing pH_Orp_Board
RunningMedian samples_Orp2 = RunningMedian(10);//temporary, for testing pH_Orp_Board

EthernetServer server(80);      //Create a server at port 80
EthernetClient net;             //Ethernet client to connect to MQTT server

//Date-Time variables for use with internal RTC (Real Time Clock) module
char TimeBuffer[25];
bool DoneForTheDay = false;

//serial printing stuff
String _endl = "\n";

//State Machine
//Getting a 12 bits temperature reading on a DS18b20 sensor takes >750ms
//Here we use the sensor in asynchronous mode, request a temp reading and use
//the nice "YASM" state-machine library to do other things while it is being obtained
YASM gettemp;

//Callbacks
//Here we use the SoftTimer library which handles multiple timers (Tasks)
//It is more elegant and readable than a single loop() functtion, especially
//when tasks with various frequencies are to be used
void EthernetClientCallback(Task* me);
void OrpRegulationCallback(Task* me);
void PHRegulationCallback(Task* me);
void GenericCallback(Task* me);
void PublishDataCallback(Task* me);

Task t1(500, EthernetClientCallback);         //Check for Ethernet client every 0.5 secs
Task t2(1000, OrpRegulationCallback);         //ORP regulation loop every 1 sec
Task t3(1100, PHRegulationCallback);          //PH regulation loop every 1.1 sec
Task t4(PublishInterval, PublishDataCallback);          //Publish data to MQTT broker every 30 secs
Task t5(600, GenericCallback);                 //Various things handled/updated in this loop every 0.6 secs


#if !defined(CONTROLLINO_MAXI)
// provide function to sync time with RTC time
time_t syncTimeRTC() {
  DateTime now = rtc.now();

  //convert Datetime to time_t
  time_t tt = now.unixtime();

  return tt;
}
#endif

void setup()
{
  //Serial port for debug info
  Serial.begin(57600);
  delay(200);

  //Nextion TFT
  myNex.begin(9600);
  ResetTFT();

  //Initialize Eeprom
  EEPROM.setMemPool(memoryBase, EEPROMSizeMega);

  //Get address of "ConfigVersion" setting
  configAdress = EEPROM.getAddress(sizeof(StoreStruct));

  //Read ConfigVersion. If does not match expected value, restore default values
  uint8_t vers = EEPROM.readByte(configAdress);

  if (vers == CONFIG_VERSION)
  {
    Serial << F("Stored config version: ") << CONFIG_VERSION << F(". Loading settings from eeprom") << _endl;
    loadConfig();//Restore stored values from eeprom
  }
  else
  {
    Serial << F("Stored config version: ") << CONFIG_VERSION << F(". Loading default settings, not from eeprom") << _endl;
    saveConfig();//First time use. Save default values to eeprom
  }

  //Initialize pump objects with stored config data
  PhPump.SetFlowRate(storage.pHPumpFR);
  PhPump.SetTankVolume(storage.pHTankVol);
  ChlPump.SetFlowRate(storage.ChlPumpFR);
  ChlPump.SetTankVolume(storage.ChlTankVol);

  //RTC Stuff (embedded battery operated clock). In case board is MEGA_2560, need to initialize the date time!
#if defined(CONTROLLINO_MAXI)
  Controllino_RTC_init(0);
  setTime((uint8_t)Controllino_GetHour(), (uint8_t)Controllino_GetMinute(), (uint8_t)Controllino_GetSecond(), (uint8_t)Controllino_GetDay(), (uint8_t)Controllino_GetMonth(), (uint8_t)Controllino_GetYear() + 2000);
#else
  if (! rtc.begin())
  {
    Serial << F("Couldn't find RTC") << endl;
    while (1);
  }
  else
  {
    setSyncProvider(&syncTimeRTC);
    //setSyncInterval(30); // in seconds, default 300
  }
#endif

  //Define pins directions
  pinMode(FILTRATION_PUMP, OUTPUT);
  pinMode(PH_PUMP, OUTPUT);
  pinMode(CHL_PUMP, OUTPUT);
  pinMode(HEAT_ON, OUTPUT);

  pinMode(RELAY_R1, OUTPUT);
  pinMode(RELAY_R2, OUTPUT);
  pinMode(RELAY_R6, OUTPUT);
  pinMode(RELAY_R7, OUTPUT);
  pinMode(RELAY_R8, OUTPUT);
  pinMode(RELAY_R9, OUTPUT);

  pinMode(CHL_LEVEL, INPUT_PULLUP);
  pinMode(PH_LEVEL, INPUT_PULLUP);

  pinMode(ORP_MEASURE, INPUT);
  pinMode(PH_MEASURE, INPUT);
  pinMode(PSI_MEASURE, INPUT);


  // initialize Ethernet device
  // if the ip config is the default one, use DHCP to allocate an ip otherwise use the eeprom-stored config
  if (!storage.ipConfiged)
  {
    if (!Ethernet.begin(storage.mac)) //DHCP
    {
      Serial << F("Failed to open ethernet connection through DHCP") << _endl;
    }
  }
  else
  {
    Ethernet.begin(storage.mac, storage.ip, storage.dnsserver, storage.gateway, storage.subnet);
  }
  delay(1500);

  //8 seconds watchdog timer to reset system in case it freezes for more than 8 seconds
  wdt_enable(WDTO_8S);

  // start to listen for clients
  server.begin();

  // Initialize the Bonjour/MDNS library. You can now reach or ping this
  // hardware via the host name "PoolMaster.local", provided that your operating
  // system is Bonjour-enabled (such as MacOS X).
  EthernetBonjour.begin("PoolMaster");

  // Now let's register the service we're offering (a web service) via Bonjour!
  // To do so, we call the addServiceRecord() method. The first argument is the
  // name of our service instance and its type, separated by a dot. In this
  // case, the service type is _http. There are many other service types, use
  // google to look up some common ones, but you can also invent your own
  // service type, like _mycoolservice - As long as your clients know what to
  // look for, you're good to go.
  // The second argument is the port on which the service is running. This is
  // port 80 here, the standard HTTP port.
  // The last argument is the protocol type of the service, either TCP or UDP.
  // Of course, our service is a TCP service.
  // With the service registered, it will show up in a Bonjour-enabled web
  // browser. As an example, if you are using Apple's Safari, you will now see
  // the service under Bookmarks -> Bonjour (Provided that you have enabled
  // Bonjour in the "Bookmarks" preferences in Safari).
  EthernetBonjour.addServiceRecord("PoolMaster Bonjour Webserver._http", 80, MDNSServiceTCP);

  //Start temperature measurement state machine
  gettemp.next(gettemp_start);

#if defined(pHOrpBoard) //using the I2C pHOrpBoard as interface to the pH and Orp probes
  //Initialize ADC library of pH_Orp_Board
  Wire.begin();
  adc.setSpeed(ADS1115_SPEED_8SPS);
#endif

  //start filtration pump at power-on if within scheduled time slots -- You can choose not to do this and start pump manually
  if (storage.AutoMode && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
    FiltrationPump.Start();

  //Init MQTT
  MQTTClient.setOptions(60, false, 6000);
  MQTTClient.setWill(PoolTopicStatus, "offline", true, LWMQTT_QOS1);
  MQTTClient.begin(MqttServerIP, net);
  // MQTTClient.setHost(MqttServerIP, 21883);
  MQTTClient.onMessage(messageReceived);
  MQTTConnect();

  PublishSettings();

  //Initialize PIDs
  storage.PhPIDwindowStartTime = millis();
  storage.OrpPIDwindowStartTime = millis();

  //Limit the PIDs output range in order to limit max. pumps runtime (safety first...)
  PhPID.SetSampleTime(600000);
  PhPID.SetTunings(storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd);
  PhPID.SetOutputLimits(0, 600000);//Whatever happens, don't allow continuous injection of Acid for more than 10mins within a PID Window
  OrpPID.SetSampleTime(300000);
  OrpPID.SetTunings(storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd);
  OrpPID.SetOutputLimits(0, 600000);//Whatever happens, don't allow continuous injection of Chl for more than 10mins within a PID Window

  //let the PIDs off at start
  SetPhPID(false);
  SetOrpPID(false);

  //Initialize pumps
  FiltrationPump.SetMaxUpTime(0); //no runtime limit for the filtration pump
  HeatCirculatorPump.SetMaxUpTime(0); //no runtime limit for the Heating system circulator pump
  PhPump.SetMaxUpTime(storage.PhPumpUpTimeLimit * 1000);
  ChlPump.SetMaxUpTime(storage.ChlPumpUpTimeLimit * 1000);

  //Initialize Filtration schedule
  storage.FiltrationDuration = 12;
  storage.FiltrationStop = storage.FiltrationStart + storage.FiltrationDuration;

  //Ethernet client check loop
  SoftTimer.add(&t1);
  t1.init();

  //Orp regulation loop
  SoftTimer.add(&t2);
  t2.init();

  //PH regulation loop
  SoftTimer.add(&t3);
  t3.init();

  //Publish loop
  SoftTimer.add(&t4);
  t4.init();

  //Generic loop
  SoftTimer.add(&t5);
  t5.init();

  UpdateRTC();

  //display remaining RAM space. For debug
  Serial << F("[memCheck]: ") << freeRam() << F("b") << _endl;

}


//Connect to MQTT broker and subscribe to the PoolTopicAPI topic in order to receive future commands
//then publish the "online" message on the "status" topic. If Ethernet connection is ever lost
//"status" will switch to "offline". Very useful to check that the Arduino is alive and functional
void MQTTConnect()
{
  //MQTTClient.connect(MqttServerClientID);
  MQTTClient.connect(MqttServerClientID, MqttServerLogin, MqttServerPwd);
  /*  int8_t Count=0;
    while (!MQTTClient.connect(MqttServerClientID, MqttServerLogin, MqttServerPwd) && (Count<4))
    {
      Serial<<F(".")<<_endl;
      delay(500);
      Count++;
    }
  */
  if (MQTTClient.connected())
  {
    MQTTConnection = true;

    //String PoolTopicAPI = "Home/Pool/Api";
    //Topic to which send/publish API commands for the Pool controls
    MQTTClient.subscribe(PoolTopicAPI);

    //tell status topic we are online
    if (MQTTClient.publish(PoolTopicStatus, F("online"), true, LWMQTT_QOS1))
      Serial << F("published: Home/Pool/status - online") << _endl;
    else
    {
      Serial << F("Unable to publish on status topic; MQTTClient.lastError() returned: ") << MQTTClient.lastError() << F(" - MQTTClient.returnCode() returned: ") << MQTTClient.returnCode() << _endl;
    }
  }
  else
  {
    Serial << F("Failed to connect to the MQTT broker") << _endl;
    MQTTConnection = false;
  }

}

//MQTT callback
//This function is called when messages are published on the MQTT broker on the PoolTopicAPI topic to which we subscribed
//Add the received command to a message queue for later processing and exit the callback
void messageReceived(String &topic, String &payload)
{
  String TmpStrPool(PoolTopicAPI);

  //Pool commands. This check might be redundant since we only subscribed to this topic
  if (topic == TmpStrPool)
  {
    if (queueIn.enqueue(payload))
    {
      Serial << F("Added command to queue: ") << payload << _endl;
    }
    else
    {
      Serial << F("Could not add command to queue, queue is full") << _endl;
    }
    Serial << F("FreeRam: ") << freeRam() << F(" - Qeued messages: ") << queueIn.itemCount() << _endl;
  }
}

//Loop where various tasks are updated/handled
void GenericCallback(Task* me)
{
  //clear watchdog timer
  wdt_reset();
  //Serial<<F("Watchdog Reset")<<_endl;

  //run the MDNS / Bonjour! module
  EthernetBonjour.run();

  //request temp reading
  gettemp.run();

  //Update MQTT thread
  MQTTClient.loop();

  //UPdate Nextion TFT
  UpdateTFT();

  //update pumps
  HeatCirculatorPump.loop();
  FiltrationPump.loop();
  PhPump.loop();
  ChlPump.loop();

  //Process queued incoming JSON commands if any
  if (queueIn.itemCount() > 0)
    ProcessCommand(queueIn.dequeue());

  //reset time counters at midnight and send sync request to time server
  if ((hour() == 0) && (minute() == 0))
  {
    //First store current Chl and Acid consumptions of the day in Eeprom
    storage.AcidFill = storage.AcidFill - PhPump.GetTankUsage();
    storage.ChlFill = storage.ChlFill - ChlPump.GetTankUsage();
    saveConfig();

    HeatCirculatorPump.ResetUpTime();
    FiltrationPump.ResetUpTime();
    PhPump.ResetUpTime();
    ChlPump.ResetUpTime();

    EmergencyStopFiltPump = false;

    //sync RTC with time server
    UpdateRTC();
  }
  else
  {
    DoneForTheDay = false;
  }

  //compute next Filtering duration and stop time (in hours)
  if ((hour() == storage.FiltrationStart + 1) && (minute() == 0))
  {
    storage.FiltrationDuration = round(storage.TempValue / 2);
    if (storage.FiltrationDuration < 3) storage.FiltrationDuration = 3;
    storage.FiltrationStop = storage.FiltrationStart + storage.FiltrationDuration;
    Serial << F("storage.FiltrationDuration: ") << storage.FiltrationDuration << _endl;
    if (storage.FiltrationStop > storage.FiltrationStopMax)
      storage.FiltrationStop = storage.FiltrationStopMax;
  }

  //start filtration pump as scheduled
  if (!EmergencyStopFiltPump && storage.AutoMode && !PSIError && (hour() == storage.FiltrationStart) && (minute() == 0))
    FiltrationPump.Start();
  PSIError = PSIError;

  //start PIDs with delay after FiltrationStart in order to let the readings stabilize
  if (FiltrationPump.IsRunning() && storage.AutoMode && !PhPID.GetMode() && ((millis() - FiltrationPump.LastStartTime) / 1000 / 60 > storage.DelayPIDs) && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
  {
    //Start PIDs
    SetPhPID(true);
    SetOrpPID(true);
  }

  //If water heating is desired and filtration has been running for over 5mins (so that measured water temp is accurate), open/close the HEAT_ON relay as required
  //in order to regulate the water temp. When closing the HEAT_ON relay, my house heating system switches to a fixed water temperature mode and starts the pool water
  //circulator in order to heat-up the heat exchanger located on the pool filtration water circuit
  if (storage.WaterHeat && FiltrationPump.IsRunning())
  {
    if (FiltrationPump.UpTime / 1000 / 60 > 5)
    {
      if (storage.TempValue < (storage.WaterTemp_SetPoint - 0.2))
      {
        HeatCirculatorPump.Start();
      }
      else if (storage.TempValue > (storage.WaterTemp_SetPoint + 0.2))
      {
        HeatCirculatorPump.Stop();
      }
    }
  }
  else
  {
    HeatCirculatorPump.Stop();
  }


  //The circulator of the pool water heating circuit needs to run regularly to avoid blocking
  //Let it run every day at noon for 2 mins
  if (storage.AutoMode && ((hour() == 12) && (minute() == 0)))
  {
    HeatCirculatorPump.Start();
  }

  if (storage.AutoMode && ((hour() == 12) && (minute() == 2)))
  {
    HeatCirculatorPump.Stop();
  }


  //stop filtration pump and PIDs as scheduled unless we are in AntiFreeze mode
  if (storage.AutoMode && FiltrationPump.IsRunning() && !AntiFreezeFiltering && ((hour() == storage.FiltrationStop) && (minute() == 0)))
  {
    SetPhPID(false);
    SetOrpPID(false);
    FiltrationPump.Stop();
  }

  //Outside regular filtration hours, start filtration in case of cold Air temperatures (<-2.0deg)
  if (!EmergencyStopFiltPump && storage.AutoMode && !PSIError && !FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && (storage.TempExternal < -2.0))
  {
    FiltrationPump.Start();
    AntiFreezeFiltering = true;
  }

  //Outside regular filtration hours and if in AntiFreezeFiltering mode but Air temperature rose back above 2.0deg, stop filtration
  if (storage.AutoMode && FiltrationPump.IsRunning() && ((hour() < storage.FiltrationStart) || (hour() > storage.FiltrationStop)) && AntiFreezeFiltering && (storage.TempExternal > 2.0))
  {
    FiltrationPump.Stop();
    AntiFreezeFiltering = false;
  }

  //If filtration pump has been running for over 7secs but pressure is still low, stop the filtration pump, something is wrong, set error flag
  if (FiltrationPump.IsRunning() && ((millis() - FiltrationPump.LastStartTime) > 7000) && (storage.PSIValue < storage.PSI_MedThreshold))
  {
    FiltrationPump.Stop();
    PSIError = true;
    MQTTClient.publish(PoolTopicError, F("PSI Error"), true, LWMQTT_QOS1);
  }
}

//PublishData loop. Publishes system info/data to MQTT broker every XX secs (30 secs by default)
void PublishDataCallback(Task* me)
{
  //Store the GPIO states in one Byte (more efficient over MQTT)
  EncodeBitmap();

  if (!MQTTClient.connected())
  {
    MQTTConnect();
    //Serial.println("MQTT reconnecting...");
  }

  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(7); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("Tmp")] =  (int)(storage.TempValue * 100);
    root[F("pH")] =  (int)(storage.PhValue * 100);
    root[F("PSI")] =  (int)(storage.PSIValue * 100);
    root[F("Orp")] =  (int)storage.OrpValue;
    root[F("FilUpT")] =  FiltrationPump.UpTime / 1000;
    root[F("PhUpT")] =  PhPump.UpTime / 1000;
    root[F("ChlUpT")] =  ChlPump.UpTime / 1000;

    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicMeas1, Payload, strlen(Payload), true, LWMQTT_QOS1);

  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //Second MQTT publish to limit size of payload at once
  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(6); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("AcidF")] =  (int)(storage.AcidFill - PhPump.GetTankUsage());
    root[F("ChlF")] =  (int)(storage.ChlFill - ChlPump.GetTankUsage());
    root[F("IO")] =  BitMap;
    root[F("IO2")] =  BitMap2;
    root[F("pH2")] =  (int)(storage.PhValue2 * 100);
    root[F("Orp2")] =  (int)storage.OrpValue2;

    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicMeas2, Payload, strlen(Payload), true, LWMQTT_QOS1);
  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //display remaining RAM space. For debug
  Serial << F("[memCheck]: ") << freeRam() << F("b") << _endl;
}


//Publishes system settings to MQTT broker
void PublishSettings()
{
  if (!MQTTClient.connected())
  {
    MQTTConnect();
    //Serial.println("MQTT reconnecting...");
  }

  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(8); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("Fw")] = Firmw;//firmware revision
    root[F( "FSta")] = (uint8_t)storage.FiltrationStart;//Filtration start hour, in the morning (hours)
    root[F( "FDu")] = (uint8_t)storage.FiltrationDuration;//Computed filtration duration based on water temperature (hours)
    root[F( "FStoM")] = (uint8_t)storage.FiltrationStopMax;//Latest hour for the filtration to run. Whatever happens, filtration won't run later than this hour (hour)
    root[F( "FSto")] = (uint8_t)storage.FiltrationStop;//Computed filtration stop hour, equal to FSta + FDu (hour)
    root[F( "Dpid")] = (uint8_t)storage.DelayPIDs;//Delay from FSta for the water regulation/PIDs to start (mins)
    root[F( "pHUTL")] = (uint8_t)(storage.PhPumpUpTimeLimit / 60); //Max allowed daily run time for the pH pump (/!\ mins)
    root[F( "ChlUTL")] = (uint8_t)(storage.ChlPumpUpTimeLimit / 60); //Max allowed daily run time for the Chl pump (/!\ mins)

    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicSet1, Payload, strlen(Payload), true, LWMQTT_QOS1);
  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //Update MQTT thread
  MQTTClient.loop();

  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(8); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("pHWS")] = (uint8_t)(storage.PhPIDWindowSize / 1000 / 60); //pH PID window size (/!\ mins)
    root[F("ChlWS")] = (uint8_t)(storage.OrpPIDWindowSize / 1000 / 60); //Orp PID window size (/!\ mins)
    root[F("pHSP")] = (int)(storage.Ph_SetPoint * 100); //pH setpoint (/!\ x100)
    root[F("OrpSP")] = (int)(storage.Orp_SetPoint);//Orp setpoint
    root[F("WSP")] = (int)(storage.WaterTemp_SetPoint * 100); //Water temperature setpoint (/!\ x100)
    root[F("WLT")] = (int)(storage.WaterTempLowThreshold * 100); //Water temperature low threshold to activate anti-freeze mode (/!\ x100)
    root[F("PSIHT")] = (uint8_t)(storage.PSI_HighThreshold * 100); //Water pressure high threshold to trigger error (/!\ x100)
    root[F("PSIMT")] = (uint8_t)(storage.PSI_MedThreshold * 100); //Water pressure medium threshold (unused yet) (/!\ x100)

    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicSet2, Payload, strlen(Payload), true, LWMQTT_QOS1);
  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //Update MQTT thread
  MQTTClient.loop();

  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(7); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("TE")] = (int)(storage.TempExternal * 100); // /!\ x100
    root[F("pHC0")] = (float)(storage.pHCalibCoeffs0);//pH sensor calibration coefficient C0
    root[F("pHC1")] = (float)(storage.pHCalibCoeffs1);//pH sensor calibration coefficient C1
    root[F("OrpC0")] = (float)(storage.OrpCalibCoeffs0);//Orp sensor calibration coefficient C0
    root[F("OrpC1")] = (float)(storage.OrpCalibCoeffs1);//Orp sensor calibration coefficient C1
    root[F("PSIC0")] = (float)(storage.PSICalibCoeffs0);//Pressure sensor calibration coefficient C0
    root[F("PSIC1")] = (float)(storage.PSICalibCoeffs1);//Pressure sensor calibration coefficient C1

    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicSet3, Payload, strlen(Payload), true, LWMQTT_QOS1);

  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //Update MQTT thread
  MQTTClient.loop();

  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(6); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("pHKp")] = (float)(storage.Ph_Kp);//pH PID coeffcicient Kp
    root[F( "pHKi")] = (float)(storage.Ph_Ki);//pH PID coeffcicient Ki
    root[F( "pHKd")] = (float)(storage.Ph_Kd);//pH PID coeffcicient Kd
    root[F( "OrpKp")] = (float)(storage.Orp_Kp);//Orp PID coeffcicient Kp
    root[F( "OrpKi")] = (float)(storage.Orp_Ki);//Orp PID coeffcicient Ki
    root[F( "OrpKd")] = (float)(storage.Orp_Kd);//Orp PID coeffcicient Kd
    
    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicSet4, Payload, strlen(Payload), true, LWMQTT_QOS1);

  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //Update MQTT thread
  MQTTClient.loop();

  if (MQTTClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 192 bytes
    const int capacity = JSON_OBJECT_SIZE(4); // value recommended by ArduinoJson Assistant with slack for the Firmw string
    StaticJsonDocument<capacity> root;

    root[F("pHTV")] = (uint8_t)storage.pHTankVol;//Acid tank nominal volume (Liters)
    root[F("ChlTV")] = (uint8_t)storage.ChlTankVol;//Chl tank nominal volume (Liters)
    root[F("pHFR")] = (float)(storage.pHPumpFR);//Acid pump flow rate (L/hour)
    root[F("OrpFR")] = (float)(storage.ChlPumpFR);//Chl pump flow rate (L/hour)

    serializeJson(root, Payload);
    MQTTClient.publish(PoolTopicSet5, Payload, strlen(Payload), true, LWMQTT_QOS1);
  }
  else
    Serial << F("Failed to connect to the MQTT broker") << _endl;

  //display remaining RAM space. For debug
  Serial << F("[memCheck]: ") << freeRam() << F("b") << _endl;
}

void PHRegulationCallback(Task * me)
{
  //do not compute PID if filtration pump is not running
  //because if Ki was non-zero that would let the OutputError increase
  if (FiltrationPump.IsRunning()  && (PhPID.GetMode() == AUTOMATIC))
  {
    PhPID.Compute();

    /************************************************
      turn the Acid pump on/off based on pid output
    ************************************************/
    if (millis() - storage.PhPIDwindowStartTime > storage.PhPIDWindowSize)
    {
      //time to shift the Relay Window
      storage.PhPIDwindowStartTime += storage.PhPIDWindowSize;
    }
    if (storage.PhPIDOutput < millis() - storage.PhPIDwindowStartTime)
      PhPump.Stop();
    else
      PhPump.Start();
  }
}

//Orp regulation loop
void OrpRegulationCallback(Task * me)
{
  //do not compute PID if filtration pump is not running
  //because if Ki was non-zero that would let the OutputError increase
  if (FiltrationPump.IsRunning() && (OrpPID.GetMode() == AUTOMATIC))
  {
    OrpPID.Compute();

    /************************************************
      turn the Acid pump on/off based on pid output
    ************************************************/
    if (millis() - storage.OrpPIDwindowStartTime > storage.OrpPIDWindowSize)
    {
      //time to shift the Relay Window
      storage.OrpPIDwindowStartTime += storage.OrpPIDWindowSize;
    }
    if (storage.OrpPIDOutput < millis() - storage.OrpPIDwindowStartTime)
      ChlPump.Stop();
    else
      ChlPump.Start();
  }
}

//Enable/Disable Chl PID
void SetPhPID(bool Enable)
{
  if (Enable)
  {
    //Stop PhPID
    PhPump.ClearErrors();
    storage.PhPIDOutput = 0.0;
    storage.PhPIDwindowStartTime = millis();
    PhPID.SetMode(1);
    storage.Ph_RegulationOnOff = 1;
  }
  else
  {
    //Stop PhPID
    PhPID.SetMode(0);
    storage.Ph_RegulationOnOff = 0;
    storage.PhPIDOutput = 0.0;
    PhPump.Stop();
  }
}

//Enable/Disable Orp PID
void SetOrpPID(bool Enable)
{
  if (Enable)
  {
    //Stop OrpPID
    ChlPump.ClearErrors();
    storage.OrpPIDOutput = 0.0;
    storage.OrpPIDwindowStartTime = millis();
    OrpPID.SetMode(1);
    storage.Orp_RegulationOnOff = 1;

  }
  else
  {
    //Stop OrpPID
    OrpPID.SetMode(0);
    storage.Orp_RegulationOnOff = 0;
    storage.OrpPIDOutput = 0.0;
    ChlPump.Stop();
  }
}

//Encode digital inputs states into one Byte (more efficient to send over MQTT)
void EncodeBitmap()
{
  BitMap = 0;
  BitMap2 = 0;
  BitMap |= (FiltrationPump.IsRunning() & 1) << 7;
  BitMap |= (PhPump.IsRunning() & 1) << 6;
  BitMap |= (ChlPump.IsRunning() & 1) << 5;
  BitMap |= (PhPump.TankLevel() & 1) << 4;
  BitMap |= (ChlPump.TankLevel() & 1) << 3;
  BitMap |= (PSIError & 1) << 2;
  BitMap |= (PhPump.UpTimeError & 1) << 1;
  BitMap |= (ChlPump.UpTimeError & 1) << 0;

  BitMap2 |= (PhPID.GetMode() & 1) << 7;
  BitMap2 |= (OrpPID.GetMode() & 1) << 6;
  BitMap2 |= (storage.AutoMode & 1) << 5;
  BitMap2 |= (storage.WaterHeat & 1) << 4;
  BitMap2 |= (digitalRead(RELAY_R1) & 1) << 3;
  BitMap2 |= (digitalRead(RELAY_R2) & 1) << 2;
  BitMap2 |= (digitalRead(RELAY_R6) & 1) << 1;
  BitMap2 |= (digitalRead(RELAY_R7) & 1) << 0;

}

//Update temperature, Ph and Orp values
void getMeasures(DeviceAddress deviceAddress_0)
{
  Serial << TimeBuffer << F(" - ");

  //Water Temperature
  samples_Temp.add(sensors_A.getTempC(deviceAddress_0));
  storage.TempValue = samples_Temp.getAverage(10);
  if (storage.TempValue == -127.00) {
    Serial << F("Error getting temperature from DS18b20_0") << _endl;
  } else {
    Serial << F("DS18b20_0: ") << storage.TempValue << F("°C") << _endl;
  }
  /*
    //Ph
    float ph_sensor_value = analogRead(PH_MEASURE) * 5.0 / 1023.0;                                        // from 0.0 to 5.0 V
    //storage.PhValue = 7.0 - ((2.5 - ph_sensor_value)/(0.257179 + 0.000941468 * storage.TempValue));     // formula to compute pH which takes water temperature into account
    //storage.PhValue = (0.0178 * ph_sensor_value * 200.0) - 1.889;                                       // formula to compute pH without taking temperature into account (assumes 27deg water temp)
    storage.PhValue = (storage.pHCalibCoeffs0 * ph_sensor_value) + storage.pHCalibCoeffs1;                //Calibrated sensor response based on multi-point linear regression
    samples_Ph.add(storage.PhValue);                                                                      // compute average of pH from last 5 measurements
    storage.PhValue = samples_Ph.getAverage(10);
    Serial << F("Ph: ") << storage.PhValue << F(" - ");

    //ORP
    float orp_sensor_value = analogRead(ORP_MEASURE) * 5.0 / 1023.0;                                      // from 0.0 to 5.0 V
    //storage.OrpValue = ((2.5 - orp_sensor_value) / 1.037) * 1000.0;                                     // from -2000 to 2000 mV where the positive values are for oxidizers and the negative values are for reducers
    storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;            //Calibrated sensor response based on multi-point linear regression
    samples_Orp.add(storage.OrpValue);                                                                    // compute average of ORP from last 5 measurements
    storage.OrpValue = samples_Orp.getAverage(10);
    Serial << F("Orp: ") << orp_sensor_value << " - " << storage.OrpValue << F("mV") << _endl;

    //Ph2 temporary, for testing pH_Orp_Board
    float ph_sensor_value2 = adc.convert(ADS1115_CHANNEL23,ADS1115_RANGE_6144) * 6.144 / 16383.0;          // from 0.0 to 5.0 V
    storage.PhValue2 = (storage.pHCalibCoeffs0 * ph_sensor_value2) + storage.pHCalibCoeffs1;               //Calibrated sensor response based on multi-point linear regression
    samples_Ph2.add(storage.PhValue2);                                                                     // compute average of pH from last 5 measurements
    storage.PhValue2 = samples_Ph2.getAverage(10);
    Serial << F("Ph2: ") << storage.PhValue2 << F(" - ");

    //ORP2 temporary, for testing pH_Orp_Board
    float orp_sensor_value2 = adc.convert(ADS1115_CHANNEL01,ADS1115_RANGE_6144) * 6.144 / 16383.0;         // from 0.0 to 5.0 V
    storage.OrpValue2 = (storage.OrpCalibCoeffs0 * orp_sensor_value2) + storage.OrpCalibCoeffs1;           //Calibrated sensor response based on multi-point linear regression
    samples_Orp2.add(storage.OrpValue2);                                                                   // compute average of ORP from last 5 measurements
    storage.OrpValue2 = samples_Orp2.getAverage(10);
    Serial << F("Orp2: ") << orp_sensor_value2 << " - " << storage.OrpValue2 << F("mV") << _endl;
  */

#if defined(pHOrpBoard) //using the I2C pHOrpBoard as interface to the pH and Orp probes

  //Ph
  float ph_sensor_value = adc.convert(ADS1115_CHANNEL23, ADS1115_RANGE_6144) * 6.144 / 16383.0;         // from 0.0 to 5.0 V
  storage.PhValue = (storage.pHCalibCoeffs0 * -ph_sensor_value) + storage.pHCalibCoeffs1;               //Calibrated sensor response based on multi-point linear regression
  samples_Ph.add(storage.PhValue);                                                                     // compute average of pH from last 5 measurements
  storage.PhValue = samples_Ph.getAverage(2);
  Serial << F("Ph: ") << (2.5 - ph_sensor_value) * 1000.0 << F("mV - ") << storage.PhValue  << _endl;

  //ORP
  float orp_sensor_value = adc.convert(ADS1115_CHANNEL01, ADS1115_RANGE_6144) * 6.144 / 16383.0;        // from 0.0 to 5.0 V
  storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;           //Calibrated sensor response based on multi-point linear regression
  samples_Orp.add(storage.OrpValue);                                                                   // compute average of ORP from last 5 measurements
  storage.OrpValue = samples_Orp.getAverage(2);
  Serial << F("Orp: ") << (2.5 - orp_sensor_value) * 1000.0 << F("mV - ") << storage.OrpValue << F("mV") << _endl;

#else //using the Phidget analog boards (PoolMaster V5.0 and earlier)

  //Ph
  float ph_sensor_value = analogRead(PH_MEASURE) * 5.0 / 1023.0;                                        // from 0.0 to 5.0 V
  //storage.PhValue = 7.0 - ((2.5 - ph_sensor_value)/(0.257179 + 0.000941468 * storage.TempValue));     // formula to compute pH which takes water temperature into account
  //storage.PhValue = (0.0178 * ph_sensor_value * 200.0) - 1.889;                                       // formula to compute pH without taking temperature into account (assumes 27deg water temp)
  storage.PhValue = (storage.pHCalibCoeffs0 * ph_sensor_value) + storage.pHCalibCoeffs1;                //Calibrated sensor response based on multi-point linear regression
  samples_Ph.add(storage.PhValue);                                                                      // compute average of pH from last 5 measurements
  storage.PhValue = samples_Ph.getAverage(10);
  Serial << F("Ph: ") << storage.PhValue << F(" - ");

  //ORP
  float orp_sensor_value = analogRead(ORP_MEASURE) * 5.0 / 1023.0;                                      // from 0.0 to 5.0 V
  //storage.OrpValue = ((2.5 - orp_sensor_value) / 1.037) * 1000.0;                                     // from -2000 to 2000 mV where the positive values are for oxidizers and the negative values are for reducers
  storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;            //Calibrated sensor response based on multi-point linear regression
  samples_Orp.add(storage.OrpValue);                                                                    // compute average of ORP from last 5 measurements
  storage.OrpValue = samples_Orp.getAverage(10);
  Serial << F("Orp: ") << orp_sensor_value << " - " << storage.OrpValue << F("mV") << _endl;

#endif

  //PSI (water pressure)
  float psi_sensor_value = ((analogRead(PSI_MEASURE) * 0.03) - 0.5) * 5.0 / 4.0;                        // from 0.5 to 4.5V -> 0.0 to 5.0 Bar (depends on sensor ref!)                                                                           // Remove this line when sensor is integrated!!!
  storage.PSIValue = (storage.PSICalibCoeffs0 * psi_sensor_value) + storage.PSICalibCoeffs1;            //Calibrated sensor response based on multi-point linear regression
  samples_PSI.add(storage.PSIValue);                                                                    // compute average of PSI from last 5 measurements
  storage.PSIValue = samples_PSI.getAverage(3);
  Serial << F("PSI: ") << psi_sensor_value << " - " << storage.PSIValue << F("Bar") << _endl;
}

bool loadConfig()
{
  EEPROM.readBlock(configAdress, storage);

  Serial << storage.ConfigVersion << '\n';
  Serial << storage.Ph_RegulationOnOff << ", " << storage.Orp_RegulationOnOff << ", " << storage.AutoMode << ", " << storage.WaterHeat << '\n';
  Serial << storage.FiltrationStart << ", " << storage.FiltrationDuration << ", " << storage.FiltrationStopMax << ", " << storage.FiltrationStop << ", " << storage.DelayPIDs << '\n';
  Serial << storage.PhPumpUpTimeLimit << ", " << storage.ChlPumpUpTimeLimit << '\n';
  //  Serial<<storage.FiltrationPumpTimeCounter<<", "<<storage.PhPumpTimeCounter<<", "<<storage.ChlPumpTimeCounter<<", "<<storage.FiltrationPumpTimeCounterStart<<", "<<storage.PhPumpTimeCounterStart<<", "<<storage.ChlPumpTimeCounterStart<<'\n';
  Serial << storage.PhPIDWindowSize << ", " << storage.OrpPIDWindowSize << ", " << storage.PhPIDwindowStartTime << ", " << storage.OrpPIDwindowStartTime << '\n';
  Serial << storage.Ph_SetPoint << ", " << storage.Orp_SetPoint << ", " << storage.PSI_HighThreshold << ", " << storage.PSI_MedThreshold << ", " << storage.WaterTempLowThreshold << ", " << storage.WaterTemp_SetPoint << ", " << storage.TempExternal << ", " << storage.pHCalibCoeffs0 << ", " << storage.pHCalibCoeffs1 << ", " << storage.OrpCalibCoeffs0 << ", " << storage.OrpCalibCoeffs1 << ", " << storage.PSICalibCoeffs0 << ", " << storage.PSICalibCoeffs1 << '\n';
  Serial << storage.Ph_Kp << ", " << storage.Ph_Ki << ", " << storage.Ph_Kd << ", " << storage.Orp_Kp << ", " << storage.Orp_Ki << ", " << storage.Orp_Kd << ", " << storage.PhPIDOutput << ", " << storage.OrpPIDOutput << ", " << storage.TempValue << ", " << storage.PhValue << ", " << storage.OrpValue << ", " << storage.PSIValue << '\n';
  Serial << storage.AcidFill << ", " << storage.ChlFill << ", " << storage.pHTankVol << ", " << storage.ChlTankVol << ", " << storage.pHPumpFR << ", " << storage.ChlPumpFR << '\n';
  Serial << storage.ip[0] << "." << storage.ip[1] << "." << storage.ip[2] << "." << storage.ip[3] << ", " << storage.subnet[0] << "." << storage.subnet[1] << "." << storage.subnet[2] << "." << storage.subnet[3] << ", " << storage.gateway[0] << "." << storage.gateway[1] << "." << storage.gateway[2] << "." << storage.gateway[3] << ", " << storage.dnsserver[0] << "." << storage.dnsserver[1] << "." << storage.dnsserver[2] << "." << storage.dnsserver[3] << ", " << _HEX(storage.mac[0]) << "."  << _HEX(storage.mac[1]) << "." << _HEX(storage.mac[2]) << "." << _HEX(storage.mac[3]) << "." << _HEX(storage.mac[4]) << "." << _HEX(storage.mac[5]) << '\n';
  Serial << storage.ipConfiged << '\n' << '\n';

  return (storage.ConfigVersion == CONFIG_VERSION);
}

void saveConfig()
{
  //update function only writes to eeprom if the value is actually different. Increases the eeprom lifetime
  EEPROM.writeBlock(configAdress, storage);
}

void ProcessCommand(String JSONCommand)
{
   StaticJsonDocument<200> command;

  //Parse Json object and find which command it is
  //JsonObject& command = doc.parseObject(JSONCommand);
  DeserializationError error = deserializeJson(command, JSONCommand);

  // Test if parsing succeeds.
  if (error)
  {
    Serial << F("Json parseObject() failed: ") << error.f_str() << endl;
    return;
  }
  else
  {
    Serial << F("Json parseObject() success - ") << endl;
    DEBUG_PRINT(JSONCommand);

    //Provide the external temperature. Should be updated regularly and will be used to start filtration for 10mins every hour when temperature is negative
    if (command.containsKey(F("TempExt")))
    {
      storage.TempExternal = (float)command[F("TempExt")];
      Serial << F("External Temperature: ") << storage.TempExternal << F("deg") << endl;
    }
    else
      //"PhCalib" command which computes and sets the calibration coefficients of the pH sensor response based on a multi-point linear regression
      //{"PhCalib":[4.02,3.8,9.0,9.11]}  -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
      if (command.containsKey(F("PhCalib")))
      {
        float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough
        //int NbPoints = command[F("PhCalib")].as<JsonArray>().copyTo(CalibPoints);
        int NbPoints = (int)copyArray(command[F("PhCalib")].as<JsonArray>(),CalibPoints); 
        Serial << F("PhCalib command - ") << NbPoints << F(" points received: ");
        for (int i = 0; i < NbPoints; i += 2)
          Serial << CalibPoints[i] << F(",") << CalibPoints[i + 1] << F(" - ");
        Serial << _endl;

        if (NbPoints == 2) //Only one pair of points. Perform a simple offset calibration
        {
          Serial << F("2 points. Performing a simple offset calibration") << _endl;

          //compute offset correction
          storage.pHCalibCoeffs1 += CalibPoints[1] - CalibPoints[0];

          //Set slope back to default value
          storage.pHCalibCoeffs0 = 3.76;

          //Store the new coefficients in eeprom
          saveConfig();
          PublishSettings();
          Serial << F("Calibration completed. Coeffs are: ") << storage.pHCalibCoeffs0 << F(",") << storage.pHCalibCoeffs1 << _endl;
        }
        else if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
        {
          Serial << NbPoints / 2 << F(" points. Performing a linear regression calibration") << _endl;

          float xCalibPoints[NbPoints / 2];
          float yCalibPoints[NbPoints / 2];

          //generate array of x sensor values (in volts) and y rated buffer values
          //storage.PhValue = (storage.pHCalibCoeffs0 * ph_sensor_value) + storage.pHCalibCoeffs1;
          for (int i = 0; i < NbPoints; i += 2)
          {
            xCalibPoints[i / 2] = (CalibPoints[i] - storage.pHCalibCoeffs1) / storage.pHCalibCoeffs0;
            yCalibPoints[i / 2] = CalibPoints[i + 1];
          }

          //Compute linear regression coefficients
          simpLinReg(xCalibPoints, yCalibPoints, storage.pHCalibCoeffs0, storage.pHCalibCoeffs1, NbPoints / 2);

          //Store the new coefficients in eeprom
          saveConfig();
          PublishSettings();
          Serial << F("Calibration completed. Coeffs are: ") << storage.pHCalibCoeffs0 << F(",") << storage.pHCalibCoeffs1 << _endl;
        }
      }
      else
        //"OrpCalib" command which computes and sets the calibration coefficients of the Orp sensor response based on a multi-point linear regression
        //{"OrpCalib":[450,465,750,784]}   -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
        if (command.containsKey(F("OrpCalib")))
        {
          float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough
          //int NbPoints = command[F("OrpCalib")].as<JsonArray>().copyTo(CalibPoints);
          int NbPoints = (int)copyArray(command[F("OrpCalib")].as<JsonArray>(),CalibPoints);
          Serial << F("OrpCalib command - ") << NbPoints << F(" points received: ");
          for (int i = 0; i < NbPoints; i += 2)
            Serial << CalibPoints[i] << F(",") << CalibPoints[i + 1] << F(" - ");
          Serial << _endl;

          if (NbPoints == 2) //Only one pair of points. Perform a simple offset calibration
          {
            Serial << F("2 points. Performing a simple offset calibration") << _endl;

            //compute offset correction
            storage.OrpCalibCoeffs1 += CalibPoints[1] - CalibPoints[0];

            //Set slope back to default value
            storage.OrpCalibCoeffs0 = -1000;

            //Store the new coefficients in eeprom
            saveConfig();
            PublishSettings();
            Serial << F("Calibration completed. Coeffs are: ") << storage.OrpCalibCoeffs0 << F(",") << storage.OrpCalibCoeffs1 << _endl;
          }
          else if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
          {
            Serial << NbPoints / 2 << F(" points. Performing a linear regression calibration") << _endl;

            float xCalibPoints[NbPoints / 2];
            float yCalibPoints[NbPoints / 2];

            //generate array of x sensor values (in volts) and y rated buffer values
            //storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;
            for (int i = 0; i < NbPoints; i += 2)
            {
              xCalibPoints[i / 2] = (CalibPoints[i] - storage.OrpCalibCoeffs1) / storage.OrpCalibCoeffs0;
              yCalibPoints[i / 2] = CalibPoints[i + 1];
            }

            //Compute linear regression coefficients
            simpLinReg(xCalibPoints, yCalibPoints, storage.OrpCalibCoeffs0, storage.OrpCalibCoeffs1, NbPoints / 2);

            //Store the new coefficients in eeprom
            saveConfig();
            PublishSettings();
            Serial << F("Calibration completed. Coeffs are: ") << storage.OrpCalibCoeffs0 << F(",") << storage.OrpCalibCoeffs1 << _endl;
          }
        }
        else
          //"PSICalib" command which computes and sets the calibration coefficients of the Electronic Pressure sensor response based on a linear regression and a reference mechanical sensor (typically located on the sand filter)
          //{"PSICalib":[0,0,0.71,0.6]}   -> multi-point linear regression calibration (minimum 2 point-couple, 6 max.) in the form [ElectronicPressureSensorReading_0, MechanicalPressureSensorReading_0, xx, xx, ElectronicPressureSensorReading_n, ElectronicPressureSensorReading_n]
          if (command.containsKey(F("PSICalib")))
          {
            float CalibPoints[12];//Max six calibration point-couples! Should be plenty enough, typically use two point-couples (filtration ON and filtration OFF)
            //int NbPoints = command[F("PSICalib")].as<JsonArray>().copyTo(CalibPoints);
            int NbPoints = (int)copyArray(command[F("PSICalib")].as<JsonArray>(),CalibPoints);
            Serial << F("PSICalib command - ") << NbPoints << F(" points received: ");
            for (int i = 0; i < NbPoints; i += 2)
              Serial << CalibPoints[i] << F(",") << CalibPoints[i + 1] << F(" - ");
            Serial << _endl;

            if ((NbPoints > 3) && (NbPoints % 2 == 0)) //we have at least 4 points as well as an even number of points. Perform a linear regression calibration
            {
              Serial << NbPoints / 2 << F(" points. Performing a linear regression calibration") << _endl;

              float xCalibPoints[NbPoints / 2];
              float yCalibPoints[NbPoints / 2];

              //generate array of x sensor values (in volts) and y rated buffer values
              //storage.OrpValue = (storage.OrpCalibCoeffs0 * orp_sensor_value) + storage.OrpCalibCoeffs1;
              //storage.PSIValue = (storage.PSICalibCoeffs0 * psi_sensor_value) + storage.PSICalibCoeffs1;
              for (int i = 0; i < NbPoints; i += 2)
              {
                xCalibPoints[i / 2] = (CalibPoints[i] - storage.PSICalibCoeffs1) / storage.PSICalibCoeffs0;
                yCalibPoints[i / 2] = CalibPoints[i + 1];
              }

              //Compute linear regression coefficients
              simpLinReg(xCalibPoints, yCalibPoints, storage.PSICalibCoeffs0, storage.PSICalibCoeffs1, NbPoints / 2);

              //Store the new coefficients in eeprom
              saveConfig();
              PublishSettings();
              Serial << F("Calibration completed. Coeffs are: ") << storage.PSICalibCoeffs0 << F(",") << storage.PSICalibCoeffs1 << _endl;
            }
          }
          else //"Mode" command which sets regulation and filtration to manual or auto modes
            if (command.containsKey(F("Mode")))
            {
              if ((int)command[F("Mode")] == 0)
              {
                storage.AutoMode = 0;

                //Stop PIDs
                SetPhPID(false);
                SetOrpPID(false);
              }
              else
              {
                storage.AutoMode = 1;
              }
              saveConfig();
            }
            else //"Heat" command which starts/stops water heating
              if (command.containsKey(F("Heat")))
              {
                if ((int)command[F("Heat")] == 0)
                {
                  storage.WaterHeat = false;
                  HeatCirculatorPump.Stop();
                }
                else
                {
                  storage.WaterHeat = true;
                }
                saveConfig();
              }
              else if (command.containsKey(F("FiltPump"))) //"FiltPump" command which starts or stops the filtration pump
              {
                if ((int)command[F("FiltPump")] == 0)
                {
                  EmergencyStopFiltPump = true;
                  FiltrationPump.Stop();  //stop filtration pump

                  //Start PIDs
                  SetPhPID(false);
                  SetOrpPID(false);
                }
                else
                {
                  EmergencyStopFiltPump = false;
                  FiltrationPump.Start();   //start filtration pump
                }
              }
              else if (command.containsKey(F("PhPump"))) //"PhPump" command which starts or stops the Acid pump
              {
                if ((int)command[F("PhPump")] == 0)
                  PhPump.Stop();          //stop Acid pump
                else
                  PhPump.Start();           //start Acid pump
              }
              else if (command.containsKey(F("ChlPump"))) //"ChlPump" command which starts or stops the Acid pump
              {
                if ((int)command[F("ChlPump")] == 0)
                  ChlPump.Stop();          //stop Chl pump
                else
                  ChlPump.Start();           //start Chl pump
              }
              else if (command.containsKey(F("PhPID"))) //"PhPID" command which starts or stops the Ph PID loop
              {
                if ((int)command[F("PhPID")] == 0)
                {
                  //Stop PID
                  SetPhPID(false);
                }
                else
                {
                  //Initialize PIDs StartTime
                  storage.PhPIDwindowStartTime = millis();
                  storage.OrpPIDwindowStartTime = millis();

                  //Start PID
                  SetPhPID(true);
                }
              }
              else if (command.containsKey(F("OrpPID"))) //"OrpPID" command which starts or stops the Orp PID loop
              {
                if ((int)command[F("OrpPID")] == 0)
                {
                  //Stop PID
                  SetOrpPID(false);
                }
                else
                {
                  //Start PID
                  SetOrpPID(true);
                }
              }
              else if (command.containsKey(F("PhSetPoint"))) //"PhSetPoint" command which sets the setpoint for Ph
              {
                storage.Ph_SetPoint = (float)command[F("PhSetPoint")];
                saveConfig();
                PublishSettings();
              }
              else if (command.containsKey(F("OrpSetPoint"))) //"OrpSetPoint" command which sets the setpoint for ORP
              {
                storage.Orp_SetPoint = (float)command[F("OrpSetPoint")];
                saveConfig();
                PublishSettings();
              }
              else if (command.containsKey(F("WSetPoint"))) //"WSetPoint" command which sets the setpoint for Water temp (currently not in use)
              {
                storage.WaterTemp_SetPoint = (float)command[F("WSetPoint")];
                saveConfig();
                PublishSettings();
              }
              else
                //"pHTank" command which is called when the pH tank is changed or refilled
                //First parameter is volume of tank in Liters, second parameter is percentage Fill of the tank (typically 100% when new)
                if (command.containsKey(F("pHTank")))
                {
                  PhPump.SetTankVolume((float)command[F("pHTank")][0]);
                  storage.AcidFill = (float)command[F("pHTank")][1];
                  PhPump.ResetUpTime();
                  saveConfig();
                  PublishSettings();
                }
                else
                  //"ChlTank" command which is called when the Chl tank is changed or refilled
                  //First parameter is volume of tank in Liters, second parameter is percentage Fill of the tank (typically 100% when new)
                  if (command.containsKey(F("ChlTank")))
                  {
                    ChlPump.SetTankVolume((float)command[F("ChlTank")][0]);
                    storage.ChlFill = (float)command[F("ChlTank")][1];
                    ChlPump.ResetUpTime();
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("WTempLow"))) //"WTempLow" command which sets the setpoint for Water temp low threshold
                  {
                    storage.WaterTempLowThreshold = (float)command[F("WTempLow")];
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("PumpsMaxUp"))) //"PumpsMaxUp" command which sets the Max UpTime for pumps
                  {
                    storage.PhPumpUpTimeLimit = (unsigned int)command[F("PumpsMaxUp")];
                    PhPump.SetMaxUpTime(storage.PhPumpUpTimeLimit * 1000);
                    storage.ChlPumpUpTimeLimit = (unsigned int)command[F("PumpsMaxUp")];
                    ChlPump.SetMaxUpTime(storage.ChlPumpUpTimeLimit * 1000);
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("OrpPIDParams"))) //"OrpPIDParams" command which sets the Kp, Ki and Kd values for Orp PID loop
                  {
                    storage.Orp_Kp = (double)command[F("OrpPIDParams")][0];
                    storage.Orp_Ki = (double)command[F("OrpPIDParams")][1];
                    storage.Orp_Kd = (double)command[F("OrpPIDParams")][2];
                    saveConfig();
                    OrpPID.SetTunings(storage.Orp_Kp, storage.Orp_Ki, storage.Orp_Kd);
                    PublishSettings();
                  }
                  else if (command.containsKey(F("PhPIDParams"))) //"PhPIDParams" command which sets the Kp, Ki and Kd values for Ph PID loop
                  {
                    storage.Ph_Kp = (double)command[F("PhPIDParams")][0];
                    storage.Ph_Ki = (double)command[F("PhPIDParams")][1];
                    storage.Ph_Kd = (double)command[F("PhPIDParams")][2];
                    saveConfig();
                    PhPID.SetTunings(storage.Ph_Kp, storage.Ph_Ki, storage.Ph_Kd);
                    PublishSettings();
                  }
                  else if (command.containsKey(F("OrpPIDWSize"))) //"OrpPIDWSize" command which sets the window size of the Orp PID loop
                  {
                    storage.OrpPIDWindowSize = (unsigned long)command[F("OrpPIDWSize")];
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("PhPIDWSize"))) //"PhPIDWSize" command which sets the window size of the Ph PID loop
                  {
                    storage.PhPIDWindowSize = (unsigned long)command[F("PhPIDWSize")];
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("Date"))) //"Date" command which sets the Date of RTC module
                  {
#if defined(CONTROLLINO_MAXI)
                    Controllino_SetTimeDate((uint8_t)command[F("Date")][0], (uint8_t)command[F("Date")][1], (uint8_t)command[F("Date")][2], (uint8_t)command[F("Date")][3], (uint8_t)command[F("Date")][4], (uint8_t)command[F("Date")][5], (uint8_t)command[F("Date")][6]); // set initial values to the RTC chip. (Day of the month, Day of the week, Month, Year, Hour, Minute, Second)
#else //Mega2560 board specifics
                    // This line sets the RTC with an explicit date & time, for example to set
                    // January 21, 2014 at 3am you would call:
                    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
                    rtc.adjust(DateTime((uint8_t)command[F("Date")][3], (uint8_t)command[F("Date")][2], (uint8_t)command[F("Date")][0], (uint8_t)command[F("Date")][4], (uint8_t)command[F("Date")][5], (uint8_t)command[F("Date")][6]));
#endif


                    setTime((uint8_t)command[F("Date")][4], (uint8_t)command[F("Date")][5], (uint8_t)command[F("Date")][6], (uint8_t)command[F("Date")][0], (uint8_t)command[F("Date")][2], (uint8_t)command[F("Date")][3]); //(Day of the month, Day of the week, Month, Year, Hour, Minute, Second)
                  }
                  else if (command.containsKey(F("FiltT0"))) //"FiltT0" command which sets the earliest hour when starting Filtration pump
                  {
                    storage.FiltrationStart = (unsigned int)command[F("FiltT0")];
                    storage.FiltrationDuration = round(storage.TempValue / 2);
                    if (storage.FiltrationDuration < 3) storage.FiltrationDuration = 3;
                    storage.FiltrationStop = storage.FiltrationStart + storage.FiltrationDuration;
                    if (storage.FiltrationStop > storage.FiltrationStopMax)
                      storage.FiltrationStop = storage.FiltrationStopMax;
                    Serial << F("storage.FiltrationStart: ") << storage.FiltrationStart << _endl;
                    Serial << F("storage.FiltrationStop: ") << storage.FiltrationStop << _endl;
                    Serial << F("storage.FiltrationDuration: ") << storage.FiltrationDuration << _endl;

                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("FiltT1"))) //"FiltT1" command which sets the latest hour for running Filtration pump
                  {
                    storage.FiltrationStopMax = (unsigned int)command[F("FiltT1")];
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("URTC"))) //"URTC" command which updates RTC date/Time
                  {
                    UpdateRTC();
                  }
                  else if (command.containsKey(F("PubPeriod"))) //"PubPeriod" command which sets the periodicity for publishing system info to MQTT broker
                  {
                    PublishPeriod = (unsigned long)command[F("PubPeriod")] * 1000; //in secs
                    t4.setPeriodMs(PublishPeriod); //in msecs
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("Clear"))) //"Clear" command which clears the UpTime and pressure errors of the Pumps
                  {
                    if (PSIError)
                      PSIError = false;

                    if (PhPump.UpTimeError)
                      PhPump.ClearErrors();

                    if (ChlPump.UpTimeError)
                      ChlPump.ClearErrors();

                    MQTTClient.publish(PoolTopicError, "", true, LWMQTT_QOS1);

                    //start filtration pump if within scheduled time slots
                    if (!EmergencyStopFiltPump && storage.AutoMode && (hour() >= storage.FiltrationStart) && (hour() < storage.FiltrationStop))
                      FiltrationPump.Start();
                  }
                  else if (command.containsKey(F("DelayPID"))) //"DelayPID" command which sets the delay from filtering start before PID loops start regulating
                  {
                    storage.DelayPIDs = (unsigned int)command[F("DelayPID")];
                    saveConfig();
                    PublishSettings();
                  }
                  else if (command.containsKey(F("PSIHigh"))) //"PSIHigh" command which sets the water high-pressure threshold
                  {
                    storage.PSI_HighThreshold = (float)command[F("PSIHigh")];
                    saveConfig();
                    PublishSettings();
                  }
                  else
                    //"Relay" command which is called to actuate relays from the CONTROLLINO.
                    //Parameter 1 is the relay number (R0 in this example), parameter 2 is the relay state (ON in this example).
                    if (command.containsKey(F("Relay")))
                    {
                      switch ((int)command[F("Relay")][0])
                      {
                        case 1:
                          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R1, true) : digitalWrite(RELAY_R1, false);
                          break;
                        case 2:
                          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R2, true) : digitalWrite(RELAY_R2, false);
                          break;
                        case 6:
                          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R6, true) : digitalWrite(RELAY_R6, false);
                          break;
                        case 7:
                          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R7, true) : digitalWrite(RELAY_R7, false);
                          break;
                        case 8:
                          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R8, true) : digitalWrite(RELAY_R8, false);
                          break;
                        case 9:
                          (bool)command[F("Relay")][1] ? digitalWrite(RELAY_R9, true) : digitalWrite(RELAY_R9, false);
                          break;
                      }
                    }
                    else if (command.containsKey(F("Reboot")))//"Reboot" command forces a reboot of the controller
                    {
                      while (1);
                    }
                    else if (command.containsKey(F("pHPumpFR")))//"PhPumpFR" set flow rate of Ph pump
                    {
                      storage.pHPumpFR = (float)command[F("pHPumpFR")];
                      PhPump.SetFlowRate((float)command[F("pHPumpFR")]);
                      saveConfig();
                      PublishSettings();
                    }
                    else if (command.containsKey(F("ChlPumpFR")))//"ChlPumpFR" set flow rate of Chl pump
                    {
                      storage.ChlPumpFR = (float)command[F("ChlPumpFR")];
                      ChlPump.SetFlowRate((float)command[F("ChlpumpFR")]);
                      saveConfig();
                      PublishSettings();
                    }
                    else if (command.containsKey(F("RstpHCal")))//"RstpHCal" reset the calibration coefficients of the pH probe
                    {
                      storage.pHCalibCoeffs0 = 4.3;
                      storage.pHCalibCoeffs1 = -2.63;
                      saveConfig();
                      PublishSettings();
                    }
                    else if (command.containsKey(F("RstOrpCal")))//"RstOrpCal" reset the calibration coefficients of the Orp probe
                    {
                      storage.OrpCalibCoeffs0 = -1189;
                      storage.OrpCalibCoeffs1 = 2564;
                      saveConfig();
                      PublishSettings();
                    }
                    else if (command.containsKey(F("RstPSICal")))//"RstPSICal" reset the calibration coefficients of the pressure sensor
                    {
                      storage.PSICalibCoeffs0 = 1.11;
                      storage.PSICalibCoeffs1 = 0;
                      saveConfig();
                      PublishSettings();
                    }

    //Publish/Update on the MQTT broker the status of our variables
    PublishDataCallback(NULL);
  }
}

//Compute free RAM
//useful to check if it does not shrink over time
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


////////////////////////gettemp state machine///////////////////////////////////
//Init DS18B20 one-wire library
void gettemp_start()
{
  //String containing MAC address of temp sensor to be written to XML file
  sDS18b20_0 = F("<DS18b20_0 Mac='0x28, 0x92, 0x25, 0x41, 0x0A, 0x00, 0x00, 0xEE'>");

  // Start up the library
  sensors_A.begin();

  // set the resolution
  sensors_A.setResolution(DS18b20_0, TEMPERATURE_RESOLUTION);

  //don't wait ! Asynchronous mode
  sensors_A.setWaitForConversion(false);

  gettemp.next(gettemp_request);
}

//Request temperature asynchronously
void gettemp_request()
{
  sensors_A.requestTemperatures();
  gettemp.next(gettemp_wait);
}

//Wait asynchronously for requested temperature measurement
void gettemp_wait()
{ //we need to wait that time for conversion to finish
  if (gettemp.elapsed(1000 / (1 << (12 - TEMPERATURE_RESOLUTION))))
    gettemp.next(gettemp_read);
}

//read and print temperature measurement
void gettemp_read()
{
  sprintf(TimeBuffer, "%d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());
  getMeasures(DS18b20_0);
  gettemp.next(gettemp_request);
}

//Linear regression coefficients calculation function
// pass x and y arrays (pointers), lrCoef pointer, and n.
//The lrCoef array is comprised of the slope=lrCoef[0] and intercept=lrCoef[1].  n is the length of the x and y arrays.
//http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
void simpLinReg(float * x, float * y, double & lrCoef0, double & lrCoef1, int n)
{
  // initialize variables
  float xbar = 0;
  float ybar = 0;
  float xybar = 0;
  float xsqbar = 0;

  // calculations required for linear regression
  for (int i = 0; i < n; i++)
  {
    xbar += x[i];
    ybar += y[i];
    xybar += x[i] * y[i];
    xsqbar += x[i] * x[i];
  }

  xbar /= n;
  ybar /= n;
  xybar /= n;
  xsqbar /= n;

  // simple linear regression algorithm
  lrCoef0 = (xybar - xbar * ybar) / (xsqbar - xbar * xbar);
  lrCoef1 = ybar - lrCoef0 * xbar;
}
