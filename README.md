# PoolMaster

Arduino/Controllino-Maxi (ATmega2560) based Ph/ORP regulator for home pool sysem. No warranty, use at your own risk
(c) Loic74 <loic74650@gmail.com> 2018

***Compatibility***
For this sketch to work on your setup you must change the following in your code:
- possibly the pinout definitions in case you are not using a CONTROLLINO MAXI board
- the code related to the RTC module in case your setup does not have one
- MAC address of DS18b20 water temperature sensor
- MAC and IP address of the Ethernet shield
- MQTT broker IP address and login credentials
- possibly the topic names on the MQTT broker to subscribe and publish to
- the Kp,Ki,Kd parameters for both PID loops in case your peristaltic pumps have a different throughput than 2Liters/hour. For instance, if twice more, divide the parameters by 2

***Brief description:***
Three main metrics are measured and periodically reported over MQTT: water temperature, PH and ORP values
Pumps states, tank-level states and other parmaters are also periodically reported
Two PID regulation loops are running in parallel: one for PH, one for ORP
PH is regulated by injecting Acid from a tank into the pool water (a relay starts/stops the Acid peristaltic pump)
ORP is regulated by injecting Chlorine from a tank into the pool water (a relay starts/stops the Chlorine peristaltic pump)
Defined time-slots and water temperature are used to start/stop the filtration pump for a daily given amount of time (a relay starts/stops the filtration pump) 
A lightweight webserver provides a simple dynamic webpage with a summary of all system parameters. An XML file with more info is available at http://Your_ARDUINO_LOCAL_IP/Info
Communication with the system is performed using the MQTT protocol over an Ethernet connection to the local network/MQTT broker.

Every 30 seconds (by default), the system will publish on the "PoolTopic" (see in code below) the following payloads in Json format:

Temp: measured Water temperature value in °C
Ph: measured Ph value
PhError/100: Ph PID regulation loop instantaneous error
Orp: measured Orp (Redox) value
OrpError/100: Orp PID regulation loop instantaneous error 
FiltUpTime: current running time of Filtration pump in seconds (reset every 24h)
PhUpTime: current running time of Ph pump in seconds (reset every 24h)
ChlUpTime: current running time of Chl pump in seconds (reset every 24h)
IO: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are (LSB first):

FiltPump: current state of Filtration Pump (0=on, 1=off)
PhPump: current state of Ph Pump (0=on, 1=off)
ChlPump: current state of Chl Pump (0=on, 1=off)
PhlLevel: current state of Acid tank level (0=empty, 1=ok)
ChlLevel: current state of Chl tank level (0=empty, 1=ok)
Mode: (0=manual, 1=auto). 
pHErr: pH pump overtime error flag
ChlErr: Chl pump overtime error flag

<p align="center">
  <img src="/docs/PoolMaster.jpg" width="350" title="Overview">
</p>
   
***MQTT API***
Below are the Payloads/commands to publish on the "PoolTopicAPI" topic (see in code below) in Json format in order to launch actions on the Arduino:

{"Mode":1} or {"Mode":0}         -> set "Mode" to manual (0) or Auto (1). In Auto, filtration starts/stops at set times of the day and pH and Orp are regulated 
{"FiltPump":1} or {"FiltPump":0} -> manually start/stop the filtration pump. 
{"ChlPump":1} or {"ChlPump":0}   -> manually start/stop the Chl pump to add more Chlorine
{"PhPump":1} or {"PhPump":0}     -> manually start/stop the Acid pump to lower the Ph
{"PhPID":1} or {"PhPID":0}       -> start/stop the Ph PID regulation loop
{"OrpPID":1} or {"OrpPID":0}     -> start/stop the Orp PID regulation loop
{"PhOffset":0.93}                -> calibrate Ph reading by adding an offset, 0.93 in this example
{"OrpOffset":-185.0}             -> calibrate Orp reading by adding an offset, -185.0mV in this example
{"PhSetPoint":7.4}               -> set the Ph setpoint, 7.4 in this example
{"OrpSetPoint":750.0}            -> set the Orp setpoint, 750mV in this example
{"WSetPoint":27.0}               -> set the water temperature setpoint, 27.0deg in this example (for future use. Water heating not handled yet)
{"WTempLow":10.0}                -> set the water low-temperature threshold below which there is no need to regulate Orp and Ph (ie. in winter)
{"OrpPIDParams":[2857,0,0]}      -> respectively set Kp,Ki,Kd parameters of the Orp PID loop. In this example they are set to 2857, 0 and 0
{"PhPIDParams":[1330000,0,0.0]}  -> respectively set Kp,Ki,Kd parameters of the Ph PID loop. In this example they are set to 1330000, 0 and 0.0
{"OrpPIDWSize":600000}           -> set the window size of the Orp PID loop in msec, 10mins in this example
{"PhPIDWSize":600000}            -> set the window size of the Ph PID loop in msec, 10mins in this example
{"Date":[1,1,1,18,13,32,0]}      -> set date/time of RTC module in the following format: (Day of the month, Day of the week, Month, Year, Hour, Minute, Seconds), in this example: Monday 1st January 2018 - 13h32mn00secs
{"FiltT0":9}                     -> set the earliest hour (9:00 in this example) to run filtration pump. Filtration pump will not run beofre that hour
{"FiltT1":20}                    -> set the latest hour (20:00 in this example) to run filtration pump. Filtration pump will not run after that hour
{"PubPeriod":30}                 -> set the periodicity (in seconds) at which the system info (pumps states, tank levels states, measured values, etc) will be published to the MQTT broker
{"PumpsMaxUp":1800}              -> set the Max Uptime (in secs) for the Ph and Chl pumps over a 24h period. If over, PID regulation is stopped and a warning flag is raised
{"Clear":1}                      -> reset the pH and Orp pumps overtime error flags in order to let the regulation loops continue. "Mode" also needs to be switched back to Auto (1) after an error flag was raised


***Hardware***
CONTROLLINO MAXI (ATmega2560): https://www.controllino.biz/controllino-maxi/maxi-and-maxi-pure/
Phidgets PH/ORB amplifier modules: https://www.phidgets.com/docs/1130_User_Guide#Measuring_the_pH
Galvanic isolator for the pH and Orp probes https://www.dfrobot.com/product-1621.html
Peristaltic pumps: SEKO PR 4
Water grounding: http://electrolyseur.fr/pool-terre.html
Water temperature probe (DS18B20): http://electrolyseur.fr/kit-sonde-DS18B20-filtration-piscine.html


***Links and references***
Webserver code inspired by this blog: https://startingelectronics.org/tutorials/arduino/ethernet-shield-web-server-tutorial/
Linear regression code: http://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
FreeRAM computation code: http://jeelabs.org/2011/05/22/atmega-memory-use/



