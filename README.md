<h2>PoolMaster 5.0.0</h2>
<h2>Arduino Mega2560 (or Controllino-Maxi) Ph/Orp (Chlorine) regulation system for home pools</h2>

<br />
<p align="center"> <img src="/docs/PoolMaster_2.jpg" width="802" title="Overview"> </p> <br /><br />

<br />
<p align="center"> <img src="/docs/Grafana.png" width="802" title="Dashboard"> </p> <br /><br />

<br />
<p align="center"> <img src="/docs/Nextion.png" width="802" title="Nextion 3.5" touch screen"> </p> <br /><br />

<h4>Brief description</h4>
	
<p>Four main metrics are measured and periodically reported over MQTT and a 3.5" Nextion touch screen: water temperature and pressure, pH and ORP values.<br />
Pumps states, tank-levels estimates and other parameters are also periodically reported<br />
Two PID regulation loops are running in parallel: one for PH, one for ORP<br />
An additional simple (on/off) regulation loop is handling the water temperature (it starts/stops the house-heating system circulator which brings heat to a heat exchanger mounted on the pool water pipes)<br />
pH is regulated by injecting Acid from a tank into the pool water (a relay starts/stops the Acid peristaltic pump)<br />
ORP is regulated by injecting Chlorine from a tank into the pool water (a relay starts/stops the Chlorine peristaltic pump)<br />
Defined time-slots and water temperature are used to start/stop the filtration pump for a daily given amount of time (a relay starts/stops the filtration pump)<br />
Tank-levels are estimated based on the running-time and flow-rate of each pump.<br />
Ethernet connectivity parameters can be set through a webpage accessible from the LAN at http://PoolMaster.local.<br />
If an ethernet connection is available, the internal clock (RTC) is synchronized with a time-server every day at midnight.<br />

An API function enables telling the system what the outside air temperature is. In case it is below -2.0°C, filtration is started until it rises back above +2.0°C<br />
Communication with the system is performed using the MQTT protocol over an Ethernet connection to the local network/MQTT broker.<br /><br />

Every 30 seconds (by default), the system will publish on the "PoolTopicMeas1" and "PoolTopicMeas2"(see in code below) the following payloads in Json format:<br />
  {"Tmp":818,"pH":321,"PSI":56,"Orp":583,"FilUpT":8995,"PhUpT":0,"ChlUpT":0}<br />
  {"AcidF":100,"ChlF":100,"IO":11,"IO2":0}<br />
  Tmp: measured Water temperature value in °C x100 (8.18°C in the above example payload)<br />
  pH: measured pH value x100 (3.21 in the above example payload)<br />
  Orp: measured Orp (aka Redox) value in mV (583mV in the above example payload)<br />
  PSI: measured Water pressure value in bar x100 (0.56bar in the above example payload)<br />
  FiltUpT: current running time of Filtration pump in seconds (reset every 24h. 8995secs in the above example payload)<br />
  PhUpT: current running time of Ph pump in seconds (reset every 24h. 0secs in the above example payload)<br />
  ChlUpT: current running time of Chl pump in seconds (reset every 24h. 0secs in the above example payload)<br />
  AcidF: percentage fill estimate of acid tank ("pHTank" command must have been called when a new acid tank was set in place in order to have accurate value)<br />
  ChlF: percentage fill estimate of Chlorine tank ("ChlTank" command must have been called when a new Chlorine tank was set in place in order to have accurate value)<br />
IO: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are :<br />	
<ul>
<li>FiltPump: current state of Filtration Pump (0=on, 1=off)</li>
<li>PhPump: current state of Ph Pump (0=on, 1=off)</li>
<li>ChlPump: current state of Chl Pump (0=on, 1=off)</li>
<li>PhlLevel: current state of Acid tank level (0=empty, 1=ok)</li>
<li>ChlLevel: current state of Chl tank level (0=empty, 1=ok)</li>
<li>PSIError: over-pressure error</li>
<li>pHErr: pH pump overtime error flag</li>
<li>ChlErr: Chl pump overtime error flag</li>
</ul><br />
IO2: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are :<br /><br />
<ul>
<li>pHPID: current state of pH PID regulation loop (1=on, 0=off)</li>
<li>OrpPID: current state of Orp PID regulation loop (1=on, 0=off)</li>
<li>Mode: state of pH and Orp regulation mode (0=manual, 1=auto)</li>
<li>Heat: state of water heat command (0=off, 1=on)</li>
<li>R1: state of Relay1 (0=off, 1=on)</li>
<li>R2: state of Relay2 (0=off, 1=on)</li>
<li>R6: state of Relay6 (0=off, 1=on)</li>
<li>R7: state of Relay7 (0=off, 1=on)</li>
</ul><br />

<h4>How to compile</h4>
<p>
- this code was developped for two main hardware configurations (see list in the hardware section below):<br /> 
<ul>
<li>Controllino-Maxi or</li> 
<li>Arduino Mega 2560 + Ethernet shield + relay shield + RTC module</li></ul>
- select the target board type in the Arduino IDE (either "Arduino Mega 2560" or "Controllino Maxi") code should compile for both types<br />


<h4>Compatibility</h4>
	
<p>For this sketch to work on your setup you must change the following in the code (in the "Config.h" file):<br />
- possibly the pinout definitions depending on your wiring<br />
- the unique address of the DS18b20 water temperature sensor<br />
- MAC and IP address of the Ethernet shield<br />
- MQTT broker IP address and login credentials<br />
- possibly the topic names on the MQTT broker to subscribe and publish to<br />
- the Kp,Ki,Kd parameters for both PID loops in case your peristaltic pumps have a different throughput than 1.5Liters/hour for the pH pump and 3.0Liters/hour for the Chlorine pump. Also the default Kp values were adjusted for a 50m3 pool volume. You might have to adjust the Kp values in case of a different pool volume and/or peristaltic pumps throughput (start by adjusting it proportionally). In any case these parameters are likely to require adjustments for every pool<br /></p>

<h4>Tips</h4>
Before attempting to regulate your pool water with this automated system, it is essential that you start with:<br />
1- testing your water quality (using liquid kits and/or test strips for instance) and balancing it properly (pH, Chlorine, Alkalinity, Hardness). Proper water balancing will greatly ease the pH stability and regulation<br />
2- calibrating the pH probe using calibrated buffer solutions (pay attention to the water temperature which plays a big role in pH readings)<br />
3- adjusting pH to 7.4 <br />
4- once above steps 1 to 3 are ok, you can start regulating ORP<br /><br />
 
Notes:<br />
a/ the ORP sensor should theoretically not be calibrated nore temperature compensated (by nature its 0mV pivot point cannot shift)<br />
b/ the ORP reading is strongly affected by the pH value and the water temperature. Make sure pH is adjusted at 7.4<br />
c/ prefer platinium ORP probes for setpoints >500mV (ie. Pools and spas)<br />
e/ the response time of ORP sensors can be fast in reference buffer solutions (10 secs) and yet very slow in pool water (minutes or more) as it depends on the water composition <br /><br />


<p align="center"> <img src="/docs/PoolMaster.jpg" width="702" title="Overview"> </p> <br /><br />
<p align="center"> <img src="/docs/PoolMasterBox_pf.jpg" width="702" title="Overview"> </p> <br /><br />

<h4>MQTT API</h4>
<p>
Below are the Payloads/commands to publish on the "PoolTopicAPI" topic (see hardcoded in code) in Json format in order to launch actions on the Arduino:<br />
<ul>
<li>{"Mode":1} or {"Mode":0}         -> set "Mode" to manual (0) or Auto (1). In Auto, filtration starts/stops at set times of the day and pH and Orp are regulated</li> 
<li>{"Heat":1} or {"Heat":0}         -> start/stop the regulation of the pool water temperature</li>
<li>{"FiltPump":1} or {"FiltPump":0} -> manually start/stop the filtration pump</li>
<li>{"ChlPump":1} or {"ChlPump":0}   -> manually start/stop the Chl pump to add more Chlorine</li>
<li>{"PhPump":1} or {"PhPump":0}     -> manually start/stop the Acid pump to lower the Ph</li>
<li>{"PhPID":1} or {"PhPID":0}       -> start/stop the Ph PID regulation loop</li>
<li>{"OrpPID":1} or {"OrpPID":0}     -> start/stop the Orp PID regulation loop</li>
<li>{"PhCalib":[4.02,3.8,9.0,9.11]}  -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
<li>{"OrpCalib":[450,465,750,784]}   -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
<li>{"PhSetPoint":7.4}               -> set the Ph setpoint, 7.4 in this example</li>
<li>{"OrpSetPoint":750.0}            -> set the Orp setpoint, 750mV in this example</li>
<li>{"WSetPoint":27.0}               -> set the water temperature setpoint, 27.0deg in this example</li>
<li>{"WTempLow":10.0}                -> set the water low-temperature threshold below which there is no need to regulate Orp and Ph (ie. in winter)</li>
<li>{"OrpPIDParams":[2857,0,0]}      -> respectively set Kp,Ki,Kd parameters of the Orp PID loop. In this example they are set to 2857, 0 and 0</li>
<li>{"PhPIDParams":[1330000,0,0.0]}  -> respectively set Kp,Ki,Kd parameters of the Ph PID loop. In this example they are set to 1330000, 0 and 0</li>
<li>{"OrpPIDWSize":3600000}           -> set the window size of the Orp PID loop in msec, 60mins in this example</li>
<li>{"PhPIDWSize":1200000}            -> set the window size of the Ph PID loop in msec, 20mins in this example</li>
<li>{"Date":[1,1,1,18,13,32,0]}      -> set date/time of RTC module in the following format: (Day of the month, Day of the week, Month, Year, Hour, Minute, Seconds), in this example: Monday 1st January 2018 - 13h32mn00secs</li>
<li>{"FiltT0":9}                     -> set the earliest hour (9:00 in this example) to run filtration pump. Filtration pump will not run beofre that hour</li>
<li>{"FiltT1":20}                    -> set the latest hour (20:00 in this example) to run filtration pump. Filtration pump will not run after that hour</li>
<li>{"PubPeriod":30}                 -> set the periodicity (in seconds) at which the system info (pumps states, tank levels states, measured values, etc) will be published to the MQTT broker</li>
<li>{"PumpsMaxUp":1800}              -> set the Max Uptime (in secs) for the Ph and Chl pumps over a 24h period. If over, PID regulation is stopped and a warning flag is raised</li>
<li>{"Clear":1}                      -> reset the pH and Orp pumps overtime error flags in order to let the regulation loops continue. "Mode" also needs to be switched back to Auto (1) after an error flag was raised</li>
<li>{"DelayPID":30}                  -> Delay (in mins) after FiltT0 before the PID regulation loops will start. This is to let the Orp and pH readings stabilize first. 30mins in this example. Should not be > 59mins</li>
<li>{"TempExt":4.2}                  -> Provide the system with the external temperature. Should be updated regularly and will be used to start filtration when temperature is less than 2°C. 4.2deg in this example</li>
<li>{"PSIHigh":1.0}                  -> set the water high-pressure threshold (1.0bar in this example). When water pressure is over that threshold, an error flag is set</li>
<li>{"pHTank":[20,100]}              -> call this function when the Acid tank is replaced or refilled. First parameter is the tank volume in Liters, second parameter is its percentage fill (100% when full)</li>
<li>{"ChlTank":[20,100]}             -> call this function when the Chlorine tank is replaced or refilled. First parameter is the tank volume in Liters, second parameter is its percentage fill (100% when full)</li>
<li>{"Relay":[1,1]}                  -> call this generic command to actuate spare relays. Parameter 1 is the relay number (R1 in this example), parameter 2 is the relay state (ON in this example). This command is useful to use spare relays for additional features (lighting, etc). Available relay numbers are 1,2,6,7,8,9</li>
<li>{"Reboot":1}                     -> call this command to reboot the controller (after 8 seconds from calling this command)</li>
<li>{"pHPumpFR":1.5}                 -> call this command to set pH pump flow rate un L/s. In this example 1.5L/s</li>
<li>{"ChlPumpFR":3}                  -> call this command to set Chl pump flow rate un L/s. In this example 3L/s</li>
<li>{"RstpHCal":1}                   -> call this command to reset the calibration coefficients of the pH probe</li>
<li>{"RstOrpCal":1}                  -> call this command to reset the calibration coefficients of the Orp probe</li>
<li>{"RstPSICal":1}                  -> call this command to reset the calibration coefficients of the pressure sensor</li>


</ul>
</p><br />


<h4>Hardware</h4>
<p>
<ul>
<li><a title="https://www.controllino.biz/product/controllino-maxi/" href="https://www.controllino.biz/product/controllino-maxi/">x1 CONTROLLINO MAXI (ATmega2560)</a> or Arduino Mega 2560 + Ethernet shield + relay shield + RTC module</li>
<li><a title="https://www.phidgets.com/?tier=3&catid=11&pcid=9&prodid=103" href="https://www.phidgets.com/?tier=3&catid=11&pcid=9&prodid=103">x2 Phidgets PH/ORB amplifier modules</a></li> 
<li><a title="https://www.dfrobot.com/product-1621.html" href="https://www.dfrobot.com/product-1621.html">x2 Galvanic isolator for the pH and Orp probes</a></li> 
<li><a title="https://www.trattamento-acque.net/dosaggio/pompe-peristaltiche/pompe-a-portata-fissa/pompa-serie-mp2-p-detail.html" href="https://www.trattamento-acque.net/dosaggio/pompe-peristaltiche/pompe-a-portata-fissa/pompa-serie-mp2-p-detail.html">x2 Peristaltic pumps, suction lances for tanks, pH and Orp probes</a></li>
<li><a title="http://electrolyseur.fr/pool-terre.html" href="http://electrolyseur.fr/pool-terre.html">x1 Water grounding</a></li>
<li><a title="http://electrolyseur.fr/kit-sonde-DS18B20-filtration-piscine.html" href="http://electrolyseur.fr/kit-sonde-DS18B20-filtration-piscine.html">x1 Water temperature probe (DS18B20)</a></li>
<li><a title="https://fr.aliexpress.com/item/OOTDTY-G1-4-Pouces-5-v-0-0-5-MPa-Pression-Capteur-Capteur-D-huile-Carburant/32851667666.html?transAbTest=ae803_5&ws_ab_test=searchweb0_0%2Csearchweb201602_3_10065_10068_319_10892_317_10696_10084_453_454_10083_10618_10304_10307_10820_10821_537_10302_536_10902_10843_10059_10884_10887_321_322_10103%2Csearchweb201603_57%2CppcSwitch_0&algo_pvid=2456b33d-d7ee-4515-863d-af0c6b322395&algo_expid=2456b33d-d7ee-4515-863d-af0c6b322395-20
" href="https://fr.aliexpress.com/item/OOTDTY-G1-4-Pouces-5-v-0-0-5-MPa-Pression-Capteur-Capteur-D-huile-Carburant/32851667666.html?transAbTest=ae803_5&ws_ab_test=searchweb0_0%2Csearchweb201602_3_10065_10068_319_10892_317_10696_10084_453_454_10083_10618_10304_10307_10820_10821_537_10302_536_10902_10843_10059_10884_10887_321_322_10103%2Csearchweb201603_57%2CppcSwitch_0&algo_pvid=2456b33d-d7ee-4515-863d-af0c6b322395&algo_expid=2456b33d-d7ee-4515-863d-af0c6b322395-20
">x1 Pressure sensor</a></li>
<li><a title="https://www.itead.cc/nextion-nx4832k035.html" href="https://www.itead.cc/nextion-nx4832k035.html">x1 Nextion Enhanced NX4832K035 - Generic 3.5'' HMI Touch Display</a></li>

</ul>
</p><br />

<h4>Wiring</h4>
<p>
Below is a (quick and dirty) wiring diagram with the Controllino MAXI. Right click and display image in full screen to see the details.
<p align="center"> <img src="/docs/Wiring.jpg" width="702" title="Wiring"> </p> <br />
</p>

<h4>Cloud integration example (<a title="https://blynk.io/" href="https://blynk.io/">BLYNK</a>)</h4>
<p>
See NodeRed folder for more info and code
<p align="center"> <img src="/docs/Blynk2.jpg" width="302" title="Blynk"> </p> <br />
<p align="center"> <img src="/NodeRed/NodeRed-to-Blynk.jpg" width="702" title="Blynk"> </p> <br />
</p>

<h4>Non-cloud home automation integration example (Node-Red + InfluxDB + Grafana)</h4>
<p>
See the NodeRed folder for more info and this <a title="https://www.youtube.com/watch?v=JdV4x925au0" href="https://www.youtube.com/watch?v=JdV4x925au0">tutorial</a> on how to create a Grafana dashboard from MQTT data.<br />
<p align="center"> <img src="/docs/Grafana.png" width="702" title="Overview"> </p> <br />
<p align="center"> <img src="/docs/NodeRedDashboard.png" width="702" title="Overview"> </p> <br />

</p>

<h4>Non-cloud home automation integration example (<a title="https://www.jeedom.com" href="https://www.jeedom.com">JEEDOM</a>)</h4>
<p>
<p align="center"> <img src="/docs/JeedomInterface.jpg" width="702" title="Overview"> </p> <br />
<p align="center"> <img src="/docs/TuileJeedom.jpg" width="702" title="Overview"> </p> <br />
<p align="center"> <img src="/docs/VirtuelJeedom.jpg" width="702" title="Overview"> </p> <br />
</p>
