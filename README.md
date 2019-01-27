<h2>PoolMaster</h2>
<h2>Arduino/Controllino-Maxi (ATmega2560) based Ph/ORP regulator for home pool sysem</h2>
	
<h4>Compatibility</h4>
	
<p>For this sketch to work on your setup you must change the following in the code:<br />
- possibly the pinout definitions in case you are not using a CONTROLLINO MAXI board<br />
- the code related to the RTC module in case your setup does not have one<br />
- MAC address of DS18b20 water temperature sensor<br />
- MAC and IP address of the Ethernet shield<br />
- MQTT broker IP address and login credentials<br />
- possibly the topic names on the MQTT broker to subscribe and publish to<br />
- the Kp,Ki,Kd parameters for both PID loops in case your peristaltic pumps have a different throughput than 2Liters/hour. For instance, if twice more, divide the parameters by 2<br /></p>

<h4>Brief description</h4>
	
<p>Three main metrics are measured and periodically reported over MQTT and an LCD display: water temperature, PH and ORP values<br />
Pumps states, tank-level states and other parmaters are also periodically reported<br />
Two PID regulation loops are running in parallel: one for PH, one for ORP<br />
PH is regulated by injecting Acid from a tank into the pool water (a relay starts/stops the Acid peristaltic pump)<br />
ORP is regulated by injecting Chlorine from a tank into the pool water (a relay starts/stops the Chlorine peristaltic pump)<br />
Defined time-slots and water temperature are used to start/stop the filtration pump for a daily given amount of time (a relay starts/stops the filtration pump) <br />
A lightweight webserver provides a simple dynamic webpage with a summary of all system parameters. An XML file with more info is available at http://ARDUINO_LOCAL_IP/Info<br />
Communication with the system is performed using the MQTT protocol over an Ethernet connection to the local network/MQTT broker.<br /><br />

Every 30 seconds (by default), the system will publish on the "PoolTopic" (hardcoded, see code) the following payload in Json format:<br />

{"Tmp":818,"pH":321,"pHEr":0,"Orp":583,"OrpEr":0,"FilUpT":8995,"PhUpT":0,"ChlUpT":0,"IO":11,"IO2":0}


Temp: measured Water temperature value in °C x100 (8.18°C in the above example payload)<br />
pH: measured pH value x100 (3.21 in the above example payload)<br />
PhError/100: Ph PID regulation loop instantaneous error (0 in the above example payload)<br />
Orp: measured Orp (aka Redox) value in mV (583mV in the above example payload)<br />
OrpError/100: Orp PID regulation loop instantaneous error (0 in the above example payload)<br />
FiltUpTime: current running time of Filtration pump in seconds (reset every 24h. 8995secs in the above example payload)<br />
PhUpTime: current running time of Ph pump in seconds (reset every 24h. 0secs in the above example payload)<br />
ChlUpTime: current running time of Chl pump in seconds (reset every 24h. 0secs in the above example payload)<br />
IO: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are :<br />
<ul>
<li>FiltPump: current state of Filtration Pump (0=on, 1=off)</li>
<li>PhPump: current state of Ph Pump (0=on, 1=off)</li>
<li>ChlPump: current state of Chl Pump (0=on, 1=off)</li>
<li>PhlLevel: current state of Acid tank level (0=empty, 1=ok)</li>
<li>ChlLevel: current state of Chl tank level (0=empty, 1=ok)</li>
<li>Mode: (0=manual, 1=auto)</li>
<li>pHErr: pH pump overtime error flag</li>
<li>ChlErr: Chl pump overtime error flag</li>
</ul><br />
IO2: a variable of type BYTE where each individual bit is the state of a digital input on the Arduino. These are :<br />
<ul>
<li>pHPID: current state of pH PID regulation loop (1=on, 0=off)</li>
<li>OrpPID: current state of Orp PID regulation loop (1=on, 0=off)</li>
</ul><br />

</p>


<p align="center"> <img src="/docs/PoolMaster.jpg" width="602" title="Overview"> </p> <br /><br />
<p align="center"> <img src="/docs/PoolMasterBox_pf.jpg" width="602" title="Overview"> </p> <br /><br />
<p align="center"> <img src="/docs/Web.jpg" width="602" title="Overview"> </p> <br /><br />
<p align="center"> <img src="/docs/WebXML.jpg" width="602" title="Overview"> </p> <br />


<h4>MQTT API</h4>
<p>
Below are the Payloads/commands to publish on the "PoolTopicAPI" topic (see hardcoded in code) in Json format in order to launch actions on the Arduino:<br />
<ul>
<li>{"Mode":1} or {"Mode":0}         -> set "Mode" to manual (0) or Auto (1). In Auto, filtration starts/stops at set times of the day and pH and Orp are regulated</li> 
<li>{"FiltPump":1} or {"FiltPump":0} -> manually start/stop the filtration pump</li>
<li>{"ChlPump":1} or {"ChlPump":0}   -> manually start/stop the Chl pump to add more Chlorine</li>
<li>{"PhPump":1} or {"PhPump":0}     -> manually start/stop the Acid pump to lower the Ph</li>
<li>{"PhPID":1} or {"PhPID":0}       -> start/stop the Ph PID regulation loop</li>
<li>{"OrpPID":1} or {"OrpPID":0}     -> start/stop the Orp PID regulation loop</li>
<li>{"PhCalib":[4.02,3.8,9.0,9.11]}  -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
<li>{"OrpCalib":[450,465,750,784]}   -> multi-point linear regression calibration (minimum 1 point-couple, 6 max.) in the form [ProbeReading_0, BufferRating_0, xx, xx, ProbeReading_n, BufferRating_n]
<li>{"PhSetPoint":7.4}               -> set the Ph setpoint, 7.4 in this example</li>
<li>{"OrpSetPoint":750.0}            -> set the Orp setpoint, 750mV in this example</li>
<li>{"WSetPoint":27.0}               -> set the water temperature setpoint, 27.0deg in this example (for future use. Water heating not handled yet)</li>
<li>{"WTempLow":10.0}                -> set the water low-temperature threshold below which there is no need to regulate Orp and Ph (ie. in winter)</li>
<li>{"OrpPIDParams":[2857,0,0]}      -> respectively set Kp,Ki,Kd parameters of the Orp PID loop. In this example they are set to 2857, 0 and 0</li>
<li>{"PhPIDParams":[1330000,0,0.0]}  -> respectively set Kp,Ki,Kd parameters of the Ph PID loop. In this example they are set to 1330000, 0 and 0</li>
<li>{"OrpPIDWSize":1800000}           -> set the window size of the Orp PID loop in msec, 30mins in this example</li>
<li>{"PhPIDWSize":1800000}            -> set the window size of the Ph PID loop in msec, 30mins in this example</li>
<li>{"Date":[1,1,1,18,13,32,0]}      -> set date/time of RTC module in the following format: (Day of the month, Day of the week, Month, Year, Hour, Minute, Seconds), in this example: Monday 1st January 2018 - 13h32mn00secs</li>
<li>{"FiltT0":9}                     -> set the earliest hour (9:00 in this example) to run filtration pump. Filtration pump will not run beofre that hour</li>
<li>{"FiltT1":20}                    -> set the latest hour (20:00 in this example) to run filtration pump. Filtration pump will not run after that hour</li>
<li>{"PubPeriod":30}                 -> set the periodicity (in seconds) at which the system info (pumps states, tank levels states, measured values, etc) will be published to the MQTT broker</li>
<li>{"PumpsMaxUp":1800}              -> set the Max Uptime (in secs) for the Ph and Chl pumps over a 24h period. If over, PID regulation is stopped and a warning flag is raised</li>
<li>{"Clear":1}                      -> reset the pH and Orp pumps overtime error flags in order to let the regulation loops continue. "Mode" also needs to be switched back to Auto (1) after an error flag was raised</li>
</ul>
</p><br />


<h4>Hardware</h4>
<p>
<ul>
<li><a title="https://www.controllino.biz/controllino-maxi/maxi-and-maxi-pure/" href="https://www.controllino.biz/controllino-maxi/maxi-and-maxi-pure/">CONTROLLINO MAXI (ATmega2560)</a></li>
<li><a title="https://www.phidgets.com/docs/1130_User_Guide#Measuring_the_pH" href="https://www.phidgets.com/docs/1130_User_Guide#Measuring_the_pH">Phidgets PH/ORB amplifier modules</a></li> 
<li><a title="https://www.dfrobot.com/product-1621.html" href="https://www.dfrobot.com/product-1621.html">Galvanic isolator for the pH and Orp probes</a></li> 
<li><a title="https://www.trattamento-acque.net/dosaggio/pompe-peristaltiche/pompe-a-portata-fissa/pompa-serie-mp2-p-detail.html" href="https://www.trattamento-acque.net/dosaggio/pompe-peristaltiche/pompe-a-portata-fissa/pompa-serie-mp2-p-detail.html">Peristaltic pumps</li>
<li><a title="http://electrolyseur.fr/pool-terre.html" href="http://electrolyseur.fr/pool-terre.html">Water grounding</a></li>
<li><a title="Water temperature probe (DS18B20)" href="Water temperature probe (DS18B20)">Water temperature probe (DS18B20)</a></li>
</ul>
</p><br />

<h4>Home automation integration example (<a title="https://www.jeedom.com" href="https://www.jeedom.com">JEEDOM</a>)</h4>
<p>
<p align="center"> <img src="/docs/TuileJeedom.jpg" width="602" title="Overview"> </p> <br />
<p align="center"> <img src="/docs/VirtuelJeedom.jpg" width="602" title="Overview"> </p> <br />
</p>