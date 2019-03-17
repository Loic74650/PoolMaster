PoolMaster: change log
=======================
v2.1.2
-------

* changed Eeprom config storage handling
* added incoming MQTT message queue to reduce load on MQTT callback function (causing spurious socket errors and subsequent disconnections from broker)
* Ethernet deconnection now non-blocking
* Some minor changes

v2.1.1
-------

* fixed bug in AntiFreeze filtration function
* updated documentation

v2.1.0
-------

* LCD display now toggles between two screens every 6 seconds, shows more info
* modified PhPump() and ChlPump() functions to prevent the pumps from starting if Tank Levels are low
* Tanks Level errors now display error code on the LCD display but do not disable the PID loops or the filtering pûmp anymore, only the peristaltic pumps
* PID regulation loops start are now delayed for 30mins (by default) after the filtration start in order to let the readings stabilize first
* New API function to tell the system what the external temperature is. This is used to start regular short filtering periods of 10mins every hour when external temperature is below 2deg.
* Improved PID loops tuning

v2.0.1
-------

* /!\ changed CONTROLLINO pin number for the PH_PUMP
* Tuned PID parameters
* fixed bug in Mode API function

v2.0.0
-------

* New API function for multi-point linear regression calibration of pH and Orp sensors
* Former API function for the Offset Calibration of the pH and Orp sensors are no longer available
* getMeasures() function now based on calibrated linear function to compute pH and Orp values
* PublishDataCallback() function now using ArduinoJSON library to generate payload
* Eeprom, Webserver and XML file now store/display the calibration coefficients for the pH and Orp sensors

v1.0.0
-------

* First release
* LCD display support
* MQTT API
* Updated hardware list