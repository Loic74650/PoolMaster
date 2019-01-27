PoolMaster: change log
=======================

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