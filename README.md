# Apres_Battery_Monitor_V3.2.0
****This project is under Development and not fully tested yet****

The Project is based on: [SensESP_V3-3-Battery_Monitor](https://github.com/Techstyleuk/SensESP_V3-3-Battery_Monitor) with the following additions:

Changes for version 3.2.0
 1. Change SOC for Battery B to LiFePo4 - complete 
 2. Add AC Current Sensors x2 - complete
    1. Inverter
    2. Shore Power
 3. 4th INA219 for fuel level sensor - Complete

- The AC Current Sensor [SCT-013-030](https://amzn.to/3LTfUx4)
  - This device outputs 1v for every 30A flowing through the wire, so it is clamped onto a AC wire - in our case a Shore power or inverter line
  - Different versions are available with different current levels.  My boat has 30A service, so I chose 30A. 
- The DC Current and Voltage Sensor [HiLetgo 2pcs INA219 I2C Bi-Directional DC Current Power Supply Sensor](https://amzn.to/4b1D52w)

This device is featured on our [Youtube Channel](http://www.youtube.com/@ApresSail)

Thanks to Matt Bailey from [Boating with the Baileys](https://github.com/Boatingwiththebaileys) for inspiration on this project
