Arduino
=======================

This repository brings the embedded software used for the development of the ASV used in my master degree.
The embedded microcontroller is an Arduino Uno R3. It interfaces with the IMU GY-80 with a GPS unit, interact with the H-Bridge relays and communicates with the main computer running the software MOOS-IvP

This repository is organized as follows:

-   Projects/barco: This folder contains the Arduino code that is embedded in the ASV. A more detailed description can be found in my master thesis at my the moos-ivp-extend repository.

-   libraries: This folder contains the libraries used. TinyGPS, HMC5883L, ADXL345 and GY80IMU. The GY80IMU is a custom library adapted from other libraries (such as HMC5883L and ADXL345). This library is the main code to interface with the GY80IMU using I2C.

-   echo: this folder is necessary as an example for the library, but there is nothing special in it. Studying the code in the Projects/barco gives a better example on how to use the libraries.

