# Tilt
Simplifying the magic float hydrometer

This project expects an ESP8266 with an MPU6050 connected at SDA(4) and SCL(5).
I'm using an Adafruit Huzzah 8266 with integrated charger for development.

Currently interfaces with a node red server to get gravity, temp, and the polynomial.
Can also change the gravity calibration poly too.

TODO: More cleanup and error checking, then onto sleep code to save battery.
