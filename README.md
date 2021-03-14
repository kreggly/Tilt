# Tilt
Simplifying the magic float hydrometer

This project expects an ESP8266 with an MPU6050 connected at SDA(4) and SCL(5).
I'm using an Adafruit Huzzah 8266 with integrated charger for development.

TODO: Add Wifi and MQTT support to interface with a NodeRed MQTT Server.

This will allow the Gravity and Temperature to be Subscribed to, and allow
the curve fit algorithm to calibrate gravity to be updated via a push to the ESP.
