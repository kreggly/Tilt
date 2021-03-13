// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "RunningMedian.h"
#include <FS.h>          //this needs to be first
#include "tinyexpr.h"
#include "OneWire.h"
#include "DallasTemperature.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#ifdef NO_CONSOLE
#define CONSOLE(x) \
    do             \
    {              \
    } while (0)
#define CONSOLELN CONSOLE
#define CONSOLEF CONSOLE
#else
#define CONSOLE(...)               \
    do                             \
    {                              \
        Serial.print(__VA_ARGS__); \
    } while (0)
#define CONSOLELN(...)               \
    do                               \
    {                                \
        Serial.println(__VA_ARGS__); \
    } while (0)
#endif


#define MEDIANROUNDSMAX 49
#define MEDIANROUNDSMIN 29
#define MEDIANAVRG MEDIANROUNDSMIN
#define MEDIAN_MAX_SIZE MEDIANROUNDSMAX

#define TEMP_CELSIUS 0
#define TEMP_FAHRENHEIT 1
#define TEMP_KELVIN 2

#define D1 5
#define D6 12
#define ONE_WIRE_BUS D6 // DS18B20 on ESP pin12
#define OW_PINS \
    (const uint8_t[]) { D1, D6 }
#define RESOLUTION 12 // 12bit resolution == 750ms update rate
#define OWinterval (760 / (1 << (12 - RESOLUTION)))

uint8_t my_tempscale = TEMP_CELSIUS;
int8_t my_OWpin = -1;
uint32_t DSreqTime = 0;
float Volt, Temperatur, Tilt, Gravity; // , corrGravity;

OneWire *oneWire;
DallasTemperature DS18B20;
DeviceAddress tempDeviceAddress;

char my_polynominal[100] = "-0.00031*tilt^2+0.557*tilt-14.054";


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


RunningMedian samples = RunningMedian(MEDIANROUNDSMAX);

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    initDS18B20();

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}





void requestTemp()
{
  if (!DSreqTime)
  {
    DS18B20.requestTemperatures();
    DSreqTime = millis();
  }
}

bool isDS18B20ready()
{
  return millis() - DSreqTime > OWinterval;
}

void getAccSample()
{
  accelgyro.getAcceleration(&ax, &az, &ay);
}

float getTilt()
{
  uint32_t start = millis();
  uint8_t i = 0;

  for (; i < MEDIANROUNDSMAX; i++)
  {
    while (!accelgyro.getIntDataReadyStatus())
      delay(2);
    
    getAccSample();
    
    float _tilt = calculateTilt();
    
    samples.add(_tilt);

    if (i >= MEDIANROUNDSMIN)
      break;
  }
 
  
  return samples.getAverage(MEDIANAVRG);
}


float calculateTilt()
{
  float _ax = ax;
  float _ay = ay;
  float _az = az;
  float pitch = (atan2(_ay, sqrt(_ax * _ax + _az * _az))) * 180.0 / M_PI;
  float roll = (atan2(_ax, sqrt(_ay * _ay + _az * _az))) * 180.0 / M_PI;
  return sqrt(pitch * pitch + roll * roll);
}

void initDS18B20()
{
  if (my_OWpin == -1)
  {
    my_OWpin = detectTempSensor(OW_PINS);
    if (my_OWpin == -1)
    {
      Serial.println("ERROR: cannot find a OneWire Temperature Sensor!");
      return;
    }
  }
  pinMode(my_OWpin, OUTPUT);
  digitalWrite(my_OWpin, LOW);
  delay(100);
  oneWire = new OneWire(my_OWpin);
  DS18B20 = DallasTemperature(oneWire);
  DS18B20.begin();

  bool device = DS18B20.getAddress(tempDeviceAddress, 0);
  if (!device)
  {
    my_OWpin = detectTempSensor(OW_PINS);
    if (my_OWpin == -1)
    {
      Serial.println("ERROR: cannot find a OneWire Temperature Sensor!");
      return;
    }
    else
    {
      delete oneWire;
      oneWire = new OneWire(my_OWpin);
      DS18B20 = DallasTemperature(oneWire);
      DS18B20.begin();
      DS18B20.getAddress(tempDeviceAddress, 0);
    }
  }

  DS18B20.setWaitForConversion(false);
  DS18B20.setResolution(tempDeviceAddress, RESOLUTION);
  requestTemp();
}



int detectTempSensor(const uint8_t pins[])
{

  for (uint8_t p = 0; p < sizeof(pins); p++)
  {
    const byte pin = pins[p];
    byte i;
    byte present = 0;
    byte type_s;
    byte data[12];
    byte addr[8];
    float celsius;

    CONSOLE(F("scanning for OW device on pin: "));
    CONSOLELN(pin);
    OneWire ds(pin);

    if (!ds.search(addr))
    {
      CONSOLELN(F("No devices found!"));
      ds.reset_search();
      delay(250);
      continue;
    }

    CONSOLE("Found device with ROM =");
    for (i = 0; i < 8; i++)
    {
      CONSOLE(' ');
      CONSOLE(addr[i], HEX);
    }

    if (OneWire::crc8(addr, 7) != addr[7])
    {
      CONSOLELN(" CRC is not valid!");
      continue;
    }
    CONSOLELN();

    // the first ROM byte indicates which chip
    switch (addr[0])
    {
    case 0x10:
      CONSOLELN("  Chip = DS18S20"); // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      CONSOLELN("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      CONSOLELN("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      CONSOLELN("Device is not a DS18x20 family device.");
      continue;
    }

    ds.reset();
    ds.select(addr);
    ds.write(0x44, 1); // start conversion, with parasite power on at the end

    delay(900); // maybe 750ms is enough, maybe not
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Read Scratchpad

    CONSOLE("  Data = ");
    CONSOLE(present, HEX);
    CONSOLE(" ");
    for (i = 0; i < 9; i++)
    { // we need 9 bytes
      data[i] = ds.read();
      CONSOLE(data[i], HEX);
      CONSOLE(" ");
    }
    CONSOLE(" CRC=");
    CONSOLELN(OneWire::crc8(data, 8), HEX);

    // Convert the data to actual temperature
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s)
    {
      raw = raw << 3; // 9 bit resolution default
      if (data[7] == 0x10)
      {
        // "count remain" gives full 12 bit resolution
        raw = (raw & 0xFFF0) + 12 - data[6];
      }
    }
    else
    {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00)
        raw = raw & ~7; // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20)
        raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40)
        raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
    celsius = (float)raw / 16.0;
    CONSOLE(F("  Temperature = "));
    CONSOLE(celsius);
    CONSOLELN(F(" Celsius, "));

    Temperatur = celsius; //todo hack
    return pin;
  }
  return -1;
}



float getTemperature(bool block = false)
{
  float t = Temperatur;

  // we need to wait for DS18b20 to finish conversion
  if (!DSreqTime ||
      (!block && !isDS18B20ready()))
    return t;

  // if we need the result we have to block
  while (!isDS18B20ready())
    delay(10);
  DSreqTime = 0;

  t = DS18B20.getTempCByIndex(0);

  if (t == DEVICE_DISCONNECTED_C || // DISCONNECTED
      t == 85.0)                    // we read 85 uninitialized
  {
    Serial.println("ERROR: OW DISCONNECTED");
    pinMode(my_OWpin, OUTPUT);
    digitalWrite(my_OWpin, LOW);
    delay(100);
    oneWire->reset();

    Serial.println("OW Retry");
    
    initDS18B20();
    
    delay(OWinterval);
    
    t = getTemperature(false);
  }
}

float calculateGravity()
{
  double _tilt = Tilt;
  double _temp = Temperatur;
  float _gravity = 0;
  int err;
  te_variable vars[] = {{"tilt", &_tilt}, {"temp", &_temp}};
  te_expr *expr = te_compile(my_polynominal, vars, 2, &err);

  if (expr)
  {
    _gravity = te_eval(expr);
    te_free(expr);
  }
  else
  {
    Serial.println("Gravity calculation error");
  }
  return _gravity;
}


void loop() {



Tilt = getTilt();
Serial.println(Tilt);


//Temperature = getTemperature(true);
//Serial.println(Temperature);

Gravity = calculateGravity();
Serial.println(Gravity);

delay(1000);
 
}
