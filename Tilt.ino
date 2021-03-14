#include "Wire.h" //i2c lib for accelermeter lib
#include "MPU6050.h" //lib for accelerometer
#include "RunningMedian.h" //lib for doing a quick median filter on accelerometer data
#include "math.h" //for trig functions
#include "tinyexpr.h" //nifty little library for applying curve fit algorithm

#define TILT_DEBUG //enable for sprint messages

#define MEDIANROUNDSMAX 49
#define MEDIANROUNDSMIN 29
#define MEDIANAVRG MEDIANROUNDSMIN
#define MEDIAN_MAX_SIZE MEDIANROUNDSMAX

RunningMedian samples = RunningMedian(MEDIANROUNDSMAX);

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

float Volt, Temperature, Tilt, Gravity;

//char my_polynominal[100] = "-0.00031*tilt^2+0.557*tilt-14.054";
char my_polynominal[100] = "-0.00031*tilt^2+0.557*tilt";

void setup() {
  Wire.begin();

#ifdef TILT_DEBUG    
  Serial.begin(38400);
#endif

  accelgyro.initialize();
  accelgyro.setTempFIFOEnabled(true);

  //run the IMU_Zero app to get the offsets
  accelgyro.setXAccelOffset(-3300);
  accelgyro.setYAccelOffset(1991);
  accelgyro.setZAccelOffset(1396);
  accelgyro.setXGyroOffset (-5);
  accelgyro.setYGyroOffset (18);
  accelgyro.setZGyroOffset (-21);
}

float getTemperature()
{
  //Run accelgyro.setTempFIFOEnabled(true) in setup to function

  return ((accelgyro.getTemperature() / 340.0) + 36.534); //coerce into Celcius degrees
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

float calculateGravity()
{
  double _tilt = Tilt;
  double _temp = Temperature;
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
#ifdef TILT_DEBUG
    Serial.println("Gravity calculation error");
#endif
  }
  return _gravity;
}


void loop() {

  Tilt = getTilt();
  Temperature = getTemperature();
  Gravity = calculateGravity();

  #ifdef TILT_DEBUG
    Serial.print("Gravity = ");
    Serial.println(Gravity);

    Serial.print("Temperature = ");
    Serial.println(Temperature);
  #endif

  delay(1000);
}
