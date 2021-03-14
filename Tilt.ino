#include "Wire.h" //i2c lib for accelermeter lib
#include "MPU6050.h" //lib for accelerometer
#include "RunningMedian.h" //lib for doing a quick median filter on accelerometer data
#include "math.h" //for trig functions
#include "tinyexpr.h" //nifty little library for applying curve fit algorithm

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
const char* ssid     = "xxxxxx";
const char* password = "xxxxxx";
const char* mqtt_server = "xxxxxx";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

//#define TILT_DEBUG //enable for sprint messages

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

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {  

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  payload[length] = '\0'; //terminate the byte buffer so we can cast it as a string and copy it.
 
  //TODO - rationalize new polynomial, else restore the old one.
  strcpy(my_polynominal,(char *)payload);

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP";
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("Data", "HIHI");
      // ... and resubscribe
      client.subscribe("Polynomial");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
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

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();

  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    dtostrf(value,0,0,msg);
    client.publish("Tick", msg);
    
    dtostrf(Gravity,0,2,msg);
    client.publish("Gravity", msg);
    
    dtostrf(Temperature,0,2,msg);
    client.publish("Temperature", msg);

    client.publish("Poly",my_polynominal);

  }

}
