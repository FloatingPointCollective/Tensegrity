

#include <ESP8266WiFi.h>
//#include <Ticker.h>       //"threads"
#include <Wire.h>         //I2C
//#include <MQTT.h>         //MQTT stuff
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "MMA7455.h"
#include "MPR121.h"
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include <HslColor.h>
#include <RgbColor.h>

//Define pins
const int led = 13;
const int sda = 14;
const int scl = 5;

//Constants
const char* ssid = "TenseTower";//"Serenity";
const char* password = "61da6b36195ca405878ff3995861d0c9"; //"849hayes";
const uint8_t accelerometer_address = 0x1D;//MMA7455_I2C_ADDRESS;
const uint8_t capacitive_sensor_address = 0x5A;
const int UDP_PACKET_SIZE = 128;
const RgbColor red = RgbColor(255, 0, 0);
const RgbColor green = RgbColor(0, 255, 0);
const RgbColor blue = RgbColor(0, 0, 255);
const RgbColor white = RgbColor(255);
const RgbColor black = RgbColor(0);

//Variables
//Ticker flipper;
uint32_t chip_ID = 0;
WiFiClient wclient;
IPAddress server(192,168,1,142);
uint16_t port = 8080;
PubSubClient client(wclient, server, port);
bool i2c_devices[128];
bool has_accelerometer = 0; 
int accelerometer_available = 0;
bool has_capacitive_sensor = 0;
int capacitive_sensor_available = 0;
String client_name; //MQTT client name
bool MQTT_available = 0;
double dX,dY,dZ; //accelerometer g
int16_t x,y,z; //raw accelerometer data
bool touchStates[12]; //to keep track of the previous touch states
byte packetBuffer[ UDP_PACKET_SIZE]; 
WiFiUDP udp;
MMA_7455 mySensor = MMA_7455(); //Make an instance of MMA_7455
char xVal, yVal, zVal; //Variables for the values from the sensor
NeoPixelBus strip = NeoPixelBus(12, 13);

void i2c_scan(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    i2c_devices[address] = 0;
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
      i2c_devices[address] = 1;
      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void led_flip()
{
   static int state = 0;
   if(state){
    state = 0;
    digitalWrite(led,1);
    //Serial.println("1");
   } else {
    state = 1;
    digitalWrite(led,0);
 //Serial.println("0");
   }
  
}

void main_loop();


void setup() {
  //Debug led
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  //Debug serial port
  Serial.begin(115200);
  Serial.println("");

  chip_ID = ESP.getChipId();
  Serial.print("ESP8266 ID: ");
  Serial.println(chip_ID);
  
  //Join WiFi network
  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //Set up I2C
  Wire.begin(4,sda); 
  //Find devices
  i2c_scan();
  //accelerometer
  if(i2c_devices[accelerometer_address]){
        has_accelerometer = 1;
        Serial.println("Found accelerometer");
        int err = 0;
        mySensor.initSensitivity(2);
        if(err=0){//MMA7455_init()){
          Serial.println("Accelerometer initialization error");
          accelerometer_available = 0;
          Serial.println(err);
        } else {
          Serial.println("Accelerometer initialized successfully");
          accelerometer_available = 1;
        }
  } else {
        Serial.println("Accelerometer not available");
  }

  //capacitive sensor
  if(i2c_devices[capacitive_sensor_address]){
        has_capacitive_sensor = 1;
        Serial.println("Found capacitive sensor");
        mpr121_setup();
        Serial.println("Capacitive sensor initialized");
        capacitive_sensor_available = 1;
  } else {
        Serial.println("Capacitive sensor not available");
  }

  
  Serial.println("Initialized and connected to WiFi");
  Serial.println(WiFi.localIP());
  //flipper.attach(0.5,led_flip);
  
  client_name = String("ESP8266/");
  client_name += chip_ID;
  if(client.connect(client_name)){
    String bootup_state;
    bootup_state = String(chip_ID);
    bootup_state += ":\t";
    bootup_state += "ACC:";
    bootup_state += accelerometer_available;
    client.publish("bootup",bootup_state);    
    Serial.println("Sent data to broker");
    MQTT_available = 1;
  } else {
    Serial.println("Couldn't connect to broker");
    MQTT_available = 0;
  }

  //Set up LEDs
  strip.Begin();
  strip.Show();
  
  //ESP.wdtDisable();
  //flipper.attach(0.01,main_loop); //start main loop
  while(1){
    main_loop();
    //yield to keep WiFi alive!
    yield();//delay(10); 
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:


}

void main_loop(){
  static int cur = 0;
  //LED stuff
  RgbColor base_color = red;
  if(touchStates[3]){
    base_color = green;
  }
  if(dZ>50){
    base_color = white; 
  }
  for (unsigned i=0;i<12;++i){  
  if(touchStates[i]){
    strip.SetPixelColor(i, blue);
  } else {
    strip.SetPixelColor(i, base_color);
  }
  }
  
//  strip.SetPixelColor(0, white);
//  strip.SetPixelColor(1, red);
//  strip.SetPixelColor(2, green);
//  strip.SetPixelColor(3, blue);
//  strip.SetPixelColor(4, white);
//  strip.SetPixelColor(5, green);
  
  strip.Show();
  
  if(accelerometer_available){
    //get new accelerometer data
    int error;
    x = y = z = 0;
    mySensor.readAllAxes(&x, &y, &z);
    //x = mySensor.readAxis('x'); //Read out the 'x' Axis
    //y = mySensor.readAxis('y'); //Read out the 'y' Axis
    //z = mySensor.readAxis('z'); //Read out the 'z' Axis
    error = 0;//MMA7455_xyz(&x, &y, &z); // get the accelerometer values.
    // The function MMA7455_xyz returns the 'g'-force 
    // as an integer in 64 per 'g'.
    // set x,y,z to zero (they are not written in case of an error).
    dX = (int16_t) x;//    / 64.0;          // calculate the 'g' values.
    dY = (int16_t) y;// / 64.0;
    dZ = (int16_t) z;// / 64.0;
  }
  if(capacitive_sensor_available){
    readTouchInputs();    
  }
  if(MQTT_available ){
    //publish state
    
    if((WiFi.status() != WL_CONNECTED)){
      Serial.println("WiFi not connected!!!");
    } else {
//      Serial.println("UDP send");
//      memset(packetBuffer, 0, UDP_PACKET_SIZE);
//      for(int i=0;i<128;++i){
//        packetBuffer[i] = i;
//      }
//      udp.beginPacket(server  , 4999); //NTP requests are to port 123
//      udp.write(packetBuffer, UDP_PACKET_SIZE);
//      udp.endPacket();
//      Serial.println("UDP done");
//      client.publish("bootup","OK");  
    }
    String state;
    state = String("State: ");
    state += cur++;
    //Serial.println(client_name);
    //Serial.println(state);
      //Serial.println("publishing");
      StaticJsonBuffer<600> jsonBuffer; //needs to be very large for some reason
      JsonObject& root = jsonBuffer.createObject();
      root["has_accelerometer"] = accelerometer_available;
      root["iter"] = cur;
      root["acc_X"] = dX;
      root["acc_Y"] = dY;
      root["acc_Z"] = dZ;
      root["has_capacitive_sensor"] = capacitive_sensor_available;
      JsonArray& touch = root.createNestedArray("touch");
      for(unsigned i=0;i<12;++i){
        touch.add(touchStates[i]);
      }
      touch.add(touchStates[3]);
      touch.add(touchStates[4]);
      touch.add(touchStates[5]);
      char buffer[600];
      
      root.printTo(buffer,sizeof(buffer));
      client.publish("state",buffer);
      //Serial.println("done");
    
  }

  
}

