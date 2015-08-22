//Accelorometer libs
#include "MMA7455.h"
#include <Wire.h>         //I2C

//LED libs
#include <NeoPixelBus.h>
#include <NeoPixelAnimator.h>
#include <HslColor.h>
#include <RgbColor.h>

uint32_t chip_ID = 0;
//acceloremter stuff
bool i2c_devices[128];
const uint8_t accelerometer_address = 0x1D;//MMA7455_I2C_ADDRESS;
MMA_7455 mySensor = MMA_7455(); //Make an instance of MMA_7455
int accelerometer_available = 0;
bool has_accelerometer = 0;
double dX,dY,dZ; //accelerometer g
int16_t x,y,z; //raw accelerometer data

//Define pins
const int sda = 14;

//LED stuff
NeoPixelBus strip = NeoPixelBus(12, 13);
////sine waves
float s1 = 0;
float s2 = 0;
float s3 = 0;
float sinSpeed = 2;


void setup() {
  //Debug serial port
  Serial.begin(230400);
  
  Serial.println("");

  chip_ID = ESP.getChipId();
  Serial.print("ESP8266 ID: ");
  Serial.println(chip_ID);
  
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

  //Set up LEDs
  strip.Begin();
  strip.Show();

}

void loop() {
  //Serial.println("looping");
  // put your main code here, to run repeatedly:
  if(accelerometer_available){
    //get new accelerometer data
    int error;
    x = y = z = 0;
    mySensor.readAllAxes(&x, &y, &z);
    error = 0; // get the accelerometer values.
    // The function MMA7455_xyz returns the 'g'-force 
    // as an integer in 64 per 'g'.
    // set x,y,z to zero (they are not written in case of an error).
    dX = (int16_t) x;//    / 64.0;          // calculate the 'g' values.
    dY = (int16_t) y;// / 64.0;
    dZ = (int16_t) z;// / 64.0;

    //perform some calculations on the raw values
    dX = abs(dX);
    dY = abs(dY);
    dZ = abs(dZ);
    ////cap values at 255
    dX = constrain(dX,0,255);
    dY = constrain(dY,0,255);
    dZ = constrain(dZ,0,255);
    
    Serial.print("dX: ");
    Serial.print(dX);
    Serial.print("  dY: ");
    Serial.print(dY);
    Serial.print("  dZ: ");
    Serial.println(dZ);
  }

  //set led colors for cap 1
  int np = 6;
  for (unsigned i=0;i<6;++i){
      int r = getColorValue(i, np, s1, dX);
      int g = getColorValue(i, np, s1, dY);
      int b = getColorValue(i, np, s1, dZ);
      
      strip.SetPixelColor(i,RgbColor(r, g, b));
      
      //strip.SetPixelColor(i, RgbColor(dX, dY, dZ));
      
  }

  //set led colors for cap 2
  //apply "inverse" color to second cap
  for (unsigned i=6;i<12;++i){
     // Serial.print("sine: ");
     // Serial.println(sin(s1));
      strip.SetPixelColor(i, RgbColor(dX, dY, dZ));
  }

//  sinSpeed /= 20;
  
  s1+= .02 * sinSpeed;
  s2+= .03 * sinSpeed;
  s3+= .04 * sinSpeed;
  
  strip.Show();
}

int getColorValue(int index, int numPixels, float sine, double d){

  double sineVal = sin(sine * index/numPixels);

  //float mappedSineVal = map(sineVal, -1, 1, .9, 1);
  float mappedSine = sineVal+1;

/*
  Serial.print("sineVal: ");
  Serial.println(sineVal);
  
  Serial.print("mappedSineVal: ");
  Serial.println(mappedSine);

  Serial.print("d: ");
  Serial.println(d);*/
  
  //remap sine value range to effect the color value slightly only 
  float r =  round(mappedSine * d);
  //r = constrain(mappedSine,0, 255);

/*
  Serial.print("r: ");
  Serial.println(r);*/

 // delay(100);
  
  return r;
}

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
