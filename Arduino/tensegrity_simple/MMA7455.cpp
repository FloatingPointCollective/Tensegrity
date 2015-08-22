//DOCUMENTATION HERE: https://code.google.com/p/mma-7455-arduino-library/
// MMA_7455.cpp - 3 Axis Accelerometer Library
// Moritz Kemper, IAD Physical Computing Lab
// moritz.kemper@zhdk.ch
// ZHdK, 03/04/2012
// Released under Creative Commons Licence

#include "MMA7455.h"

#define MMA_7455_ADDRESS 0x1D //I2C Adsress for the sensor
#define MMA_7455_MODE_CONTROLL 0x16 //Call the sensors Mode Control

#define MMA_7455_2G_MODE 0x05 //Set Sensitivity to 2g
#define MMA_7455_4G_MODE 0x09 //Set Sensitivity to 4g
#define MMA_7455_8G_MODE 0x01 //Set Sensitivity to 8g

#define X_OUT 0x06 //Register for reading the X-Axis
#define Y_OUT 0x07 //Register for reading the Y-Axis
#define Z_OUT 0x08 //Register for reading the Z-Axis
    
MMA_7455::MMA_7455()
{ 
 // Wire.begin();
}

void MMA_7455::initSensitivity(int sensitivity)
{
  delay(1000);
  Wire.beginTransmission(MMA_7455_ADDRESS);
  Wire.write(MMA_7455_MODE_CONTROLL);
  if(sensitivity == 2)
  {
    Wire.write(MMA_7455_2G_MODE);  //Set Sensitivity to 2g Detection
  }
  if(sensitivity == 4)
  {
    Wire.write(MMA_7455_4G_MODE);  //Set Sensitivity to 4g Detection
  }
  if(sensitivity == 8)
  {
    Wire.write(MMA_7455_8G_MODE);  //Set Sensitivity to 8g Detection
  }
  Wire.endTransmission();
  delay(1000);
}

void MMA_7455::calibrateOffset(float x_axis_offset, float y_axis_offset, float z_axis_offset)
{
  _x_axis_offset = x_axis_offset;
  _y_axis_offset = y_axis_offset;
  _z_axis_offset = z_axis_offset;
}
unsigned char MMA_7455::readAxis(char axis)
{
  Wire.beginTransmission(MMA_7455_ADDRESS);
  if(axis == 'x' || axis == 'X')
  {
    Wire.write(X_OUT);
  }
  if(axis == 'y' || axis == 'Y')
  {
    Wire.write(Y_OUT);
  }
  if(axis == 'z' || axis == 'Z')
  {
    Wire.write(Z_OUT);
  }
  Wire.endTransmission();
  Wire.beginTransmission(MMA_7455_ADDRESS);
  Wire.requestFrom(MMA_7455_ADDRESS, 1);
  if(Wire.available())
  {
    _buffer = Wire.read();
  }
  if(axis == 'x' || axis == 'X')
  {
    _buffer = _buffer + _x_axis_offset;
  }
  if(axis == 'y' || axis == 'Y')
  {
    _buffer = _buffer + _y_axis_offset;
  }
  if(axis == 'z' || axis == 'Z')
  {
    _buffer = _buffer + _z_axis_offset;
  }

  return _buffer;
}

void MMA_7455::readAllAxes(int16_t* x, int16_t* y, int16_t* z)
{
  uint8_t data[6];
  Wire.beginTransmission(MMA_7455_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(MMA_7455_ADDRESS);
  Wire.requestFrom(MMA_7455_ADDRESS, 6);
  for(unsigned i=0; i<6;++i){
    if(Wire.available()){
      data[i] = Wire.read();
    } else {
      break;
    }
  }
  *x = (((uint16_t)data[1])<<8) | data[0];
  *y = (((uint16_t)data[3])<<8) | data[2];
  *z = (((uint16_t)data[5])<<8) | data[4];
  if(*x>511){
    *x -= 1024;
  }
  if(*y>511){
    *y -= 1024;
  }
  if(*z>511){
    *z -= 1024;
  }
  
}

//#include <Wire.h>
//#include <Arduino.h>
//#include "MMA7455.h"
//
//int MMA7455_init(void)
//{
//  uint16_t x, y, z;
//  int error;
//  xyz_union xyz;
//  uint8_t c1, c2;
//
//  // Initialize the sensor
//  //
//  // Sensitivity:
//  //    2g : GLVL0
//  //    4g : GLVL1
//  //    8g : GLVL1 | GLVL0
//  // Mode:
//  //    Standby         : 0
//  //    Measurement     : MODE0
//  //    Level Detection : MODE1
//  //    Pulse Detection : MODE1 | MODE0
//  // There was no need to add functions to write and read 
//  // a single byte. So only the two functions to write 
//  // and read multiple bytes are used.
//
//  // Set mode for "2g sensitivity" and "Measurement Mode".
//  c1 = bit(MMA7455_GLVL0) | bit(MMA7455_MODE0);
//  error = MMA7455_write(MMA7455_MCTL, &c1, 1);
//  
//  if (error != 0)
//    return (error);
//  Serial.println("Set 2G sensitivity");
//
//  // Read it back, to test the sensor and communication.
//  error = MMA7455_read(MMA7455_MCTL, &c2, 1);
//  if (error != 0)
//    return (error);
//  Serial.println("Tested sensor and communication");
//
//  if (c1 != c2)
//    return (-99);
//  Serial.println("c1!=c2");
//
//  // Clear the offset registers.
//  // If the Arduino was reset or with a warm-boot,
//  // there still could be offset written in the sensor.
//  // Only with power-up the offset values of the sensor 
//  // are zero.
//  xyz.value.x = xyz.value.y = xyz.value.z = 0;
//  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
//  if (error != 0)
//    return (error);
//  Serial.println("Cleared offset registers");
//
//  // The mode has just been set, and the sensor is activated.
//  // To get a valid reading, wait some time.
//  delay(100);
//
//#define USE_INTERNAL_OFFSET_REGISTERS
//#ifdef USE_INTERNAL_OFFSET_REGISTERS
//
//  // Calcuate the offset.
//  //
//  // The values are 16-bits signed integers, but the sensor
//  // uses offsets of 11-bits signed integers.
//  // However that is not a problem, 
//  // as long as the value is within the range.
//
//  // Assuming that the sensor is flat horizontal, 
//  // the 'z'-axis should be 1 'g'. And 1 'g' is 
//  // a value of 64 (if the 2g most sensitive setting 
//  // is used).  
//  // Note that the actual written value should be doubled
//  // for this sensor.
//
//  error = MMA7455_xyz (&x, &y, &z); // get the x,y,z values
//  if (error != 0)
//    return (error);
//  Serial.println("Got initial XYZ");
//
//  xyz.value.x = 2 * -x;        // The sensor wants double values.
//  xyz.value.y = 2 * -y;
//  xyz.value.z = 2 * -(z-64);   // 64 is for 1 'g' for z-axis.
//
//  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
//  if (error != 0)
//    return (error);
//  Serial.println("wrote offsets");
//
//  // The offset has been set, and everything should be okay.
//  // But by setting the offset, the offset of the sensor
//  // changes.
//  // A second offset calculation has to be done after 
//  // a short delay, to compensate for that.
//  delay(200);
//
//  error = MMA7455_xyz (&x, &y, &z);    // get te x,y,z values again
//  if (error != 0)
//    return (error);
//  Serial.println("got values again");
//
//  xyz.value.x += 2 * -x;       // add to previous value
//  xyz.value.y += 2 * -y;
//  xyz.value.z += 2 * -(z-64);  // 64 is for 1 'g' for z-axis.
//
//  // Write the offset for a second time.
//  // This time the offset is fine tuned.
//  error = MMA7455_write(MMA7455_XOFFL, (uint8_t *) &xyz, 6);
//  if (error != 0)
//    return (error);
//  Serial.println("wrote offset again");
//
//#endif
//
//  return (0);          // return : no error
//}
//
//
//// --------------------------------------------------------
//// MMA7455_xyz
////
//// Get the 'g' forces.
//// The values are with integers as 64 per 'g'.
////
//int MMA7455_xyz( uint16_t *pX, uint16_t *pY, uint16_t *pZ)
//{
//  xyz_union xyz;
//  int error;
//  uint8_t c;
//
//  // Wait for status bit DRDY to indicate that 
//  // all 3 axis are valid.
//  do
//  {
//    error = MMA7455_read (MMA7455_STATUS, &c, 1);
//  } while ( !bitRead(c, MMA7455_DRDY) && error == 0);
//  if (error != 0)
//    return (error);
//
//  // Read 6 bytes, containing the X,Y,Z information 
//  // as 10-bit signed integers.
//  error = MMA7455_read (MMA7455_XOUTL, (uint8_t *) &xyz, 6);
//  if (error != 0)
//    return (error);
//
//  // The output is 10-bits and could be negative.
//  // To use the output as a 16-bit signed integer,
//  // the sign bit (bit 9) is extended for the 16 bits.
//  if (xyz.reg.x_msb & 0x02)    // Bit 9 is sign bit.
//    xyz.reg.x_msb |= 0xFC;     // Stretch bit 9 over other bits.
//  else
//    xyz.reg.x_msb &= 0x3;
//
//  if (xyz.reg.y_msb & 0x02)
//    xyz.reg.y_msb |= 0xFC;
//  else
//    xyz.reg.y_msb &= 0x3;
//
//  if (xyz.reg.z_msb & 0x02)
//    xyz.reg.z_msb |= 0xFC;
//  else
//    xyz.reg.z_msb &= 0x3;
//
//  // The result is the g-force in units of 64 per 'g'.
//  *pX = xyz.value.x;
//  *pY = xyz.value.y;
//  *pZ = xyz.value.z;
//
//  return (0);                  // return : no error
//}
//
//
//// --------------------------------------------------------
//// MMA7455_read
////
//// This is a common function to read multiple bytes 
//// from an I2C device.
////
//// It uses the boolean parameter for Wire.endTransMission()
//// to be able to hold or release the I2C-bus. 
//// This is implemented in Arduino 1.0.1.
////
//// Only this function is used to read. 
//// There is no function for a single byte.
////
//int MMA7455_read(int start, uint8_t *buffer, int size)
//{
//  int i, n, error;
//
//  Wire.beginTransmission(MMA7455_I2C_ADDRESS);
//  n = Wire.write(start);
//  if (n != 1)
//    return (-10);
//
//  n = Wire.endTransmission(false); // hold the I2C-bus
//  if (n != 0)
//    return (n);
//
//  // Third parameter is true: relase I2C-bus after data is read.
//  Wire.requestFrom((MMA7455_I2C_ADDRESS), (size), true);
//  i = 0;
//  while(Wire.available() && i<size)
//  {
//    buffer[i++]=Wire.read();
//  }
//  if ( i != size)
//    return (-11);
//
//  return (0);                  // return : no error
//}
//
//
//// --------------------------------------------------------
//// MMA7455_write
////
//// This is a common function to write multiple bytes 
//// to an I2C device.
////
//// Only this function is used to write. 
//// There is no function for a single byte.
////
//int MMA7455_write(int start, const uint8_t *pData, int size)
//{
//  int n, error;
//
//  Wire.beginTransmission(MMA7455_I2C_ADDRESS);
//  n = Wire.write(start);        // write the start address
//  if (n != 1)
//    return (-20);
//
//  n = Wire.write(pData, size);  // write data bytes
//  if (n != size)
//    return (-21);
//
//  error = Wire.endTransmission(true); // release the I2C-bus
//  if (error != 0)
//    return (error);
//
//  return (0);                   // return : no error
//}
