// MMA_7455.h - 3 Axis Accelerometer Library
// Moritz Kemper, IAD Physical Computing Lab
// moritz.kemper@zhdk.ch
// ZHdK, 03/04/2012
// Released under Creative Commons Licence

#ifndef MMA_7455_h
#define MMA_7455_h

#include "Arduino.h"
#include "Wire.h"

class MMA_7455
{
  public:
    MMA_7455();
    void initSensitivity(int sensitivity);
    void calibrateOffset(float x_axis_offset, float y_axis_offset, float z_axis_offset);
    unsigned char readAxis(char axis);
    void readAllAxes(int16_t* x, int16_t* y, int16_t* z);
  private:
    unsigned char _buffer;
  float _x_axis_offset;
  float _y_axis_offset;
  float _z_axis_offset;
};

#endif

//// MMA7455 Accelerometer
//// ---------------------
////
//// By arduino.cc user "Krodal".
//// May 2012
//// Open Source / Public Domain
////
//// Fixes to union and type conversions by Arduino.cc user "Afroviking"
//// August 2014
////
//// Using Arduino 1.0.1
//// It will not work with an older version, since Wire.endTransmission() 
//// uses a parameter to hold or release the I2C bus.
////
//// Documentation:
////     - The Freescale MMA7455L datasheet 
////     - The AN3468 Application Note (programming).
////     - The AN3728 Application Note (calibrating offset).
//// 
//// The MMA7455 can be used by writing and reading a single byte, 
//// but it is also capable to read and write multiple bytes.
////
//// The accuracy is 10-bits.
//// 
//
//#ifndef MMA7455_H
//#define MMA7455_H
//#include <Wire.h>
//
//
//// Register names according to the datasheet.
//// Register 0x1C is sometimes called 'PW', and sometimes 'PD'.
//// The two reserved registers can not be used.
//#define MMA7455_XOUTL 0x00      // Read only, Output Value X LSB
//#define MMA7455_XOUTH 0x01      // Read only, Output Value X MSB
//#define MMA7455_YOUTL 0x02      // Read only, Output Value Y LSB
//#define MMA7455_YOUTH 0x03      // Read only, Output Value Y MSB
//#define MMA7455_ZOUTL 0x04      // Read only, Output Value Z LSB
//#define MMA7455_ZOUTH 0x05      // Read only, Output Value Z MSB
//#define MMA7455_XOUT8 0x06      // Read only, Output Value X 8 bits
//#define MMA7455_YOUT8 0x07      // Read only, Output Value Y 8 bits
//#define MMA7455_ZOUT8 0x08      // Read only, Output Value Z 8 bits
//#define MMA7455_STATUS 0x09     // Read only, Status Register
//#define MMA7455_DETSRC 0x0A     // Read only, Detection Source Register
//#define MMA7455_TOUT 0x0B       // Temperature Output Value (Optional)
//#define MMA7455_RESERVED1 0x0C  // Reserved
//#define MMA7455_I2CAD 0x0D      // Read/Write, I2C Device Address
//#define MMA7455_USRINF 0x0E     // Read only, User Information (Optional)
//#define MMA7455_WHOAMI 0x0F     // Read only, "Who am I" value (Optional)
//#define MMA7455_XOFFL 0x10      // Read/Write, Offset Drift X LSB
//#define MMA7455_XOFFH 0x11      // Read/Write, Offset Drift X MSB
//#define MMA7455_YOFFL 0x12      // Read/Write, Offset Drift Y LSB
//#define MMA7455_YOFFH 0x13      // Read/Write, Offset Drift Y MSB
//#define MMA7455_ZOFFL 0x14      // Read/Write, Offset Drift Z LSB
//#define MMA7455_ZOFFH 0x15      // Read/Write, Offset Drift Z MSB
//#define MMA7455_MCTL 0x16       // Read/Write, Mode Control Register 
//#define MMA7455_INTRST 0x17     // Read/Write, Interrupt Latch Reset
//#define MMA7455_CTL1 0x18       // Read/Write, Control 1 Register
//#define MMA7455_CTL2 0x19       // Read/Write, Control 2 Register
//#define MMA7455_LDTH 0x1A       // Read/Write, Level Detection Threshold Limit Value
//#define MMA7455_PDTH 0x1B       // Read/Write, Pulse Detection Threshold Limit Value
//#define MMA7455_PD 0x1C         // Read/Write, Pulse Duration Value
//#define MMA7455_LT 0x1D         // Read/Write, Latency Time Value (between pulses)
//#define MMA7455_TW 0x1E         // Read/Write, Time Window for Second Pulse Value
//#define MMA7455_RESERVED2 0x1F  // Reserved
//
//// Defines for the bits, to be able to change 
//// between bit number and binary definition.
//// By using the bit number, programming the MMA7455 
//// is like programming an AVR microcontroller.
//// But instead of using "(1<<X)", or "_BV(X)", 
//// the Arduino "bit(X)" is used.
//#define MMA7455_D0 0
//#define MMA7455_D1 1
//#define MMA7455_D2 2
//#define MMA7455_D3 3
//#define MMA7455_D4 4
//#define MMA7455_D5 5
//#define MMA7455_D6 6
//#define MMA7455_D7 7
//
//// Status Register
//#define MMA7455_DRDY MMA7455_D0
//#define MMA7455_DOVR MMA7455_D1
//#define MMA7455_PERR MMA7455_D2
//
//// Mode Control Register
//#define MMA7455_MODE0 MMA7455_D0
//#define MMA7455_MODE1 MMA7455_D1
//#define MMA7455_GLVL0 MMA7455_D2
//#define MMA7455_GLVL1 MMA7455_D3
//#define MMA7455_STON MMA7455_D4
//#define MMA7455_SPI3W MMA7455_D5
//#define MMA7455_DRPD MMA7455_D6
//
//// Control 1 Register
//#define MMA7455_INTPIN MMA7455_D0
//#define MMA7455_INTREG0 MMA7455_D1
//#define MMA7455_INTREG1 MMA7455_D2
//#define MMA7455_XDA MMA7455_D3
//#define MMA7455_YDA MMA7455_D4
//#define MMA7455_ZDA MMA7455_D5
//#define MMA7455_THOPT MMA7455_D6
//#define MMA7455_DFBW MMA7455_D7
//
//// Control 2 Register
//#define MMA7455_LDPL MMA7455_D0
//#define MMA7455_PDPL MMA7455_D1
//#define MMA7455_DRVO MMA7455_D2
//
//// Interrupt Latch Reset Register
//#define MMA7455_CLR_INT1 MMA7455_D0
//#define MMA7455_CLR_INT2 MMA7455_D1
//
//// Detection Source Register
//#define MMA7455_INT1 MMA7455_D0
//#define MMA7455_INT2 MMA7455_D1
//#define MMA7455_PDZ MMA7455_D2
//#define MMA7455_PDY MMA7455_D3
//#define MMA7455_PDX MMA7455_D4
//#define MMA7455_LDZ MMA7455_D5
//#define MMA7455_LDY MMA7455_D6
//#define MMA7455_LDX MMA7455_D7
//
//// I2C Device Address Register
//#define MMA7455_I2CDIS MMA7455_D7
//
//// Default I2C address for the MMA7455
//#define MMA7455_I2C_ADDRESS 0x1D
//
//
//// When using an union for the registers and 
//// the axis values, the byte order of the accelerometer
//// should match the byte order of the compiler and AVR chip.
//// Both have the lower byte at the lower address, 
//// so they match.
//// This union is only used by the low level functions.
//union xyz_union
//{
//  struct
//  {
//    uint8_t x_lsb;
//    uint8_t x_msb;
//    uint8_t y_lsb;
//    uint8_t y_msb;
//    uint8_t z_lsb;
//    uint8_t z_msb;
//  } reg;
//  struct 
//  {
//    uint16_t x;
//    uint16_t y;
//    uint16_t z;
//  } value;
//};
//
//
//// --------------------------------------------------------
//// MMA7455_init
////
//// Initialize the MMA7455.
//// Set also the offset, assuming that the accelerometer is 
//// in flat horizontal position.
//// 
//// Important notes about the offset:
////    The sensor has internal registers to set an offset.
////    But the offset could also be calculated by software.
////    This function uses the internal offset registers 
////    of the sensor.
////    That turned out to be bad idea, since setting the
////    offset alters the actual offset of the sensor.
////    A second offset calculation had to be implemented 
////    to fine tune the offset.
////    Using software variables for the offset would be 
////    much better.
//// 
////    The offset is influenced by the slightest vibration
////    (like a computer on the table).
////    
//int MMA7455_init(void);
//
//// --------------------------------------------------------
//// MMA7455_xyz
////
//// Get the 'g' forces.
//// The values are with integers as 64 per 'g'.
////
//int MMA7455_xyz( uint16_t *pX, uint16_t *pY, uint16_t *pZ);
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
//int MMA7455_read(int start, uint8_t *buffer, int size);
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
//int MMA7455_write(int start, const uint8_t *pData, int size);
//
//#endif 
//

