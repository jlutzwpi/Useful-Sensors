// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//Version 7, October 4, 2017.
//	  Modified script to EXPLICITLY work with the new Nano 33 BLE and Nano 33 BLE Sense boards. 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
#include <NDP.h>                                            // Arduino_library/libraries/NDP/src/
#include <NDP_utils.h>                                      // Arduino_library/libraries/TinyML_NDP_utils/src
#include <Arduino.h>
#include "TinyML_init.h"                                    // Arduino_library/libraries/TinyML_lowpower_lib
#include "SAMD21_init.h"                                    // Arduino_library/libraries/TinyML_lowpower_lib
#include "NDP_PMU.h" 
#include "PMIC_init.h" 
#include <Wire.h>
#include "wiring_private.h"

TwoWire myWire(&sercom5, 0, 1);

#define DO_NOT_WAIT_FOR_SERIAL_PORT 0                       // For battery powered autonomus usage, use this option
#define WAIT_FOR_SERIAL_PORT 1                              // For debugging purposes use this option, program will wait until serial port is turned on, Arduino IDE Tools --> Serial Monitor

static const byte kSlaveAddress = 0x62;
static const byte kModeRegister = 0x01;
static const byte kRunIdModelRegister = 0x02;
static const byte kSingleCaptureRegister = 0x03;
static const byte kCalibrationRegister = 0x04;
static const byte kSetSmoothingRegister = 0x05;

typedef enum{
    MODE_STANDBY, MODE_CONTINUOUS
} DeviceMode_t;

typedef struct {
  float confidence;
  float id_confidence;
  uint8_t bounding_box[4];
  int8_t identity;
} inference_results_t;

inference_results_t results;

void setup()
{
  SAMD21_init(WAIT_FOR_SERIAL_PORT);                        // Setting up SAMD21, the program will wait with Red LED
  PMIC_init();                                              // PMIC initalization
  //pinMode(ENABLE_5V, OUTPUT);
  //digitalWrite(ENABLE_5V, LOW);                             // The 5V switch is turned on so 5V appears on pin#2 of TinyML board so an external actuator can be powered on     
  //pmuBoost();
  //Serial.println("PMU Boost to provide 5V on Pin # 2");
  myWire.begin();
  pinPeripheral(0, PIO_SERCOM_ALT);   //Assign SDA function to pin 0
  pinPeripheral(1, PIO_SERCOM_ALT);

  myWire.beginTransmission(kSlaveAddress);
  myWire.write(kModeRegister);
  myWire.write(MODE_CONTINUOUS);
  myWire.endTransmission();
  //
  myWire.beginTransmission(kSlaveAddress);
  myWire.write(kRunIdModelRegister);
  myWire.write((byte)true);
  myWire.endTransmission();
  //
  myWire.beginTransmission(kSlaveAddress);
  myWire.write(kSetSmoothingRegister);
  myWire.write((byte)true);
  myWire.endTransmission();
  Serial.begin(115200);
  while (!Serial);             
  Serial.println("\nI2C Scanner");
}

inference_results_t read() {
  inference_results_t results = {};
  myWire.requestFrom(kSlaveAddress, sizeof(inference_results_t));
  //Wire.read(kSlaveAddress, (char*)&results, sizeof(inference_results_t));
  if (myWire.available() != sizeof(inference_results_t)) {
    Serial.println("*** Size mismatch: " + String(myWire.available()) + " vs " + String(sizeof(inference_results_t)));
    return results;
  }
  volatile float confidence;
  int8_t* confidence_bytes = (int8_t*)(&confidence);
  for (int i=0; i<sizeof(float); i++) {
    confidence_bytes[i] = myWire.read();
  }
  results.confidence = confidence;
  for (int i=0; i<sizeof(float); i++) {
    confidence_bytes[i] = myWire.read();
  }
  results.id_confidence = confidence;
  results.bounding_box[0] = myWire.read();
  results.bounding_box[1] = myWire.read();
  results.bounding_box[2] = myWire.read();
  results.bounding_box[3] = myWire.read();
  results.identity = myWire.read();

  return results;
}
 
void loop()
{
  byte error, address;
  int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    myWire.beginTransmission(address);
    error = myWire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
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

  results = read();
  int confidence = results.confidence * 100;
  Serial.println("ID: " + String(results.identity));
  Serial.println("Confidence: " + String(confidence) + "%");
  Serial.println("-------------------------");
  Serial.println("Bbox coords: (" + String(results.bounding_box[0]) + "," + String(results.bounding_box[1]) + "), (" + String(results.bounding_box[2]) + "," + String(results.bounding_box[3]) + ").");
  
 
  delay(3000);           // wait 5 seconds for next scan
}
