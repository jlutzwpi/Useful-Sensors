/**
 * MIT License
 *
 * Copyright (c) 2022 R. Dunbar Poor
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <NDP.h>
#include <NDP_utils.h>
#include <Arduino.h>
#include "TinyML_init.h"
#include "NDP_init.h"
#include "NDP_loadModel.h"
#include "SAMD21_init.h"
#include "SAMD21_lowpower.h"
#include "AudioZero.h"
#include <Wire.h>
#include "wiring_private.h"

TwoWire myWire(&sercom5, 0, 1);

#define DO_NOT_WAIT_FOR_SERIAL_PORT 0                       // For battery powered autonomus usage, use this option
#define WAIT_FOR_SERIAL_PORT 1                              // For debugging purposes use this option, program will wait until serial port is turned on, Arduino IDE Tools --> Serial Monitor

//added by Justin to access the SD card for reading wav files
extern SdFat SD;
File myFile;
// filenames of wave files to play
const char who[] = "who3.wav";

static const byte kSlaveAddress = 0x62;
static const byte kModeRegister = 0x01;
static const byte kRunIdModelRegister = 0x03;
static const byte kSingleCaptureRegister = 0x02;
static const byte kCalibrationRegister = 0x04;
static const byte kSetSmoothingRegister = 0x05;
int id = -1;
bool recognized = false;
int match = 0;

#define NDP_MICROPHONE 0                                    // Model and this slection MUST match
#define NDP_SENSOR 1                                        // Model and this slection MUST match
#define NUMBER_OF_CLASSIFIERS 2                             // This includes z_openset

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


/**
 * @brief One-time setup, called upon reboot.
 */
void setup(void) {
  SAMD21_init(DO_NOT_WAIT_FOR_SERIAL_PORT);                        // Setting up SAMD21 (0) will not wait for serial port, (1) will wait and RGB LED will be Red
  myWire.begin();
  pinPeripheral(0, PIO_SERCOM_ALT);   //Assign SDA function to pin 0
  pinPeripheral(1, PIO_SERCOM_ALT);

  //pinPeripheral(15, PIO_SERCOM);

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
  //Serial.println("\nSpooky Picture");
  NDP_init("ei_model.bin",NDP_MICROPHONE);                  // Setting up NDP, Stuck Blue LED means model is not read 
  //Serial.println(""); 
  delay(2000);                                              // Flushing out buffer
  NDP.poll();
}

/**
 * @brief Read I2C data structure from the sensor and return it.
 */
inference_results_t read() {
  inference_results_t results = {};
  myWire.requestFrom(kSlaveAddress, sizeof(inference_results_t));
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

void NDP_INT_service(){                                  

  if(recognized == false && id == -1) 
  {
    myFile = SD.open("who4.wav", FILE_READ);
    AudioZero.begin(22050);
    AudioZero.play(myFile);
    AudioZero.end();
    myFile.close();
    delay(1000);
  }
  // Justin identified for the first time
  if(recognized == false && match == 1)
  {
    Serial.println("\n***We have a match! ****\n");
    recognized = true;
    //update the calibration register
    myWire.beginTransmission(kSlaveAddress);
    myWire.write(kCalibrationRegister);
    myWire.write((byte)true);
    myWire.endTransmission();
    delay(100);
  }
  //match for justin
  if(id > -1)
  {
    //play hi justin
    myFile = SD.open("hijustin.wav", FILE_READ);
    AudioZero.begin(22050);
    AudioZero.play(myFile);
    AudioZero.end();
    myFile.close();
  }
  else
  {
    //play don't know you
    myFile = SD.open("dontknowyou.wav", FILE_READ);
    AudioZero.begin(22050);
    AudioZero.play(myFile);
    AudioZero.end();
    myFile.close();
  }
  delay(2000);
}


void loop() {
  results = read();
  float confidence = results.confidence * 100;
  //float id_conf = results.id_confidence * 100;
  id = results.identity;
  Serial.println("ID: " + String(results.identity));
  //Serial.println("ID Confidence: " + String(id_conf));
  Serial.println("Face Confidence: " + String(confidence) + "%");
  Serial.println("-------------------------");
  Serial.println("Bbox coords: (" + String(results.bounding_box[0]) + "," + String(results.bounding_box[1]) + "), (" + String(results.bounding_box[2]) + "," + String(results.bounding_box[3]) + ").");
  //if we are confident in our results, play spooky voice
  match = NDP.poll(); 
  if(confidence > 85)
  {
    //if there is high confidence that a face is detected, have the picture start talking
    NDP_INT_service(); 
  }
  delay(2000);
}
