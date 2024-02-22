//data structure[pattern, nowRead2, nowRead3, nowRead4]
#include <ArduinoBLE.h>
#include <stdint.h>
#include "LSM6DS3.h"
#include "Wire.h"
#include <math.h>

#include "Res.h"
#define RES_PIN A0

#define ANGLE_PIN A1
#define JOINTSWELL_PIN A2
#define THIGHSWELL_PIN A3

#define SMOOTH_DEG 10

// Create an instance of class returning Flex sensor ADC value
Res res(RES_PIN);  // sample

Res angle(ANGLE_PIN);
Res joint(JOINTSWELL_PIN);
Res thigh(THIGHSWELL_PIN);

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A

uint16_t preRead;
uint16_t nowRead;
// flex sensor data [nowRead2, nowRead3, nowRead4]
uint16_t preRead2;
uint16_t nowRead2;
uint16_t preRead3;
uint16_t nowRead3;
uint16_t preRead4;
uint16_t nowRead4;
//final data sending through ble
uint8_t angleDataA1;
uint8_t jointSwellDataA2;
uint8_t tightSwellDataA3;

const uint8_t bendingThreshold = 2;
int pattern;

//BLE Setup
BLEService bleService("56363134-0000-0000-0000-000000000000");

BLEUnsignedIntCharacteristic xyzpCharacteristic("007A", BLERead | BLENotify);
// // BLEShortCharacteristic bendCharacteristic("007B", BLERead | BLENotify);
BLEShortCharacteristic bendCharacteristic("007B", BLERead | BLENotify);

void setup() {
  Serial.begin(9600);
  //Call .begin() to configure the IMUs
  // if (myIMU.begin() != 0) {
  //   Serial.println("Device error");
  // } else {
  //   Serial.println("Device OK!");
  // }
  // delay(1000);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1)
      ;
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("V614");
  BLE.setAdvertisedService(bleService);

  // add the characteristic to the service
  bleService.addCharacteristic(xyzpCharacteristic);
  bleService.addCharacteristic(bendCharacteristic);

  // add service
  BLE.addService(bleService);

  // set the initial value for the characeristic:
  xyzpCharacteristic.writeValue(0);
  bendCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE Peripheral");
}

bool motionDetection() {
  bool motionState = false;

  preRead = nowRead;
  preRead2 = nowRead2;
  preRead3 = nowRead3;
  preRead4 = nowRead4;
  // read the flex sensor
  nowRead = res.Read(SMOOTH_DEG);
  nowRead2 = angle.Read(SMOOTH_DEG);
  nowRead3 = joint.Read(SMOOTH_DEG);
  nowRead4 = thigh.Read(SMOOTH_DEG);

  // Calculate pattern based on sensor readings
  pattern = random(0, 5);

  // check if movement is above the threshold
  if (abs(nowRead - preRead) >= bendingThreshold || abs(nowRead2 - preRead2) >= bendingThreshold || abs(nowRead3 - preRead3) >= bendingThreshold || abs(nowRead4 - preRead4) >= bendingThreshold) {
    motionState = true;
  }
  return motionState;
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    //Serial.print("Connected to central: ");
    // print the central's MAC address:
    //Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      if (motionDetection() == true) {
        Serial.print(pattern);
        Serial.print(",");
        // Serial.print(nowRead);
        // Serial.print(",");
        Serial.print(nowRead2);
        Serial.print(",");
        Serial.print(nowRead3);
        Serial.print(",");
        Serial.println(nowRead4);

        angleDataA1 = compressTo8Bit(nowRead2);
        jointSwellDataA2 = compressTo8Bit(nowRead3);
        tightSwellDataA3 = compressTo8Bit(nowRead4);

        // Serial.print(angleDataA1);
        // Serial.print(",");
        // Serial.print(jointSwellDataA2);
        // Serial.print(",");
        // Serial.println(tightSwellDataA3);

        //convert flex sensors data and pattern in one character
        uint32_t concatFlexValues = ((uint8_t)pattern << 24) | (tightSwellDataA3 << 16) | (jointSwellDataA2 << 8) | angleDataA1;

        //uint32_t concatFlexValues = ((uint32_t)nowRead4 << 22) | ((uint32_t)nowRead3 << 12) | ((uint32_t)nowRead2 << 2) | (uint32_t)pattern;
        // Serial.println(String(concatFlexValues, HEX));
        // Serial.println(concatFlexValues);
        xyzpCharacteristic.setValue(concatFlexValues);
        bendCharacteristic.setValue(nowRead2);

        // // Extract individual values from concatFlexValues

        uint8_t expattern = (concatFlexValues >> 24) & 0xFF;
        uint16_t extightSwellDataA3 = (concatFlexValues >> 16) & 0xFF;
        uint16_t exjointSwellDataA2 = (concatFlexValues >> 8) & 0xFF;
        uint16_t exangleDataA1 = concatFlexValues & 0xFF;
        // Serial.print(expattern);
        // Serial.print(",");
        // Serial.print(extightSwellDataA3);
        // Serial.print(",");
        // Serial.print(exjointSwellDataA2);
        // Serial.print(",");
        // Serial.println(exangleDataA1);

      } else {
        // do nothing
      }
    }
  }
  delay(50);
}

uint8_t compressTo8Bit(int inputValue) {
  if (inputValue <= 2000) {
    return 0;  // If below the range, return 0
  } else if (inputValue >= 3000) {
    return 255;  // If above the range, return 255
  } else {
    // Map the input range to the 8-bit range
    return map(inputValue, 2000, 3000, 0, 255);
  }
}

// StreamTransformer<List<List<int>>, List<int>> streamTransformer =
//       StreamTransformer.fromHandlers(handleData: (data, sink) {
//         final int concatFlexValues = data[0]; // Received 32-bit value
//         // Extract individual values from the received concatFlexValues
//         final int nowRead4 = (concatFlexValues >> 22) & 0x3FF; // Masking 10 bits
//         final int nowRead3 = (concatFlexValues >> 12) & 0x3FF; // Masking 10 bits
//         final int nowRead2 = (concatFlexValues >> 2) & 0x3FF; // Masking 10 bits
//         final int pattern = concatFlexValues & 0x3; // Masking 2 bits
//         sink.add([nowRead4, nowRead3, nowRead2, pattern]);
//   });
