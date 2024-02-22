#include "Res.h"

Res::Res(uint8_t pin)
{
  _pin = pin;
}

int Res::Read(uint8_t smooth_degree)
{
  unsigned long start = micros();
  // change the resolution to 12 bits and read ADC_PIN
  analogReadResolution(12);

  int numIndex;
  int readVal = 0;

  for (numIndex = 0; numIndex < smooth_degree;) {

    // Read sensor data.
    if (micros() - start > 100 ) {

      // sum analogRead in a loop
      readVal = readVal + analogRead(_pin);

      // rest start timestamps
      start = millis();

      // 
      numIndex++;
    }
  }

  // Take an average of all the readings.
  readVal = readVal / smooth_degree;

  return readVal;
}
