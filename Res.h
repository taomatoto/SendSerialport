#ifndef RES_H
#define RES_H

#include <Arduino.h>

class Res
{
  public:
    Res(uint8_t pin);
    int Read(uint8_t smooth_degree);

    private:
      int _pin;
};

#endif /*RES_H*/
