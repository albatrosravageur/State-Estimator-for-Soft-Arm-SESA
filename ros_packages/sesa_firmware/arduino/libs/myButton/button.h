#include "Arduino.h"

class Button
{
public:
  uint8_t _pin_in;
  uint8_t _pin_out;
  int _state;
  long _last_hit;
  int _delta_millis = 200;

  void begin(uint8_t pin_in, uint8_t pin_out, int state = 0)
  {
    _pin_in = pin_in;
    _pin_out = pin_out;
    _state = state;
    _last_hit = millis();
    pinMode(_pin_in, INPUT);
    pinMode(_pin_out, OUTPUT);
    digitalWrite(_pin_out,_state);

  }
  bool hit()
  {
    if((digitalRead(_pin_in)==HIGH) && ((millis() - _last_hit) > _delta_millis))
      {
        _state = !_state;
        digitalWrite(_pin_out,_state);
        _last_hit = millis();
        return true;
      }
      else
      {
        return false;
      }
  }

  void read(){
    bool a;
    a = hit();
  }
};
