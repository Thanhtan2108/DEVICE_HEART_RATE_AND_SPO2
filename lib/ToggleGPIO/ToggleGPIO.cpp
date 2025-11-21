#include "ToggleGPIO.h"

void digitalTogglePin(uint8_t pinGPIO) {
    digitalWrite(pinGPIO, !(digitalRead(pinGPIO)));
    delay(500);
}
