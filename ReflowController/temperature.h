#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <Arduino.h>
#include <SPI.h>

typedef struct Thermocouple {
  double temperature;
  int stat;
  int chipSelect;
};

void readThermocouple(struct Thermocouple* input) {
  #define LCD_CS   7
  uint8_t lcdState = digitalRead(LCD_CS);
  digitalWrite(LCD_CS, HIGH);

  digitalWrite(input->chipSelect, LOW);

  uint32_t result = 0x0000;
  byte reply = 0;
  char data = 0; // dummy data to write
  
  for (int i = 0; i < 4; i++) { // read the 32 data bits from the MAX31855
    reply = SPI.transfer(data);
    result = result << 8;
    result |= reply;
  }
  
  result >>= 18;

  uint16_t value = 0xFFF & result; // mask off the sign bit and shit to the correct alignment for the temp data    
  input->stat = reply & B111;  
  input->temperature = value * 0.25;

  digitalWrite(input->chipSelect, HIGH);
  digitalWrite(LCD_CS, lcdState);
}

#endif
