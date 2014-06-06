#ifndef HELPERS_H
#define HELPERS_H

// ----------------------------------------------------------------------------

#include <Arduino.h>

// ----------------------------------------------------------------------------
// 

class ScopedTimer {
public:
  ScopedTimer(const char * Label)
    : label(Label), ts(millis())
  {
  }
  ~ScopedTimer() {
    Serial.print(label); Serial.print(": ");
    Serial.println(millis() - ts);
  }
private:
  const char *label;
  const unsigned long ts;
};


// ----------------------------------------------------------------------------
// data type conversion helpers

// use preventMinus if a decimal is converted
void itoa10(int32_t n, char *result, bool preventMinus = false) {
  uint32_t u;
  uint16_t i = 0;

  if (n < 0) { // for negative number, prepend '-' and invert
    if (!preventMinus) {
      result[0] = '-';
      result++;    
    }
    u = ((uint32_t) -(n + 1)) + 1;
  }
  else { 
    u = (uint32_t)n;
  }
  
  do {
    result[i++] = '0' + u % 10;
    u /= 10;
  } 
  while (u > 0);
  
  // rotate string bytewise
  for (uint16_t j = 0; j < i / 2; ++j) {
    char tmp = result[j];
    result[j] = result[i - j - 1];
    result[i - j - 1] = tmp;
  }
  result[i] = '\0';
}

void ftoa(char *a, double val, int precision) {
  int16_t ival = (int16_t)(val * 10);

  int16_t n = (int16_t)(ival / 10);
  itoa10(n, a); 
  while (*a != 0) a++; *a++ = '.';

  int16_t f = ival % 10;
  itoa10(f, a, true);
}

void itostr(char *r, int16_t val, char *unit = NULL) {
  char *p = r, *u = unit;
  *p++ = ' ';
  itoa10(val, p);
  while(*p != 0x00) p++;
  while(*u != 0x00) *p++ = *u++;
  *p = 0x00;
}

#endif // HELPERS_H