// ----------------------------------------------------------------------------
// Reflow Oven Controller
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------

#include <avr/eeprom.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Menu.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include "temperature.h"
#include "helpers.h"

// ----------------------------------------------------------------------------

uint8_t crc8(uint8_t *data, uint16_t data_length);

// ----------------------------------------------------------------------------

const char * ver = "3.0-pit";

//#define WITH_SERIAL 1
//#define DEBUG

// ----------------------------------------------------------------------------
// Hardware Configuration 

// 1.8" TFT via SPI -> breadboard
//#define LCD_CS  10
//#define LCD_DC   7
//#define LCD_RST  8

// 1.8 TFT test handsoldered on v0 pcb
#define LCD_CS   7
#define LCD_DC   8
#define LCD_RST  9


// Thermocouples via SPI
#define THERMOCOUPLE1_CS  3

#define PIN_HEATER   0 // SSR for the heater
#define PIN_FAN      1 // SSR for the fan

#define PIN_ZX       2 // pin for zero crossing detector
#define INT_ZX       1 // interrupt for zero crossing detector
                       // Leonardo == Pro Micro:
                       //   Pin: 3 2 0 1 7
                       //   Int: 0 1 2 3 4 

// ----------------------------------------------------------------------------

volatile uint32_t timerTicks     = 0;
volatile uint32_t zeroCrossTicks = 0;
volatile uint8_t  phaseCounter   = 0;

char buf[15]; // generic char buffer

// ----------------------------------------------------------------------------
// Ensure that Solid State Relais are off when starting
//
void setupRelayPins(void) {
  DDRD  |= (1 << 2) | (1 << 3); // output
  PORTD &= ~((1 << 2) | (1 << 3));
}

void killRelayPins(void) {
  Timer1.stop();
  detachInterrupt(INT_ZX);
  PORTD |= (1 << 2) | (1 << 3);
}

// ----------------------------------------------------------------------------
// wave packet control: only turn the solid state relais on for a percentage 
// of complete sinusoids (i.e. 1x 360°)

#define CHANNELS       2
#define CHANNEL_HEATER 0
#define CHANNEL_FAN    1

typedef struct Channel_s {
  volatile uint8_t target; // percentage of on-time
  uint8_t state;           // current state counter
  int32_t next;            // when the next change in output shall occur  
  bool action;             // hi/lo active
  uint8_t pin;             // io pin of solid state relais
} Channel_t;

Channel_t Channels[CHANNELS] = {
  // heater
  { 0, 0, 0, false, 2 }, // PD2 == RX == Arduino Pin 0
  // fan
  { 0, 0, 0, false, 3 }  // PD3 == TX == Arduino Pin 1
};

// delay to align relay activation with the actual zero crossing
uint16_t zxLoopDelay = 0;

// calibrate zero crossing: how many timerIsr happen within one zero crossing
#define zxCalibrationLoops 25
struct {
  volatile uint8_t iterations;
  volatile uint8_t measure[zxCalibrationLoops];
} zxLoopCalibration = {
  zxCalibrationLoops, {}
};

// Zero Crossing ISR; per ZX, process one channel per interrupt only
// NB: use native port IO instead of digitalWrite for better performance
void zeroCrossingIsr(void) {
  static uint8_t ch = 0;

  // reset phase control timer
  phaseCounter = 0;
  TCNT1 = 0;

  zeroCrossTicks++;

  // calculate wave packet parameters
  Channels[ch].state += Channels[ch].target;
  if (Channels[ch].state >= 100) {
    Channels[ch].state -= 100;
    Channels[ch].action = false;
  }
  else {
    Channels[ch].action = true;
  }
  Channels[ch].next = timerTicks + zxLoopDelay;

  ch = ((ch + 1) % CHANNELS); // next channel

  if (zxLoopCalibration.iterations) {
    zxLoopCalibration.iterations--;
  }
}

// ----------------------------------------------------------------------------

// data type for the values used in the reflow profile
typedef struct profileValues_s {
  int16_t soakTemp;
  int16_t soakDuration;
  int16_t peakTemp;
  int16_t peakDuration;
  double  rampUpRate;
  double  rampDownRate;
  uint8_t checksum;
} Profile_t;

Profile_t activeProfile; // the one and only instance

int idleTemp = 50; // the temperature at which to consider the oven safe to leave to cool naturally
int fanAssistSpeed = 20; // default fan speed

// EEPROM offsets
const uint16_t offsetFanSpeed   = 30 * sizeof(Profile_t) + 1; // one byte
const uint16_t offsetProfileNum = 30 * sizeof(Profile_t) + 2; // one byte

int profileNumber = 0;
boolean thermocoupleOneActive = true; // this is used to keep track of which thermocouple input is used for control

Thermocouple A;

// ----------------------------------------------------------------------------

uint32_t startZeroCrossTicks;
uint32_t lastUpdate = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastSerialOutput = 0;

// ----------------------------------------------------------------------------
// UI

Adafruit_ST7735 tft = Adafruit_ST7735(LCD_CS, LCD_DC, LCD_RST);

/*
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0  
#define ST7735_WHITE   0xFFFF
*/

ClickEncoder Encoder(A1, A0, A2, 2);

Menu::Engine Engine;

int16_t encMovement;
int16_t encAbsolute;
int16_t encLastAbsolute = -1;
bool encLastAccelerationState = true;

bool updateMenu = true;
bool headlineRendered = false;

const uint8_t menuItemsVisible = 6;
const uint8_t menuItemHeight = 12;

// track menu item state to improve render preformance
typedef struct {
  const Menu::Item_t *mi;
  uint8_t pos;
  bool current;
} LastItemState_t;

LastItemState_t previousItems[menuItemsVisible];

// ----------------------------------------------------------------------------
// state machine

typedef enum {
  None     = 0,
  Idle     = 1,
  Settings = 2,
  Edit     = 3,

  UIMenuEnd = 9,

  RampToSoak = 10,
  Soak,
  RampUp,
  Peak,
  RampDown,
  CoolDown,

  Complete = 20
} State;

State currentState  = Idle;
State previousState = Idle;
bool stateChanged = false;
uint32_t stateChangedTicks = 0;

// ----------------------------------------------------------------------------

// must be cleared at enter/exit / function
void clearLastMenuItemRenderState() {
  for (uint8_t i = 0; i < menuItemsVisible; i++) {
    previousItems[i].mi = NULL;
  }
}

// ----------------------------------------------------------------------------

extern const Menu::Item_t miRampUpRate, miRampDnRate, miSoakTime, 
                          miSoakTemp, miPeakTime, miPeakTemp,
                          miLoadProfile, miSaveProfile;

// ----------------------------------------------------------------------------

void getEditItemValue(const Menu::Item_t *mi, double **d, int16_t **i) {
  if (mi == &miRampUpRate) *d = &activeProfile.rampUpRate;
  if (mi == &miRampDnRate) *d = &activeProfile.rampDownRate;
  if (mi == &miSoakTime)   *i = &activeProfile.soakDuration;
  if (mi == &miSoakTemp)   *i = &activeProfile.soakTemp;
  if (mi == &miPeakTime)   *i = &activeProfile.peakDuration;
  if (mi == &miPeakTemp)   *i = &activeProfile.peakTemp;
}

// ----------------------------------------------------------------------------

bool getEditItemLabel(const Menu::Item_t *mi, char *label) {
  int16_t *iValue = NULL;
  double  *dValue = NULL;
  char *p;

  getEditItemValue(mi, &dValue, &iValue);

  if (mi == &miRampUpRate || mi == &miRampDnRate) {
    p = label;
    *p++ = ' ';
    ftoa(p, *dValue, 1);
    while(*p != 0x00) p++;
    *p++ = 0xf7; *p++ = 'C'; *p++ = '/'; *p++ = 's';
    *p = 0x00;
  }
  else {
    if (mi == &miPeakTemp || mi == &miSoakTemp) {
      itostr(label, *iValue, "\367C");
    }
    if (mi == &miPeakTime || mi == &miSoakTime) {
      itostr(label, *iValue, "s");
    }
  }

  return dValue || iValue;
}

// ----------------------------------------------------------------------------

bool menuExit(const Menu::Action_t a) {
  Encoder.setAccelerationEnabled(encLastAccelerationState);  
  clearLastMenuItemRenderState();
  Engine.lastInvokedItem = &Menu::NullItem;
  updateMenu = false;
  return false;
}

// ----------------------------------------------------------------------------

bool menuDummy(const Menu::Action_t a) {
  return true;
}

// ----------------------------------------------------------------------------

void printDouble(double val, uint8_t precision = 2) {
  ftoa(buf, val, precision);
  tft.print(buf);
}

// ----------------------------------------------------------------------------

bool editProfileValue(const Menu::Action_t action) {
  int16_t *iValue = NULL;
  double  *dValue = NULL;

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    getEditItemValue(Engine.currentItem, &dValue, &iValue);

    if (initial) {
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      tft.setCursor(10, 80);
      tft.print("Edit & click to save.");

      encLastAccelerationState = Encoder.getAccelerationEnabled();
      Encoder.setAccelerationEnabled(true);
    }

    for (uint8_t i = 0; i < menuItemsVisible; i++) {
      if (previousItems[i].mi == Engine.currentItem) {
        uint8_t y = previousItems[i].pos * menuItemHeight + 2;

        if (initial) {
          tft.fillRect(69, y - 1, 60, menuItemHeight - 2, ST7735_RED);
        }

        tft.setCursor(70, y);
        break;
      }
    }

    tft.setTextColor(ST7735_WHITE, ST7735_RED);

    if (Engine.currentItem == &miRampUpRate || Engine.currentItem == &miRampDnRate) {
      double tmp;
      if (initial) {
        tmp = *dValue;
        tmp *= 10.0;
        encAbsolute = (int16_t)tmp;
      }
      else {
        tmp = encAbsolute;
        tmp /= 10;
        *dValue = tmp;
      }
      printDouble(*dValue);
      tft.print("\367C/s");
    }
    else {
      if (initial) encAbsolute = *iValue;
      else *iValue = encAbsolute;
      tft.print(*iValue);
      if (Engine.currentItem == &miPeakTemp || Engine.currentItem == &miSoakTemp) {
        tft.print("\367C");
      }
      if (Engine.currentItem == &miPeakTime || Engine.currentItem == &miSoakTime) {
        tft.print("s");
      }
    }

    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
  }

  if (action == Menu::actionTrigger) {  // click on already active item
    clearLastMenuItemRenderState();
    Engine.navigate(Engine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      Encoder.setAccelerationEnabled(encLastAccelerationState);
      return false;
    }
  }

  return true;
}

// ----------------------------------------------------------------------------

bool factoryReset(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    if (initial) {
      tft.setTextColor(ST7735_BLACK, ST7735_WHITE);
      tft.setCursor(10, 80);
      tft.print("Click to confirm");
      tft.setCursor(10, 90);
      tft.print("Doubleclick to exit");
    }
  }

  if (action == Menu::actionTrigger) { // do it
    factoryReset();
    clearLastMenuItemRenderState();
    Engine.navigate(Engine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }
}

// ----------------------------------------------------------------------------

bool saveLoadProfile(const Menu::Action_t action) {
  bool isLoad = Engine.currentItem == &miLoadProfile;

  if (action == Menu::actionDisplay) {
    bool initial = currentState != Edit;
    currentState = Edit;

    tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

    if (initial) {
      encAbsolute = profileNumber;      
      tft.setCursor(10, 90);
      tft.print("Doubleclick to exit");
    }

    if (encAbsolute > 30) encAbsolute = 30;
    if (encAbsolute <  0) encAbsolute =  0;

    tft.setCursor(10, 80);
    tft.print("Click to ");
    tft.print((isLoad) ? "load " : "save ");
    tft.setTextColor(ST7735_WHITE, ST7735_RED);
    tft.print(encAbsolute);
  }

  if (action == Menu::actionTrigger) { // do it
    if (isLoad) {
      loadProfile(encAbsolute);
    }
    else {
      saveProfile(encAbsolute);
    }
    clearLastMenuItemRenderState();
    Engine.navigate(Engine.getParent());
    return false;
  }

  if (action == Menu::actionParent) {    
    if (currentState == Edit) { // leave edit mode only, returning to menu
      currentState = Settings;
      clearLastMenuItemRenderState();
      return false;
    }
  }
}

// ----------------------------------------------------------------------------

bool cycleStart(const Menu::Action_t action) {
  if (action == Menu::actionDisplay) {
    startZeroCrossTicks = zeroCrossTicks;
    menuExit(action);
    currentState = RampToSoak;
    headlineRendered = false;
    updateMenu = false;
  }
}

// ----------------------------------------------------------------------------

bool fanSettings(const Menu::Action_t action) {
}

// ----------------------------------------------------------------------------

void renderMenuItem(const Menu::Item_t *mi, uint8_t pos) {
  //ScopedTimer tm("  render menuitem");
  bool isCurrent = Engine.currentItem == mi;
  uint8_t y = pos * menuItemHeight + 2;

  if (previousItems[pos].mi == mi 
      && previousItems[pos].pos == pos 
      && previousItems[pos].current == isCurrent) 
  {
    return; // don't render the same item in the same state twice
  }

  tft.setCursor(10, y);

  // menu cursor bar
  tft.fillRect(8, y - 2, tft.width() - 16, menuItemHeight, isCurrent ? ST7735_BLUE : ST7735_WHITE);

  if (isCurrent) tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  else tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  tft.print(Engine.getLabel(mi));

  if (getEditItemLabel(mi, buf)) {
    tft.print(buf);
  }
 
  // mark items that have children
  if (Engine.getChild(mi) != &Menu::NullItem) {
    tft.print(" \x10   ");
  }

  previousItems[pos].mi = mi;
  previousItems[pos].pos = pos;
  previousItems[pos].current = isCurrent;
}

// ----------------------------------------------------------------------------
// NB: Adafruit GFX ASCII-Table is bogous: https://github.com/adafruit/Adafruit-GFX-Library/issues/22
// Name, Label, Next, Previous, Parent, Child, Callback

MenuItem(miExit, "", Menu::NullItem, Menu::NullItem, Menu::NullItem, miCycleStart, menuExit);

MenuItem(miCycleStart,  "Start Cycle",  miEditProfile, Menu::NullItem, miExit, Menu::NullItem, cycleStart);
MenuItem(miEditProfile, "Edit Profile", miLoadProfile, miCycleStart,   miExit, miRampUpRate, menuDummy);
  MenuItem(miRampUpRate, "Ramp up  ",   miSoakTemp,      Menu::NullItem, miEditProfile, Menu::NullItem, editProfileValue);
  MenuItem(miSoakTemp,   "Soak temp", miSoakTime,      miRampUpRate,   miEditProfile, Menu::NullItem, editProfileValue);
  MenuItem(miSoakTime,   "Soak time", miPeakTemp,      miSoakTemp,     miEditProfile, Menu::NullItem, editProfileValue);
  MenuItem(miPeakTemp,   "Peak temp", miPeakTime,      miSoakTime,     miEditProfile, Menu::NullItem, editProfileValue);
  MenuItem(miPeakTime,   "Peak time", miRampDnRate,    miPeakTemp,     miEditProfile, Menu::NullItem, editProfileValue);
  MenuItem(miRampDnRate, "Ramp down", Menu::NullItem,  miPeakTime,     miEditProfile, Menu::NullItem, editProfileValue);
MenuItem(miLoadProfile,  "Load Profile",  miSaveProfile,  miEditProfile, miExit, Menu::NullItem, saveLoadProfile);
MenuItem(miSaveProfile,  "Save Profile",  miFanSettings,  miLoadProfile, miExit, Menu::NullItem, saveLoadProfile);
MenuItem(miFanSettings,  "Fan Settings",  miFactoryReset, miLoadProfile, miExit, Menu::NullItem, menuDummy);
MenuItem(miFactoryReset, "Factory Reset", Menu::NullItem, miFanSettings, miExit, Menu::NullItem, factoryReset);

// ----------------------------------------------------------------------------

double Setpoint;
double Input;
double Output;

// Define the PID tuning parameters
// TODO: make these configurable
// TODO: add PID autotune
/*
  typedef struct {
    double Kp;
    double Ki;
    double Kd;
  } PID_t;

  PID_t headerPID = {
    4.00,
    0.05,
    2.00
  };

  PID_t fanPID = {
     1.00,
     0.03,
    10.00
  };
*/

double Kp = 4,
       Ki = 0.05,
       Kd = 2;

double fanKp =  1,
       fanKi =  0.03,
       fanKd = 10;

PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

uint8_t fanValue;
uint8_t heaterValue;

// ----------------------------------------------------------------------------

#define NUMREADINGS 10

double airTemp[NUMREADINGS];
double readingsT1[NUMREADINGS]; // the readings used to make a stable temp rolling average
double runningTotalRampRate;
double rampRate = 0;
double rateOfRise = 0;          // the result that is displayed
double totalT1 = 0;             // the running total
double averageT1 = 0;           // the average
uint8_t index = 0;              // the index of the current reading

// ----------------------------------------------------------------------------
// timer interrupt handling

// ticks with 100µS
void timerIsr(void) {
  static uint32_t lastTicks = 0;

  // phase control for the fan 
  if (++phaseCounter > 90) {
    phaseCounter = 0;
  }

  if (phaseCounter > Channels[CHANNEL_FAN].target) {
    PORTD &= ~(1 << Channels[CHANNEL_FAN].pin);
  }
  else {
    PORTD |=  (1 << Channels[CHANNEL_FAN].pin);
  }

  // wave packet control for heater
  if (Channels[CHANNEL_HEATER].next > lastTicks // FIXME: this looses ticks when overflowing
      && timerTicks > Channels[CHANNEL_HEATER].next) 
  {
    if (Channels[CHANNEL_HEATER].action) PORTD |= (1 << Channels[CHANNEL_HEATER].pin);
    else PORTD &= ~(1 << Channels[CHANNEL_HEATER].pin);
    lastTicks = timerTicks;
  }

  // handle encoder + button
  if (!(timerTicks % 10)) {
    Encoder.service();
  }

  timerTicks++;

  if (zxLoopCalibration.iterations) {
    zxLoopCalibration.measure[zxLoopCalibration.iterations]++;
  }
}

// ----------------------------------------------------------------------------

void abortWithError(int error) {
  killRelayPins();

  tft.setTextColor(ST7735_WHITE, ST7735_RED);
  tft.fillScreen(ST7735_RED);

  tft.setCursor(10,10);
  tft.setTextSize(2);
  tft.print(error);
/*
  lcd.clear();

  switch (error) {
    case 1:
      lcd.print("Temperature"); 
      lcd.setCursor(0,1);
      lcd.print("following error");
      lcd.setCursor(0,2);
      lcd.print("during heating");
      break;
    case 2:
      lcd.print("Temperature"); 
      lcd.setCursor(0,1);
      lcd.print("following error");
      lcd.setCursor(0,2);
      lcd.print("during cooling");
      break;
    case 3:
      lcd.print("Thermocouple input"); 
      lcd.setCursor(0,1);
      lcd.print("open circuit");
      lcd.setCursor(0,2);
      lcd.print("Power off &");
      lcd.setCursor(0,3);
      lcd.print("check connections");
      break;
  }
*/

  while (1) { // and stop forever...
    ;
  }
}

// ----------------------------------------------------------------------------

void displayThermocoupleData(struct Thermocouple* input) {
  switch (input->stat) {
    case 0:
      printDouble(input->temperature);
      tft.print("\367C");
      break;
    case 1:
      tft.print("---");
      break;
  }
}

// ----------------------------------------------------------------------------

void alignRightPrefix(uint16_t v) {
  //if (v < 1e3) tft.print(' '); 
  if (v < 1e2) tft.print(' '); 
  if (v < 1e1) tft.print(' ');
}

uint16_t pxPerS, pxPerC;  // TODO use activeProfile.peakTemp + 20%

void updateProcessDisplay() {
  const uint8_t h =  86;
  const uint8_t w = 160;
  uint8_t yOffset =  30; // space not available for graph
  uint16_t xOffset =  0; // for wraparound on x axis
  uint16_t dx, dy;
  uint8_t y = 2;
  double tmp;

  // header & initial view
  tft.setTextColor(ST7735_WHITE, ST7735_BLUE);
  if (!headlineRendered) {
    headlineRendered = true;

    tft.fillScreen(ST7735_WHITE);
    tft.fillRect(0, 0, tft.width(), menuItemHeight, ST7735_BLUE);
    tft.setCursor(2, y);
    tft.print("Profile ");
    tft.print(profileNumber);

    tmp = h / (activeProfile.peakTemp * 1.10) * 100.0;
    pxPerC = (uint16_t)tmp;
    
#if 0 // FIXME
    double estimatedTotalTime = 60 * 12;
    // estimate total run time for current profile
    estimatedTotalTime = activeProfile.soakDuration + activeProfile.peakDuration;
    estimatedTotalTime += (activeProfile.soakTemp - 20.0) / (activeProfile.rampUpRate / 10);
    estimatedTotalTime += (activeProfile.peakTemp - activeProfile.soakTemp) / (activeProfile.rampUpRate / 10);
    estimatedTotalTime += (activeProfile.peakTemp - 20.0) / (activeProfile.rampDownRate  / 10);
    //estimatedTotalTime *= 2; // add some spare
    Serial.print("total est. time: ");
    Serial.println((uint16_t)estimatedTotalTime);
#endif
    tmp = 60 * 15; // FIXME should be calculated
    tmp = w / tmp * 10.0; 
    pxPerS = (uint16_t)tmp;

    // 50°C grid
    int16_t t = (uint16_t)(activeProfile.peakTemp * 1.10);
    for (uint16_t tg = 0; tg < t; tg += 50) {
      uint16_t l = h - (tg * pxPerC / 100) + yOffset;
      tft.drawFastHLine(0, l, 160, tft.Color565(0xe0, 0xe0, 0xe0));
    }
#ifdef GRAPH_VERBOSE
    Serial.print("Calc pxPerC/S: ");
    Serial.print(pxPerC);
    Serial.print("/");
    Serial.println(pxPerS);
#endif
  }

  // elapsed time
  uint16_t elapsed = (zeroCrossTicks - startZeroCrossTicks) / 100;
  tft.setCursor(120, y);
  alignRightPrefix(elapsed); 
  tft.print(elapsed);
  tft.print("s");

  y += menuItemHeight + 2;

  tft.setCursor(2, y);
  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // temperature
  tft.setTextSize(2);
  alignRightPrefix((int)A.temperature);
  displayThermocoupleData(&A);
  tft.setTextSize(1);

  // current state
  y -= 2;
  tft.setCursor(95, y);
  tft.setTextColor(ST7735_BLACK, ST7735_GREEN);
  
  switch (currentState) {
    #define casePrintState(state) case state: tft.print(#state); break;
    casePrintState(RampToSoak);
    casePrintState(Soak);
    casePrintState(RampUp);
    casePrintState(Peak);
    casePrintState(RampDown);
    casePrintState(CoolDown);
    casePrintState(Complete);
    default: tft.print((uint8_t)currentState); break;
  }
  tft.print("         "); // lazy: fill up space

  tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

  // set point
  y += 10;
  tft.setCursor(95, y);
  tft.print("Sp:"); 
  alignRightPrefix((int)Setpoint); 
  printDouble(Setpoint);
  tft.print("\367C  ");


  // draw temperature curves
  //
again:
  dx = ((elapsed - xOffset) * pxPerS) / 10;
  if (dx > w) { // wrap around
    xOffset = elapsed - 1;
    goto again; // goto: i did it!
  }

#ifdef GRAPH_VERBOSE
  Serial.print(elapsed); Serial.print("="); Serial.print(dx); Serial.print(", ");
#endif

  // temperature setpoint
  dy = h - ((uint16_t)Setpoint * pxPerC / 100) + yOffset;
  tft.drawPixel(dx, dy, ST7735_BLUE);
#ifdef GRAPH_VERBOSE
  Serial.print((uint16_t)Setpoint); Serial.print("="); Serial.print(dy); Serial.print(",");
#endif

  // actual temperature
  dy = h - ((uint16_t)A.temperature * pxPerC / 100) + yOffset;
  tft.drawPixel(dx, dy, ST7735_RED);
#ifdef GRAPH_VERBOSE
  Serial.print((uint16_t)A.temperature); Serial.print("="); Serial.println(dy);
#endif

  // bottom line
  y = 119;

  // set values
  tft.setCursor(2, y);
  tft.print("H ");
  tft.print((int)heaterValue);
  tft.print('%');

  //tft.setCursor(40, y);
  tft.print(" F ");
  tft.print((int)fanValue);
  tft.print('%');

  //tft.setCursor(80, y);
  tft.print(" R ");
  printDouble(rampRate);
  tft.print("\367C/s   ");
}

// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);

  setupRelayPins();

  A.chipSelect = THERMOCOUPLE1_CS;

  //tft.initR(INITR_REDTAB);
  tft.initR(INITR_BLACKTAB);
  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setRotation(3);

  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  } 
  else {
    loadLastUsedProfile();
  }

  loadFanSpeed();

  PID.SetOutputLimits(0, 100); // max output 100%
  PID.SetMode(AUTOMATIC);

  // setup /CS line for Thermocouple
  digitalWrite(A.chipSelect, HIGH);
  pinMode(A.chipSelect, OUTPUT);

  readThermocouple(&A);
  if (A.stat != 0) {
    abortWithError(3);
  }

  // initialize moving average filter
  runningTotalRampRate = A.temperature * NUMREADINGS;
  for(int i = 0; i < NUMREADINGS; i++) {
    airTemp[i] = A.temperature; 
  }

  Timer1.initialize(100);
  Timer1.attachInterrupt(timerIsr);

  pinMode(PIN_ZX, INPUT_PULLUP);
  attachInterrupt(INT_ZX, zeroCrossingIsr, RISING);
  delay(100);

#ifdef SPLASH // fixme
  tft.fillScreen(ST7735_WHITE);
  tft.print(" Reflow controller");
  tft.setCursor(0, 1);
  tft.print("      v"); lcd.print(ver);
#endif

#ifndef FAKE // autocalibrate zero cross timing delay
  tft.fillScreen(ST7735_WHITE);
  tft.setCursor(10, 10);
  
  tft.print("Calibrating... ");
  delay(500);

  while (zxLoopDelay == 0) {
    if (zxLoopCalibration.iterations < 1) {
      for (uint8_t l = 0; l < zxCalibrationLoops; l++) {
        zxLoopDelay += zxLoopCalibration.measure[l];
      }
      zxLoopDelay /= zxCalibrationLoops;
      zxLoopDelay -= 6; // calibration offset: loop runtime
    }
  }

  tft.print(zxLoopDelay);
  delay(1000);
#endif

  menuExit(Menu::actionDisplay); // reset to initial state
  Engine.navigate(&miCycleStart);
  currentState = Settings;
  updateMenu = true;
}

// ----------------------------------------------------------------------------
/* moving average
    int samples[8];

    total -= samples[i];
    samples[i] = A.temperature; // new value
    total += samples[i];

    i = (i + 1) % 8; // next position
    average = total >> 3; // == div by 8 */
// ----------------------------------------------------------------------------

uint32_t lastFakeZXTick = 0;

void loop(void)
{
#ifdef FAKE
  if (timerTicks > lastFakeZXTick + 50) {
      lastFakeZXTick = timerTicks;
      zeroCrossTicks+=50;
  }
#endif

  // --------------------------------------------------------------------------
  // handle encoder
  //
  encMovement = Encoder.getValue();
  if (encMovement) {
    encAbsolute += encMovement;
    if (currentState == Settings) {
      Engine.navigate((encMovement > 0) ? Engine.getNext() : Engine.getPrev());
      updateMenu = true;
    }
  }

  // --------------------------------------------------------------------------
  // handle button
  //
  switch (Encoder.getButton()) {
    case ClickEncoder::Clicked:
      if (currentState == Complete) { // at end of cycle; reset at click
        menuExit(Menu::actionDisplay); // reset to initial state
        Engine.navigate(&miCycleStart);
        currentState = Settings;
        updateMenu = true;
      }
      else if (currentState < UIMenuEnd) {
        updateMenu = true;
        Engine.invoke();
      }
      break;

    case ClickEncoder::DoubleClicked:
      if (currentState < UIMenuEnd) {
        Engine.navigate(Engine.getParent());
        updateMenu = true;
      }
      break;

    case ClickEncoder::Held:
      if (currentState < UIMenuEnd) { // enter settings menu // FIXME not always good
        tft.fillScreen(ST7735_WHITE);
        tft.setTextColor(ST7735_BLACK, ST7735_WHITE);

        Engine.navigate(&miCycleStart);
        currentState = Settings;
        updateMenu = true;
      } 
      break;
  }

  // --------------------------------------------------------------------------
  // update current menu item while in edit mode
  //
  if (currentState == Edit) {
    if (Engine.currentItem != &Menu::NullItem) {
      Engine.executeCallbackAction(Menu::actionDisplay);      
    }
  }

  // --------------------------------------------------------------------------
  // handle menu update
  //
  if (updateMenu) {
    updateMenu = false;
    // Serial.print("update menu at state transfer ");
    // Serial.print(previousState);
    // Serial.print(" -> ");
    // Serial.println(currentState);
    
    if (currentState < UIMenuEnd && !encMovement && currentState != Edit) { // clear menu on child/parent navigation
      tft.fillScreen(ST7735_WHITE);
    }  
    Engine.render(renderMenuItem, menuItemsVisible);
  }

  // --------------------------------------------------------------------------
  // track state changes
  //
  if (currentState != previousState) {
    stateChangedTicks = zeroCrossTicks;
    stateChanged = true;

    // Serial.print("State change: ");
    // Serial.print(previousState);
    // Serial.print(" to ");
    // Serial.print(currentState);
    // Serial.print(" at ");
    // Serial.println(stateChangedTicks);

    previousState = currentState;
  }

  // --------------------------------------------------------------------------

  if (zeroCrossTicks - lastUpdate >= 10) {
    lastUpdate = zeroCrossTicks;

    readThermocouple(&A);
    if (A.stat != 0) {
      abortWithError(3);
    }
  
    // rolling average of the temp T1 and T2
    totalT1 -= readingsT1[index];       // subtract the last reading
    readingsT1[index] = A.temperature;
    totalT1 += readingsT1[index];       // add the reading to the total
    index = (index + 1) % NUMREADINGS;  // next position
    averageT1 = totalT1 / NUMREADINGS;  // calculate the average temp

    // need to keep track of a few past readings in order to work out rate of rise
    for (int i = 1; i < NUMREADINGS; i++) { // iterate over all previous entries, moving them backwards one index
      airTemp[i - 1] = airTemp[i];
    }

    airTemp[NUMREADINGS - 1] = averageT1; // update the last index with the newest average

    // calculate rate of rise in degrees per polling cycle time/ num readings
    rampRate = (airTemp[NUMREADINGS - 1] - airTemp[0]); // subtract earliest reading from the current one
    Input = airTemp[NUMREADINGS - 1]; // update the variable the PID reads


    // display update
    if (zeroCrossTicks - lastDisplayUpdate > 25) {
      lastDisplayUpdate = zeroCrossTicks;
      if (currentState > UIMenuEnd /*!= Idle && currentState != Settings && currentState != Edit*/) {
        updateProcessDisplay();
      }
    }

    #if WITH_SERIAL
    #endif

    switch (currentState) {
      case RampToSoak:
        if (stateChanged) {
          stateChanged = false;
          PID.SetMode(MANUAL);
          Output = 50;
          PID.SetMode(AUTOMATIC);
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(Kp, Ki, Kd);
          Setpoint = airTemp[NUMREADINGS - 1];
        }

        Setpoint += (activeProfile.rampUpRate / 10); // target set ramp up rate

        if (Setpoint >= activeProfile.soakTemp - 1) {
          currentState = Soak;
        }
        break;

      case Soak:
        if (stateChanged) {
          stateChanged = false;
          Setpoint = activeProfile.soakTemp;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.soakDuration * 100) {
          currentState = RampUp;
        }
        break;

      case RampUp:
        if (stateChanged) {
          stateChanged = false;
        }

        Setpoint += (activeProfile.rampUpRate / 10); // target set ramp up rate

        if (Setpoint >= activeProfile.peakTemp - 1) { // seems to take arodun 8 degrees rise to tail off to 0 rise
          Setpoint = activeProfile.peakTemp;
          currentState = Peak;
        }
        break;

      case Peak:
        if (stateChanged) {
          Setpoint = activeProfile.peakTemp;
          stateChanged = false;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.peakDuration * 100) {
          currentState = RampDown;
        }
        break;

      case RampDown:
        if (stateChanged) {
          stateChanged = false;
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          Setpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
        }

        Setpoint -= (activeProfile.rampDownRate / 10); 

        if (Setpoint <= idleTemp) {
          currentState = CoolDown;
        }
        break;

      case CoolDown:
        if (stateChanged) {
          stateChanged = false;
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          Setpoint = idleTemp;
        }

        if (Input < (idleTemp + 5)) {
          currentState = Complete;
          PID.SetMode(MANUAL);
          Output = 0;
        }
    }
  }

  // safety check that we're not doing something stupid. 
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.
  // both of these errors are blocking and do not exit!
  //if (Setpoint > Input + 50) abortWithError(1); // if we're 50 degree cooler than setpoint, abort
  //if (Input > Setpoint + 50) abortWithError(2); // or 50 degrees hotter, also abort

  PID.Compute();

  // decides which control signal is fed to the output for this cycle
  if (   currentState != RampDown
      && currentState != CoolDown
      && currentState != Settings
      && currentState != Complete
      && currentState != Idle
      && currentState != Settings
      && currentState != Edit)
  {
    heaterValue = Output;
    fanValue = fanAssistSpeed;
  } 
  else {
    heaterValue = 0;
    fanValue = Output;
  }

  Channels[CHANNEL_HEATER].target = heaterValue;

  double fanTmp = 90.0 / 100.0 * fanValue; // 0-100% -> 0-90° phase control
  Channels[CHANNEL_FAN].target = 90 - (uint8_t)fanTmp;
}


void saveProfile(unsigned int targetProfile) {
  profileNumber = targetProfile;

  tft.fillScreen(ST7735_GREEN);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(10, 50);
  tft.print("Saving profile ");
  tft.print(profileNumber);

#ifdef DEBUG
#endif

  saveParameters(profileNumber); // profileNumber is modified by the menu code directly, this method is called by a menu action

  delay(500); 
}

void loadProfile(unsigned int targetProfile) {
  tft.fillScreen(ST7735_GREEN);
  tft.setTextColor(ST7735_BLACK);
  tft.setCursor(10, 50);
  tft.print("Loading profile ");
  tft.print(targetProfile);

#ifdef DEBUG
#endif

  bool ok = loadParameters(targetProfile);

#ifdef DEBUG
#endif

  if (!ok) {
#if 0
    lcd.setCursor(0, 2);
    lcd.print("Checksum error!");
    lcd.setCursor(0, 3);
    lcd.print("Review profile.");
    delay(2500);
#endif
  }

  // save in any way, we have no undo
  profileNumber = targetProfile;
  saveLastUsedProfile();

  delay(500);
}

bool saveParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(Profile_t);

  activeProfile.checksum = crc8((uint8_t *)&activeProfile, sizeof(Profile_t) - sizeof(uint8_t));

  do {} while (!(eeprom_is_ready()));
  eeprom_write_block(&activeProfile, (void *)offset, sizeof(Profile_t));

  return true;
}

bool loadParameters(uint8_t profile) {
  uint16_t offset = profile * sizeof(Profile_t);

  do {} while (!(eeprom_is_ready()));
  eeprom_read_block(&activeProfile, (void *)offset, sizeof(Profile_t));

  return activeProfile.checksum == crc8((uint8_t *)&activeProfile, sizeof(Profile_t) - sizeof(uint8_t));
}

bool firstRun() { 
  // if all bytes of a profile in the middle of the eeprom space are 255, we assume it's a first run
  unsigned int offset = 15 * sizeof(Profile_t);
  for (uint16_t i = offset; i < offset + sizeof(Profile_t); i++) {
    if (EEPROM.read(i) != 255) {
      return false;
    }
  }

#if 0
  lcd.clear();
  lcd.print("First run...");
  delay(500);
#endif
  return true;
}

void makeDefaultProfile(void) {
  activeProfile.soakTemp     = 130;
  activeProfile.soakDuration =  80;
  activeProfile.peakTemp     = 220;
  activeProfile.peakDuration =  40;
  activeProfile.rampUpRate   =   0.80;
  activeProfile.rampDownRate =   2.0;
}

void factoryReset() {
  makeDefaultProfile();

  tft.fillScreen(ST7735_RED);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(10, 50);
  tft.print("Resetting...");

  // then save the same profile settings into all slots
  for (uint8_t i = 0; i < 30; i++) {
    saveParameters(i);
  }

  fanAssistSpeed = 50;
  saveFanSpeed();

  profileNumber = 0;
  saveLastUsedProfile();

  delay(500);
}

void saveFanSpeed() {
#if 0
  lcd.clear();
  lcd.print("Saving...");
#endif
  EEPROM.write(offsetFanSpeed, (uint8_t)fanAssistSpeed & 0xff);
  delay(250);
}

void loadFanSpeed() {
  fanAssistSpeed = EEPROM.read(offsetFanSpeed) & 0xff;
}

void saveLastUsedProfile() {
  EEPROM.write(offsetProfileNum, (uint8_t)profileNumber & 0xff);
}

void loadLastUsedProfile() {
  profileNumber = EEPROM.read(offsetProfileNum) & 0xff;
  loadParameters(profileNumber);
}

// ---------------------------------------------------------------------------- 
// crc8
//
// Copyright (c) 2002 Colin O'Flynn
// Minor changes by M.Thomas 9/2004 
// ----------------------------------------------------------------------------

#define CRC8INIT    0x00
#define CRC8POLY    0x18              //0X18 = X^8+X^5+X^4+X^0

// ----------------------------------------------------------------------------

uint8_t crc8(uint8_t *data, uint16_t data_length) {
  uint8_t  b;
  uint8_t  bit_counter;
  uint8_t  crc = CRC8INIT;
  uint8_t  feedback_bit;
  uint16_t loop_count;
  
  for (loop_count = 0; loop_count != data_length; loop_count++) {
    b = data[loop_count];
    bit_counter = 8;

    do {
      feedback_bit = (crc ^ b) & 0x01;

      if (feedback_bit == 0x01) {
        crc = crc ^ CRC8POLY;
      }

      crc = (crc >> 1) & 0x7F;

      if (feedback_bit == 0x01) {
        crc = crc | 0x80;
      }

      b = b >> 1;
      bit_counter--;

    } while (bit_counter > 0);
  }
  
  return crc;
}
