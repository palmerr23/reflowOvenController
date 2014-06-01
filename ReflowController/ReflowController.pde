// ----------------------------------------------------------------------------
// Reflow Oven Controller
// (c) 2014 Karl Pitrich <karl@pitrich.com>
// (c) 2012-2013 Ed Simmons
// ----------------------------------------------------------------------------

#include <avr/eeprom.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

#include <MenuBase.h>
#include <LCDMenu.h>
#include <MenuItemSubMenu.h>
#include <MenuItemSelect.h>
#include <MenuItemInteger.h>
#include <MenuItemDouble.h>
#include <MenuItemAction.h>
#include <MenuItemIntegerAction.h>
#include <TimerOne.h>
#include <ClickEncoder.h>
#include "lcdchars.h"

// ----------------------------------------------------------------------------

uint8_t crc8(uint8_t *data, uint16_t number_of_bytes_in_data);

// ----------------------------------------------------------------------------

const char * ver = "3.0-pit";

// ----------------------------------------------------------------------------
// Hardware Configuration 

// TODO use Arduino SPI.h
// SPI Bus
#define DATAOUT     14 // MOSI
#define SPICLOCK    15 // SCK

#define LCD_RW    -1
#define LCD_RS     5
#define LCD_EN     6
#define LCD_D4     7
#define LCD_D5     8
#define LCD_D6     9
#define LCD_D7    10
#define LCD_CHARS 20
#define LCD_LINES  4

#define THERMOCOUPLE1_CS  3
#define THERMOCOUPLE2_CS  4

#define BUTTON_STOP      -1

#define PIN_HEATER   0
#define PIN_FAN      1

#define PIN_ZX   2 // pin for zero crossing detector
#define INT_ZX   1 // interrupt for zero crossing detector
                   // Leonardo == Pro Micro:
                   //   Pin: 3 2 0 1 7
                   //   Int: 0 1 2 3 4 

//#define WITH_SERIAL 1
//#define DEBUG

// ----------------------------------------------------------------------------

volatile uint32_t timerTicks     = 0;
volatile uint32_t zeroCrossTicks = 0;
volatile uint8_t  phaseCounter   = 0;

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
// of complete sinusoids (i.e. 1x 360°) only
// think of this as a PWM for single sinusoid

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

// timer to align activation with the actual zero crossing
uint16_t zxLoopDelay = 0; // a value of 89 has been deduced empirically by aligning zero cross and SSR output

// calibrate zero crossing: how many timerIsr happen within one zero crossing
#define zxCalibrationLoops 25
struct {
  volatile uint8_t iterations;
  volatile uint8_t measure[zxCalibrationLoops];
} zxLoopCalibration = {
  zxCalibrationLoops, {}
};

// Zero Crossing ISR; per ZX, process one channel only
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

#include "temperature.h"

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

tcInput A, B; // the two structs for thermocouple data

// ----------------------------------------------------------------------------
// UI

LiquidCrystal lcd(LCD_RS, 
#if LCD_RW >= 0 // RW is not necessary if lcd is on dedicated pins
  LCD_RW,
#endif
  LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7
);

LCDMenu myMenu;

MenuItemAction control;

MenuItemSubMenu profile;
  MenuItemDouble rampUp_rate; 
  MenuItemInteger soak_temp;
  MenuItemInteger soak_duration;
  MenuItemInteger peak_temp;
  MenuItemInteger peak_duration;
  MenuItemDouble rampDown_rate;

MenuItemSubMenu profileLoadSave;
MenuItemIntegerAction profileLoad;
MenuItemIntegerAction profileSave;
MenuItemInteger profile_number;
MenuItemAction save_profile;
MenuItemAction load_profile;

MenuItemSubMenu fan_control;
  MenuItemInteger idle_speed;
  MenuItemAction save_fan_speed;

MenuItemAction factory_reset;

// ----------------------------------------------------------------------------

uint32_t startZeroCrossTicks;
uint32_t stateChangedTicks = 0;
uint32_t lastUpdate = 0;
uint32_t lastDisplayUpdate = 0;
uint32_t lastSerialOutput = 0;

// ----------------------------------------------------------------------------

double Setpoint;
double Input;
double Output;

// Define the PID tuning parameters
// TODO: make these configurable
// TODO: add PID autotune
/*
  typedef struct {
    double p;
    double i;
    double d;
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
double readingsT2[NUMREADINGS];

double runningTotalRampRate;
double rampRate = 0;
double rateOfRise = 0;          // the result that is displayed

double totalT1 = 0;             // the running total
double totalT2 = 0;

double averageT1 = 0;           // the average
double averageT2 = 0;

uint8_t index = 0;              // the index of the current reading

// ----------------------------------------------------------------------------

enum state {
  idle,
  rampToSoak,
  soak,
  rampUp,
  peak,
  rampDown,
  coolDown
};

state currentState = idle;
state lastState    = idle;

bool stateChanged = false;

volatile bool requestPoll = false;
volatile bool requestStop = false;

// ----------------------------------------------------------------------------
// timer interrupt handling

// ticks with 100µS
void timerIsr(void) {
  static uint32_t lastTicks = 0;

  // Phase Control for the fan 
  if (++phaseCounter > 90) {
    phaseCounter = 0;
  }

  if (phaseCounter >= Channels[CHANNEL_FAN].target) {
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

  // request ui / encoder update
  if (!(timerTicks % 10)) {
    myMenu.Encoder->service();
    if (currentState == idle) {
      requestPoll = true;
    } 
    else {
      switch(myMenu.Encoder->getButton()) {
        case ClickEncoder::Clicked:
        case ClickEncoder::DoubleClicked:
        case ClickEncoder::Pressed:
          requestStop = true;
          break;
      }
    }
  }

  timerTicks++;

  if (zxLoopCalibration.iterations) {
    zxLoopCalibration.measure[zxLoopCalibration.iterations]++;
  }
}

// ----------------------------------------------------------------------------

void abortWithError(int error) {
  killRelayPins();

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

  while (1) { // and stop forever...
    ;
  }
}

// ----------------------------------------------------------------------------

void displayThermocoupleData(struct tcInput* input) {
  switch (input->stat) {
    case 0:
      lcd.print(input->temperature, 1);
      lcd.print((char)223); lcd.print("C");
      break;
    case 1:
      lcd.print("---");
      break;
  }
}

// ----------------------------------------------------------------------------

void updateDisplay() {
  lcd.clear();

  displayThermocoupleData(&A);
  lcd.print(" ");
  displayThermocoupleData(&B);

  if (currentState != idle) {
    lcd.setCursor(16, 0);    
    lcd.print((zeroCrossTicks - startZeroCrossTicks) / 100);
    lcd.print("s");
  }
  lcd.setCursor(0, 1);

  switch (currentState) {
    case idle:       lcd.print("Idle");      break;
    case rampToSoak: lcd.print("Ramp");      break;
    case soak:       lcd.print("Soak");      break;
    case rampUp:     lcd.print("Ramp Up");   break;
    case peak:       lcd.print("Peak");      break;
    case rampDown:   lcd.print("Ramp Down"); break;
    case coolDown:   lcd.print("Cool Down"); break;
  }

  lcd.setCursor(11, 1);
  lcd.print("Sp=");
  lcd.print(Setpoint, 1);
  lcd.print((char)223); lcd.print("C");

  lcd.setCursor(0,2);
  lcd.print("Heat=");
  lcd.print((int)heaterValue);
  lcd.print('%');

  lcd.setCursor(10,2);
  lcd.print("Fan=");
  lcd.print((int)fanValue);
  lcd.print('%');
  
  lcd.setCursor(0,3);
  lcd.print("Ramp=");
  lcd.print(rampRate, 1);
  lcd.print((char)223);
  lcd.print("C/s");
}

// ----------------------------------------------------------------------------

void setup() {
  Serial.begin(57600);

  setupRelayPins();

  lcd.begin(LCD_CHARS, LCD_LINES);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.createChar(ICON_LEFT,  left);
  lcd.createChar(ICON_RIGHT, right);
  lcd.createChar(ICON_BACK,  back);
  lcd.createChar(ICON_DOT,   dot);

  A.chipSelect = THERMOCOUPLE1_CS;
  B.chipSelect = THERMOCOUPLE2_CS;

  myMenu.init(&control, &lcd, false);

  // initialise the menu strings (stored in the progmem), min and max values, pointers to variables etc
  control.init(F("Cycle start"),  &cycleStart);
  profile.init(F("Edit Profile"));
  rampUp_rate.init(F("Ramp up rate (C/S)"), &activeProfile.rampUpRate, 0.1, 5.0);
  soak_temp.init(F("Soak temp (C)"),  &activeProfile.soakTemp, 50, 180,false);
  soak_duration.init(F("Soak time (S)"), &activeProfile.soakDuration,10,300,false);
  peak_temp.init(F("Peak temp (C)"), &activeProfile.peakTemp,100,300,false);
  peak_duration.init(F("Peak time (S)"), &activeProfile.peakDuration,5,60,false);
  rampDown_rate.init(F("Ramp down rate(C/S)"), &activeProfile.rampDownRate, 0.1, 10);
  profileLoad.init(F("Load Profile"), &loadProfile, &profileNumber, F("Select Profile"), 0, 29,true);
  profileSave.init(F("Save Profile"), &saveProfile, &profileNumber, F("Select Profile"), 0, 29,true);
  fan_control.init(F("Fan settings"));
  idle_speed.init(F("Idle speed"),  &fanAssistSpeed, 0, 70,false);
  save_fan_speed.init(F("Save"),  &saveFanSpeed);
  factory_reset.init(F("Factory Reset"),  &factoryReset);

  // initialise the menu structure
  control.addItem(&profile);
  profile.addChild(&rampUp_rate);
  rampUp_rate.addItem(&soak_temp);
  soak_temp.addItem(&soak_duration);
  soak_duration.addItem(&peak_temp);
  peak_temp.addItem(&peak_duration);
  peak_duration.addItem(&rampDown_rate);

  // this needs to be replaced with a better menu structure. This relies on being able to 
  // have a menu item that allows the user to choose a number then be sent to another menu item
  control.addItem(&profileLoad);
  control.addItem(&profileSave);

  // fan speed control
  control.addItem(&fan_control);
  fan_control.addChild(&idle_speed);
  idle_speed.addItem(&save_fan_speed);

  // factory reset function
  control.addItem(&factory_reset);

  if (firstRun()) {
    factoryReset();
    loadParameters(0);
  } 
  else {
    loadLastUsedProfile();
  }

  loadFanSpeed();

  // setting up SPI bus  
  pinMode(DATAOUT,  OUTPUT);
  pinMode(SPICLOCK, OUTPUT);

  digitalWrite(A.chipSelect, HIGH);
  digitalWrite(B.chipSelect, HIGH);
  pinMode(A.chipSelect, OUTPUT);
  pinMode(B.chipSelect, OUTPUT);

  //pinMode(10,OUTPUT);
  //digitalWrite(10,HIGH); // set the pull up on the SS pin (SPI doesn't work otherwise!!)

  //The SPI control register (SPCR) has 8 bits, each of which control a particular SPI setting.
  // SPCR
  // | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |0000000000000000000
  // | SPIE | SPE | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
  // SPIE - Enables the SPI interrupt when 1
  // SPE - Enables the SPI when 1
  // DORD - Sends data least Significant Bit First when 1, most Significant Bit first when 0
  // MSTR - Sets the Arduino in master mode when 1, slave mode when 0
  // CPOL - Sets the data clock to be idle when high if set to 1, idle when low if set to 0
  // CPHA - Samples data on the falling edge of the data clock when 1, rising edge when 0'
  // SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz) 11 is slowest (250KHz)

  SPCR = (1<<SPE) | (1<<MSTR) | (1<<CPHA); // SPI enable bit set, master, data valid on falling edge of clock

  byte clr = 0;
  clr = SPSR;
  clr = SPDR;

  delay(10);

  PID.SetOutputLimits(0, 100); // max output 100%
  PID.SetMode(AUTOMATIC);

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

  lcd.clear();
  lcd.print(" Reflow controller");
  lcd.setCursor(0, 1);
  lcd.print("      v"); lcd.print(ver);

  // autocalibrate zero cross timing delay
  lcd.setCursor(0, 3);
  lcd.print("Calibrating... ");
  delay(500);

  while (zxLoopDelay == 0) {
    if (zxLoopCalibration.iterations < 1) {
      for (uint8_t l = 0; l < zxCalibrationLoops; l++) {
        zxLoopDelay += zxLoopCalibration.measure[l];
      }
      zxLoopDelay /= zxCalibrationLoops;
      zxLoopDelay -= 6; // offset calibration loop runtime
    }
  }

  lcd.print(zxLoopDelay);
  delay(1000);  

  // enter main menu
  myMenu.showCurrent();
}

// ----------------------------------------------------------------------------

void loop(void)
{
  // zeroCrossTicks / 100 -> 1 second -> 100 ticks per second


  if (zeroCrossTicks - lastUpdate >= 10) {
    lastUpdate = zeroCrossTicks;

    readThermocouple(&A);
    readThermocouple(&B);

    if (A.stat != 0) {
      abortWithError(3);
    }

/* moving average
    int samples[8];

    total -= samples[i];
    samples[i] = A.temperature; // new value
    total += samples[i];

    i = (i + 1) % 8; // next position
    average = total >> 3; // == div by 8
*/

    // rolling average of the temp T1 and T2
    totalT1 -= readingsT1[index]; // subtract the last reading
    totalT2 -= readingsT2[index];

    readingsT1[index] = A.temperature;
    readingsT2[index] = (B.stat == 0) ? B.temperature : 0;

    totalT1 += readingsT1[index]; // add the reading to the total
    totalT2 += readingsT2[index]; 
    
    index = (index + 1) % NUMREADINGS; // next position

    averageT1 = totalT1 / NUMREADINGS;  // calculate the average temp
    averageT2 = totalT2 / NUMREADINGS;


    // need to keep track of a few past readings in order to work out rate of rise
    for (int i = 1; i < NUMREADINGS; i++) { // iterate over all previous entries, moving them backwards one index
      airTemp[i - 1] = airTemp[i];
    }

    airTemp[NUMREADINGS - 1] = averageT1; // update the last index with the newest average

    // calculate rate of rise in degrees per polling cycle time/ num readings
    rampRate = (airTemp[NUMREADINGS - 1] - airTemp[0]); // subtract earliest reading from the current one

    Input = airTemp[NUMREADINGS - 1]; // update the variable the PID reads

    if (zeroCrossTicks - lastDisplayUpdate > 25) { // 4hz display during reflow cycle
      if (requestPoll) {
        requestPoll = false;
        myMenu.poll();
      }

      if (currentState != idle) {
        lastDisplayUpdate = zeroCrossTicks;
        updateDisplay();
      }
    }

#if WITH_SERIAL
    if (zeroCrossTicks - lastSerialOutput > 25) {
      lastSerialOutput = zeroCrossTicks;

      if (currentState == idle) {
        Serial.print("0,0,0,0,0,"); 
        Serial.print(averageT1); 
        Serial.print(",");
        if (B.stat == 0) { // only print the second C data if the input is valid
          Serial.print(averageT2); 
        } 
        else {
          Serial.print("999"); 
        }
        Serial.println();
      } 
      else {
        Serial.print((zeroCrossTicks - startZeroCrossTicks) / 100);
        Serial.print(",");
        Serial.print((int)currentState);
        Serial.print(",");
        Serial.print(Setpoint); 
        Serial.print(",");
        Serial.print(heaterValue); 
        Serial.print(",");
        Serial.print(fanValue); 
        Serial.print(",");
        Serial.print(averageT1); 
        Serial.print(",");
        if (B.stat == 0) { // only print the second C data if the input is valid
          Serial.print(averageT2); 
        }
        else {
          Serial.print("999"); 
        }
        Serial.println();
      }
    }
#endif

    if (requestStop) {
      requestStop = false;
      if (currentState == coolDown) {
        currentState = idle;
      } 
      else if (currentState != idle) {
        currentState = coolDown;
      }
    }

    // if the state has changed, set the flags and update the time of state change
    if (currentState != lastState) {
      lastState = currentState;
      stateChanged = true;
      stateChangedTicks = zeroCrossTicks;
    }

    switch (currentState) {
      case idle:
        break;

      case rampToSoak:
        if (stateChanged) {
          PID.SetMode(MANUAL);
          Output = 50;
          PID.SetMode(AUTOMATIC);
          PID.SetControllerDirection(DIRECT);
          PID.SetTunings(Kp, Ki, Kd);
          Setpoint = airTemp[NUMREADINGS - 1];
          stateChanged = false;
        }

        Setpoint += (activeProfile.rampUpRate / 10); // target set ramp up rate

        if (Setpoint >= activeProfile.soakTemp - 1) {
          currentState = soak;
        }
        break;

      case soak:
        if (stateChanged) {
          Setpoint = activeProfile.soakTemp;
          stateChanged = false;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.soakDuration * 100) {
          currentState = rampUp;
        }
        break;

      case rampUp:
        if (stateChanged) {
          stateChanged = false;
        }

        Setpoint += (activeProfile.rampUpRate / 10); // target set ramp up rate

        if (Setpoint >= activeProfile.peakTemp - 1) { // seems to take arodun 8 degrees rise to tail off to 0 rise
          Setpoint = activeProfile.peakTemp;
          currentState = peak;
        }
        break;

      case peak:
        if (stateChanged) {
          Setpoint = activeProfile.peakTemp;
          stateChanged = false;
        }

        if (zeroCrossTicks - stateChangedTicks >= (uint32_t)activeProfile.peakDuration * 100) {
          currentState = rampDown;
        }
        break;

      case rampDown:
        if (stateChanged) {
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          stateChanged = false;
          Setpoint = activeProfile.peakTemp - 15; // get it all going with a bit of a kick! v sluggish here otherwise, too hot too long
        }

        Setpoint -= (activeProfile.rampDownRate / 10); 

        if (Setpoint <= idleTemp) {
          currentState = coolDown;
          //PID.SetControllerDirection(DIRECT); // flip the PID the right way up again
        }
        break;

      case coolDown:
        if (stateChanged) {
          PID.SetControllerDirection(REVERSE);
          PID.SetTunings(fanKp, fanKi, fanKd);
          Setpoint = idleTemp;
        }
        if (Input < (idleTemp + 5)) {
          currentState = idle;
          PID.SetMode(MANUAL);
          Output = 0;
        }
    }
  }

  // safety check that we're not doing something stupid. 
  // if the thermocouple is wired backwards, temp goes DOWN when it increases
  // during cooling, the t962a lags a long way behind, hence the hugely lenient cooling allowance.
  // both of these errors are blocking and do not exit!
  if (Setpoint > Input + 50) abortWithError(1); // if we're 50 degree cooler than setpoint, abort
  //if (Input > Setpoint + 50) abortWithError(2); // or 50 degrees hotter, also abort

  PID.Compute();

  if (currentState != rampDown && currentState != coolDown && currentState != idle) { // decides which control signal is fed to the output for this cycle
    heaterValue = Output;
    fanValue = fanAssistSpeed;
  } 
  else {
    heaterValue = 0;
    fanValue = Output;
  }

  Channels[CHANNEL_HEATER].target = heaterValue;
  Channels[CHANNEL_FAN].target = 90 / 100 * fanValue; // 0-100% -> 0-90° phase control
}


void cycleStart() {
  requestStop = false;
  startZeroCrossTicks = zeroCrossTicks;
  currentState = rampToSoak;

  lcd.clear();
  lcd.print("Starting cycle ");
  lcd.print(profileNumber);
  delay(1000);
}


void saveProfile(unsigned int targetProfile) {
  profileNumber = targetProfile;
  lcd.clear();
  lcd.print("Saving profile ");
  lcd.print(profileNumber);

#ifdef DEBUG
  Serial.println("Check parameters:");
  Serial.print("idleTemp ");
  Serial.println(idleTemp);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("About to save parameters");
#endif

  saveParameters(profileNumber); // profileNumber is modified by the menu code directly, this method is called by a menu action

  delay(500); 
}

void loadProfile(unsigned int targetProfile) {
  lcd.clear();
  lcd.print("Loading profile ");
  lcd.print(targetProfile);

#ifdef DEBUG
  Serial.println("current parameters:");
  Serial.print("idleTemp ");
  Serial.println(idleTemp);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.println("About to load parameters");
#endif

  bool ok = loadParameters(targetProfile);

#ifdef DEBUG
  Serial.println("loaded parameters:");
  Serial.print("idleTemp ");
  Serial.println(idleTemp);
  Serial.print("ramp Up rate ");
  Serial.println(activeProfile.rampUpRate);
  Serial.print("soakTemp ");
  Serial.println(activeProfile.soakTemp);
  Serial.print("soakDuration ");
  Serial.println(activeProfile.soakDuration);
  Serial.print("peakTemp ");
  Serial.println(activeProfile.peakTemp);
  Serial.print("peakDuration ");
  Serial.println(activeProfile.peakDuration);
  Serial.print("rampDownRate ");
  Serial.println(activeProfile.rampDownRate);
  Serial.print("checksum ");
  Serial.println(activeProfile.checksum);
  Serial.println("after loading parameters");
#endif

  if (!ok) {
    lcd.setCursor(0, 2);
    lcd.print("Checksum error!");
    lcd.setCursor(0, 3);
    lcd.print("Review profile.");
    delay(2500);
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

  lcd.clear();
  lcd.print("First run...");
  delay(500);

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

  lcd.clear();
  lcd.print("Resetting...");

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

void saveFanSpeed(){
  lcd.clear();
  lcd.print("Saving...");
  EEPROM.write(offsetFanSpeed, (uint8_t)fanAssistSpeed & 0xff);
  delay(250);
}

void loadFanSpeed(){
  fanAssistSpeed = EEPROM.read(offsetFanSpeed) & 0xff;
}

void saveLastUsedProfile(){
  EEPROM.write(offsetProfileNum, (uint8_t)profileNumber & 0xff);
}

void loadLastUsedProfile(){
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

uint8_t crc8(uint8_t *data, uint16_t number_of_bytes_in_data)
{
  uint8_t  crc;
  uint8_t  bit_counter;
  uint8_t  b;
  uint8_t  feedback_bit;
  uint16_t loop_count;
  
  crc = CRC8INIT;

  for (loop_count = 0; loop_count != number_of_bytes_in_data; loop_count++) {
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
