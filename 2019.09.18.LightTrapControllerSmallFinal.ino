#include "SdFat.h"
#include <SPI.h> 
#include <OneWire.h>
#include <Wire.h> //I2C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#define INCLUDE_SCROLLING 0
#include <Adafruit_INA219.h> //Power module
#include <TimeLib.h> //Used for keeping track of time
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <DallasTemperature.h>
#include <PinChangeInterrupt.h>

//Macros that help avoid using digital read/write0
#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 8

//Digital Pins
#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define ENCODER_PIN_CENTER 6
#define PIN_BUTTON_L 2
#define PIN_BUTTON_R 3
#define TRANSISTOR_CONTROL 9
#define SD_CS_PIN 10

//Digital Pins
#define LIGHT_SENSOR_1 A0
#define LIGHT_SENSOR_2 A1
#define ENCODER_PIN_A 4
#define ENCODER_PIN_B 5
#define ENCODER_PIN_CENTER 6
#define PIN_BUTTON_L 2
#define PIN_BUTTON_R 3
#define TRANSISTOR_CONTROL 9
#define SD_CS_PIN 10


//OLED Display
#define I2C_ADDRESS 0x3C
#define OLED_RESET 4
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define DEFAULT_UPDATE_PERIOD 0 //Means do not run update routine
#define ONE_SECOND_UPDATE_PERIOD 1000
#define DISPLAY_TIME_OUT 5000
#define INTERRUPT_BOUNCE_TIME 10
#define INTERRUPT_BOUNCE_TIME_BUTTON 200
#define MAIN_MENU_UPDATE_TIMER 500

struct Time {
    byte hour;
    byte minute;
    byte second;
};

struct Date {
    int day;
    int month;
    int year;
};

struct BatteryInfo {
    float busVoltage;
    float shuntVoltage;
    float loadVoltage;
    float current;
    float power;
};

struct AccelerometerInfo {
    float accX;
    float accY;
    float accZ;
};

struct LightSensorInfo {
    word L1; //Light sensor 1
    word L2; //Light sensor 2
};

Adafruit_INA219 ina219;
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temperatureSensor(&oneWire);
File myFile;
SdFat SD;
SSD1306AsciiWire oled;
Time StartTime;
Time StopTime;

#define SET_TIME 0
#define SET_DATE 1
#define SET_START 2
#define SET_STOP 3

byte State = SET_TIME;
bool NeedReDraw = true;
bool Running = false;
bool Confirmed = false;
int CursorPosition = 0;
int SettingA = 0;
int SettingB = 0;
int SettingC = 0;
bool LightStatus = false;

void encoderInterruptRoutine() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > INTERRUPT_BOUNCE_TIME) {
      lastInterruptTime = interruptTime;
        int newValue = digitalState(ENCODER_PIN_A) == digitalState(ENCODER_PIN_B) ? -1 : 1;
        if(CursorPosition == 3) {
          Confirmed = !Confirmed;
        } else {
          if(State == SET_DATE) {
            setValuesForDate(newValue);
          } else {
            setValuesForTime(newValue);
          }
        }
        NeedReDraw = true;
    }
}

void setValuesForTime(int newValue) {
  switch(CursorPosition) {
    case 0: //Hour
      newValue += SettingA;
      if (newValue > 23) {
          newValue = newValue % 24;
      } else if (newValue < 0) {
          newValue = 24 + newValue;
      }
      SettingA = newValue;
      break;
    case 1: //Minute
      newValue += SettingB;
      if (newValue > 59) {
          newValue = newValue % 60;
      } else if (newValue < 0) {
          newValue = 60 + newValue;
      }
      SettingB = newValue;
      break;
    case 2: //Second
      newValue += SettingC;
      if (newValue > 59) {
          newValue = newValue % 60;
      } else if (newValue < 0) {
          newValue = 60 + newValue;
      }
      SettingC = newValue;
      break;
  }
}

void setValuesForDate(int newValue) {
  switch(CursorPosition) {
    case 0:
      {
        SettingA = SettingA + newValue;
        int maxDay = getMaxDayForMonth(SettingC, SettingB);
        if (SettingA > maxDay)
            SettingA = 1;
        if (SettingA < 1)
            SettingA = maxDay;
      }
      break;
    case 1:
      {
        SettingB = SettingB + newValue;
        if (SettingB > 12)
            SettingB = 1;
        if (SettingB < 1)
          SettingB = 12;
      }
      break;
    case 2:
      {
        SettingC = SettingC + newValue;
        //Limit the min year to 2019
        if (SettingC < 2019)
            SettingC = 2019;
      }
      break;
  }
}

void encoderCenterInterruptRoutine() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > INTERRUPT_BOUNCE_TIME_BUTTON) {
        lastInterruptTime = interruptTime;
        if(CursorPosition == 3 && Confirmed) {
          switch(State) {
            case SET_TIME:
              setTime(SettingA, SettingB, SettingC, 24, 5, 2019); //Date does not matter
              SettingA = day();
              SettingB = month();
              SettingC = year();
              State = SET_DATE;
              break;
            case SET_DATE:
              setTime(hour(), minute(), second(), SettingA, SettingB, SettingC);
              SettingA = 0;
              SettingB = 0;
              SettingC = 0;
              State = SET_START;
              break;
            case SET_START:
              StartTime = Time{SettingA, SettingB, SettingC};
              SettingA = 0;
              SettingB = 0;
              SettingC = 0;
              State = SET_STOP;
              break;
            case SET_STOP:
              StopTime = Time{SettingA, SettingB, SettingC};
              Running = true; //Start the system!
              break;
          }
          Confirmed = false;
          CursorPosition = 0;
        } else {
          CursorPosition = (CursorPosition + 1) % 4;
        }
        NeedReDraw = true;
    }
}

void leftInterruptRoutine() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > INTERRUPT_BOUNCE_TIME_BUTTON) {
        lastInterruptTime = interruptTime;
        
    }
}

void rightInterruptRoutine() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > INTERRUPT_BOUNCE_TIME_BUTTON) {
        lastInterruptTime = interruptTime;
        
    }
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&Adafruit128x64, I2C_ADDRESS, OLED_RESET);
  oled.setFont(utf8font10x16);
  oled.clear();
  
  ina219.begin();
  ina219.setCalibration_32V_1A();

  lis.begin(0x18);
  lis.setDataRate(LIS3DH_DATARATE_1_HZ);
  lis.setRange(LIS3DH_RANGE_4_G);
  
  Serial.print(F("Initializing SD card..."));

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("initialization failed!"));
    return;
  }
  Serial.println(F("initialization done."));

  pinAsInputPullUp(ENCODER_PIN_A);     
  pinAsInputPullUp(ENCODER_PIN_CENTER);     
  pinAsOutput(TRANSISTOR_CONTROL);

  attachPCINT(digitalPinToPCINT(ENCODER_PIN_A), encoderInterruptRoutine, RISING);
  attachPCINT(digitalPinToPCINT(ENCODER_PIN_CENTER), encoderCenterInterruptRoutine, FALLING);
}

void writeColumns() {
  myFile = SD.open("sensor.csv", FILE_WRITE);
  myFile.println(F("date,time,busVoltage,shuntVoltage,loadVoltage,current,power,temperature,lightSensor1,lightSensor2,lightStatus"));
  myFile.close();
  
  myFile = SD.open("accel.csv", FILE_WRITE);
  myFile.println(F("date,time,accX,accY,accZ"));
  myFile.close();
}

void writeFromSensors() {
  Date date = getDate();
  Time t = getTime();
  BatteryInfo bat = getBatteryInfo();
  float temp = getTemperatureReading();
  LightSensorInfo light = getLightSensorInfo();
  myFile = SD.open("sensor.csv", FILE_WRITE);

  writeTimeStamp(date,t);
  myFile.print(bat.busVoltage);
  myFile.print(",");
  myFile.print(bat.shuntVoltage);
  myFile.print(",");
  myFile.print(bat.loadVoltage);
  myFile.print(",");
  myFile.print(bat.current);
  myFile.print(",");
  myFile.print(bat.power);
  myFile.print(",");
  myFile.print(temp);
  myFile.print(",");
  myFile.print(light.L1);
  myFile.print(",");
  myFile.print(light.L2);
  myFile.print(",");
  myFile.println(LightStatus);
  myFile.close();
}

void writeTimeStamp(Date d, Time t) {
  myFile.print(d.day);
  myFile.print("/");
  myFile.print(d.month);
  myFile.print("/");
  myFile.print(d.year);
  myFile.print(",");
  myFile.print(t.hour);
  myFile.print(":");
  myFile.print(t.minute);
  myFile.print(":");
  myFile.print(t.second);
  myFile.print(",");
}

void writeAccelerometer() {
  myFile = SD.open("accel.csv", FILE_WRITE);
  Date date = getDate();
  Time t = getTime();
  writeTimeStamp(date,t);
  AccelerometerInfo acc = getAccelerometerInfo();
  myFile.print(acc.accX);
  myFile.print(",");
  myFile.print(acc.accY);
  myFile.print(",");
  myFile.println(acc.accZ);
  myFile.close();
}

Date getDate() {
  return Date{day(),month(),year()};
}

Time getTime() {
  return Time{hour(),minute(),second()};
}

BatteryInfo getBatteryInfo() {
    BatteryInfo info{};
    info.shuntVoltage = ina219.getShuntVoltage_mV();
    info.busVoltage = ina219.getBusVoltage_V();
    info.current = ina219.getCurrent_mA();
    info.power = ina219.getPower_mW();
    info.loadVoltage = info.busVoltage + (info.shuntVoltage / 1000);
    return info;
}

AccelerometerInfo getAccelerometerInfo() {
    sensors_event_t event;
    lis.getEvent(&event);
    return AccelerometerInfo{event.acceleration.x, event.acceleration.y, event.acceleration.z};
}

float getTemperatureReading() {
    temperatureSensor.requestTemperatures(); // Send the command to get temperatures
    return temperatureSensor.getTempCByIndex(0);
}

LightSensorInfo getLightSensorInfo() {
    return LightSensorInfo{analogRead(LIGHT_SENSOR_1), analogRead(LIGHT_SENSOR_2)};
}

bool isClear = false;

void loop() {
  if(Running) {
    if(!isClear) {
      oled.clear();
      writeColumns();
      isClear = true;
    }
    static unsigned long lastUpdateTime = 0;
    static byte updateCount = 60;
    if(millis() - lastUpdateTime > 1000) {
      Serial.println(F("tick"));
      lastUpdateTime = millis();
      if(betweenTime(StartTime, StopTime, getTime())) {
        digitalHigh(TRANSISTOR_CONTROL);
        LightStatus = true;
      } else {
        digitalLow(TRANSISTOR_CONTROL);
        LightStatus = false;
      }
      if(updateCount < 60) {
        writeAccelerometer();
        updateCount++;
      } else {
        updateCount = 0;
        writeFromSensors();
        writeAccelerometer();
      }
    }
  } else {
    if(NeedReDraw) {
      NeedReDraw = false;
      draw();
    }
  }
}

void draw() {
  oled.clear();
  oled.setCursor(70,0);
  if(State == SET_DATE) {
    oled.print(F("DATE"));
    oled.setCol(10);
    oled.print(F("DAY   "));
    oled.println(SettingA);
    oled.setCol(10);
    oled.print(F("MON   "));
    oled.println(SettingB);
    oled.setCol(10);
    oled.print(F("YEAR   "));
    oled.println(SettingC);
  } else {
    if(State == SET_TIME) {
      oled.print(F("TIME"));
    } else if(State == SET_START) {
      oled.print(F("START"));
    } else if(State == SET_STOP) {
      oled.print(F("STOP"));
    }
    oled.setCol(10);
    oled.print(F("HOUR   "));
    oled.println(SettingA);
    oled.setCol(10);
    oled.print(F("MIN   "));
    oled.println(SettingB);
    oled.setCol(10);
    oled.print(F("SEC   "));
    oled.println(SettingC);
  }
  oled.setCol(10);
  if(!Confirmed) {
    oled.println(F("Done?"));
  } else {
    oled.println(F("Continue"));
  }
  oled.setRow(2 * CursorPosition);
  oled.setCol(0);
  oled.print(">");
}

bool betweenTime(Time startTime, Time stopTime, Time nowTime) {
  unsigned long startTimeL = convertTimeToLong(startTime);
  unsigned long stopTimeL = convertTimeToLong(stopTime);
  unsigned long nowTimeL = convertTimeToLong(nowTime);
  if(startTimeL < stopTimeL) {
    return nowTimeL > startTimeL && nowTimeL < stopTimeL;
  } else if(startTimeL > stopTimeL) {
    return !(nowTimeL > stopTimeL && nowTimeL < startTimeL);
  } else {
    return false;
  }
}

unsigned long convertTimeToLong(Time t) {
  return (10000L * t.hour) + (100L * t.minute) + t.second;
}

int getMaxDayForMonth(int year, byte month) {
    int days;
    bool isLeapYear = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);

    switch (month) {
        case 1:
            days = 31;
            break;
        case 2:
            if (isLeapYear)
                days = 29;
            else
                days = 28;
            break;
        case 3:
            days = 31;
            break;
        case 4:
            days = 30;
            break;
        case 5:
            days = 31;
            break;
        case 6:
            days = 30;
            break;
        case 7:
            days = 31;
            break;
        case 8:
            days = 31;
            break;
        case 9:
            days = 30;
            break;
        case 10:
            days = 31;
            break;
        case 11:
            days = 30;
            break;
        case 12:
            days = 31;
            break;
        default:
            days = 31;
    }
    return days;
}
