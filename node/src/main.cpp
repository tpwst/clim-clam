// Sample RFM69 sender/node sketch, with ACK and optional encryption, and Automatic Transmission Control
// Sends periodic messages of increasing length to gateway (id=1)
// It also looks for an onboard FLASH chip, if present
// **********************************************************************************
// Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will
// be useful, but WITHOUT ANY WARRANTY; without even the
// implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public
// License for more details.
//
// Licence can be viewed at
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************

// #include <Arduino.h>
#include <creds.h>          // sensitive data
#include <avr/dtostrf.h>
// #include <dToStrF.h>
#include <RFM69.h>          //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>      //get it here: https://www.github.com/lowpowerlab/rfm69
// #include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>            //included with Arduino IDE install (www.arduino.cc)
#include <SparkFunBME280.h>//get it here: https://github.com/sparkfun/SparkFun_BME280_Arduino_Library/tree/master/src
#include <RTCZero.h>        // required to wake up m0 from standby mode

#include <GxEPD.h>
// select the display class to use, only one
// #include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>  // 1.54" b/w/r
#include <GxGDEP015OC1/GxGDEP015OC1.cpp> // 1.54" b/w

#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE ************
//*********************************************************************************************
#define NODEID        3    //must be unique for each node on same network (range up to 254, 255 is used for broadcast)
#define NETWORKID     100  //the same on all nodes that talk to each other (range up to 255)
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!

//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level (ATC_RSSI)
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -80
//*********************************************************************************************

#define LED           13 // Moteinos have LEDs on D9
#define FLASH_SS      8 // and FLASH SS on D8

#define SERIAL_BAUD   115200

//#define SERIAL_EN                //comment out if you don't want any serial output

#ifdef SERIAL_EN
  #define SERIAL_BAUD   115200
  #define DEBUG(input)   {Serial.print(input);}
  #define DEBUGln(input) {Serial.println(input);}
  #define SERIALFLUSH() {Serial.flush();}
#else
  #define DEBUG(input);
  #define DEBUGln(input);
  #define SERIALFLUSH();
#endif

#define WAIT_FOR_SERIAL_PIN A3  // logic for debug - serial monitor A3
#define SLEEP_DUR_PIN_1 A1      // logic for debug - sleep duration A1
#define SLEEP_DUR_PIN_2 A4      // logic for debug - sleep duration A4
#define BATT_PIN A5             // analog pin measures output from voltage divider A5

// e-paper display defs
#define EINK_CS 6     // orange
#define EINK_DC 5     // green
#define EINK_RST 4    // white
#define EINK_BUSY 3   // purple

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI,EINK_CS,EINK_DC,EINK_RST);
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io, EINK_RST, EINK_BUSY);
#define WIDTH 200
#define HEIGHT 200
#define LMARGIN 12
#define TMARGIN 32
#define DATA_MARGIN 12

boolean requestACK = false;
// SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

// create rtc obj
RTCZero rtc;
const byte sec = 00;
const byte min = 00;
const byte hrs = 00;
int sleep_dur;
// const int sleep_dur = 30000;

BME280 bme;
char Pstr[10];
char Fstr[10];
char Cstr[10];
char Hstr[10];
char Astr[10];
char Bstr[10];
double F,C,P,H,A;
const int bme_delay = 10;
char buffer[50];

#define BATT_NUM_SAMPLES 5
float batt;
int batt_sum = 0;
unsigned char sample_count = 0;

char buff[20];

// pin defs for moteino m0
#define RADIO_SS A2
#define RADIO_IRQ 9

#ifdef ENABLE_ATC
  RFM69_ATC radio(RADIO_SS,RADIO_IRQ,false);
#else
  RFM69 radio(RADIO_SS,RADIO_IRQ,false);
#endif

void Blink(byte PIN, int DELAY_MS) {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void setup_radio() {
  if (!radio.initialize(FREQUENCY,NODEID,NETWORKID)) {
      Serial.println("!! Radio No Init !!");
    }
  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif
    radio.encrypt(ENCRYPTKEY);
    //radio.setFrequency(919000000); //set frequency to some custom frequency

  //Auto Transmission Control - dials down transmit power to save battery (-100 is the noise floor, -90 is still pretty good)
  //For indoor nodes that are pretty static and at pretty stable temperatures (like a MotionMote) -90dBm is quite safe
  //For more variable nodes that can expect to move or experience larger temp drifts a lower margin like -70 to -80 would probably be better
  //Always test your ATC mote in the edge cases in your own environment to ensure ATC will perform as you expect
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif

    char buff[50];
    sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(buff);

  #ifdef ENABLE_ATC
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
  #endif
}

void setup_bme() {
  Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!

  //initialize weather shield BME280 sensor
  bme.setI2CAddress(0x77); //0x76,0x77 is valid.
  bme.beginI2C();
  bme.setMode(MODE_FORCED); //MODE_SLEEP, MODE_FORCED, MODE_NORMAL is valid. See 3.3
  bme.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
  bme.setFilter(0); //0 to 4 is valid. Filter coefficient. See 3.4.4
  bme.setTempOverSample(1); //0 to 16 are valid. 0 disables temp sensing. See table 24.
  bme.setPressureOverSample(1); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
  bme.setHumidityOverSample(1); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
  P = bme.readFloatPressure();
  F = bme.readTempF();
  C = bme.readTempC();
  H = bme.readFloatHumidity();
  A = bme.readFloatAltitudeMeters();
  bme.setMode(MODE_SLEEP);
}

void setup_display() {
  display.init();
  display.setTextSize(2);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(1);
}

void alarmMatch() {
  rtc.setTime(hrs,min,sec);  // reset the clock every alarm
}

void setup_rtc() {
  rtc.begin();
  rtc.setTime(hrs,min,sec);
  // value is half of what you get (14 sec = roughly 30 sec)
  // this may be due to the rtc slowing down while in standby mode?

  //  | on [█]|
  //  |░ ░ ░ ░|
  //  |█ █ █ █|
  //  |1 2 3 4|
  if (digitalRead(SLEEP_DUR_PIN_1) == HIGH && digitalRead(SLEEP_DUR_PIN_2) == HIGH) {
    rtc.setAlarmTime(0,7,28);  // 15 min
    Serial.println("Sleep: 15 min");
    sleep_dur = 900;

  //  | on [█]|
  //  |░ █ ░ ░|
  //  |█ ░ █ █|
  //  |1 2 3 4|
  } else if (digitalRead(SLEEP_DUR_PIN_1) == LOW && digitalRead(SLEEP_DUR_PIN_2) == HIGH) {
    rtc.setAlarmTime(0,0,14);  // 5 min
    Serial.println("Sleep: 5 min");
    sleep_dur = 300;

  //  | on [█]|
  //  |░ ░ █ ░|
  //  |█ █ ░ █|
  //  |1 2 3 4|
  } else if (digitalRead(SLEEP_DUR_PIN_1) == HIGH && digitalRead(SLEEP_DUR_PIN_2) == LOW) {
    rtc.setAlarmTime(0,0,28);  // 1 min
    Serial.println("Sleep: 1 min");
    sleep_dur = 60;

  //  | on [█]|
  //  |░ █ █ ░|
  //  |█ ░ ░ █|
  //  |1 2 3 4|
  } else if (digitalRead(SLEEP_DUR_PIN_1) == LOW && digitalRead(SLEEP_DUR_PIN_2) == LOW) {
    rtc.setAlarmTime(0,0,14);  // 30 sec
    Serial.println("Sleep: 30 sec");
    sleep_dur = 30;
  }

  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(alarmMatch);
}

void update_display(char* bstr, char* fstr, char* cstr, char* hstr, char* pstr, char* astr) {
  display.fillScreen(GxEPD_WHITE);

  // status bar
  display.fillScreen(GxEPD_WHITE);
  display.fillRect(0,0,display.width(),18,GxEPD_BLACK);
  // bottom bar
  display.fillRect(0,HEIGHT - 18,display.width(),18,GxEPD_BLACK);
  // draw battery icon
  display.fillRect(display.width()-(6*12),4,8,12,GxEPD_WHITE);
  display.fillRect(display.width()-(6*12)+2,2,4,1,GxEPD_WHITE);
  // draw RSSI
  display.setCursor(2,2);
  display.setTextColor(GxEPD_WHITE);
  display.print(radio.RSSI);
  // draw battery voltage
  display.setCursor(display.width()-(5*12),2);
  display.print(bstr);
  display.print("V");
  // draw temp f
  display.setCursor(DATA_MARGIN,TMARGIN);
  display.setTextColor(GxEPD_WHITE,GxEPD_BLACK);
  display.setTextSize(1);
  display.print(" TEMP ");
  display.setCursor(DATA_MARGIN,TMARGIN + 12);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(2);
  display.print(fstr);
  display.print(" F");
  // draw temp c
  display.setCursor(DATA_MARGIN,TMARGIN + 28);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(2);
  display.print(cstr);
  display.print(" C");
  // draw rh
  display.setCursor(DATA_MARGIN,TMARGIN + 64);
  display.setTextColor(GxEPD_WHITE,GxEPD_BLACK);
  display.setTextSize(1);
  display.print(" RELH ");
  display.setCursor(DATA_MARGIN,TMARGIN + 76);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(2);
  display.print(hstr);
  display.print(" %");
  // draw pressure
  display.setCursor(DATA_MARGIN,TMARGIN + 110);
  display.setTextColor(GxEPD_WHITE,GxEPD_BLACK);
  display.setTextSize(1);
  display.print(" PRSR ");
  display.setCursor(DATA_MARGIN,TMARGIN+ 122);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(2);
  display.print(pstr);
  display.print(" Pa");
  // draw sleep_dur
  display.setCursor(2,HEIGHT - 16);
  display.setTextColor(GxEPD_WHITE);
  display.print(sleep_dur);
  display.print("sec");

  display.update();
}

char* take_data() {
  //read BME sensor
  delay(bme_delay);
  bme.setMode(MODE_FORCED); //Wake up sensor and take reading
  delay(bme_delay);
  P = bme.readFloatPressure(); // in Pa
  F = bme.readTempF();
  C = bme.readTempC();
  H = bme.readFloatHumidity();
  A = bme.readFloatAltitudeMeters();
  delay(bme_delay);
  bme.setMode(MODE_SLEEP);

  // take multiple vbatt samples
  while (sample_count < BATT_NUM_SAMPLES) {
    batt_sum += analogRead(BATT_PIN);
    sample_count++;
    delay(10);
  }

  batt = (((float)batt_sum / (float)BATT_NUM_SAMPLES * 3.29) / 1024.0) * 2.0;

  // don't like calling dtostrf like this
  dtostrf(F, 3,2, Fstr);
  dtostrf(C, 3,2, Cstr);
  dtostrf(H, 3,2, Hstr);
  dtostrf(P, 3,2, Pstr);
  dtostrf(A, 3,2, Astr);
  dtostrf(batt,3,2,Bstr);

  update_display(Bstr,Fstr,Cstr,Hstr,Pstr,Astr);

  // this will be the comma delim string for TX
  sprintf(buffer, "%s,%s,%s,%s,%s,%s,%d", Cstr, Fstr, Hstr, Pstr, Astr, Bstr, sleep_dur);
  Serial.println(buffer);
  return buffer;
}

void standby_sleep() {
  Serial.println("Good-Night");

  // flash.sleep();
  radio.sleep(); //you can comment out this line if you want this node to listen for wireless programming requests

  rtc.standbyMode();

  // put samd21 to sleep
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  if (digitalRead(WAIT_FOR_SERIAL_PIN) == LOW) {
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  }
  setup_radio();
  setup_bme();
  setup_display();
  setup_rtc();
}

void loop() {
  char* payload = take_data();
  byte sendSize = (byte)strlen(payload);

  // send data
  Serial.print("Sending[");
  Serial.print(sendSize);
  Serial.print("]: ");
  for(byte i = 0; i < sendSize; i++) {
    Serial.print((char)payload[i]);
  }
    if (radio.sendWithRetry(GATEWAYID, payload, sendSize)) {
      Serial.print(" ok!");
    } else {
      Serial.print(" nothing...");
    }
  Serial.println();
  Blink(LED,3);

  // go to sleep - need to inturrpt via internal RTC (use RTCZero lib)
  standby_sleep();
}
