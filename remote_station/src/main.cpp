// REMOTE CONTROLLER
// SENDS DATA FROM ATTACHED BME280 TO RECEIVING BASE STATION
// THAT THEN SENDS DATA VIA MQTT TO A SERVER
#include <Arduino.h>
#include <creds.h>
#include <SPI.h>
#include <Wire.h>
#include <GxEPD.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <avr/dtostrf.h>
// #include <Adafruit_BME280.h>
#include <SparkFunBME280.h>
#include <RTCZero.h>

// select the display class to use, only one
// #include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>  // 1.54" b/w/r
#include <GxGDEP015OC1/GxGDEP015OC1.cpp> // 1.54" b/w

#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

// DEFINE RADIO FREQUENCY
#define RF69_FREQ 915.0

#define RFM69_CS 8
#define RFM69_INT 3
#define RFM69_RST 4
#define LED 13
#define VBATPIN A7
#define sleep_pin A2 // logic for sleep duration
#define wait_for_serial_pin A1 // logic for debug - serial monitor

// #define BME_CS 10 // chip select pin for BME280 (SPI)

#define SEALEVELPRESSURE_HPA (1013.25) // For the altitude adjustment

// Where to send packets to!
#define DEST_ADDRESS 1
// change addresses for each client board, any number :)
#define MY_ADDRESS 2

static const uint8_t CS = 6;
static const uint8_t DC = 12;
static const uint8_t RST = 11;
static const uint8_t BUSY = 10;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
// GxIO_Class io(SPI, SS, 17, 16); // arbitrary selection of 17, 16
GxIO_Class io(SPI,CS,DC,RST);
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io, RST, BUSY); // arbitrary selection of (16), 4

// Adafruit_BME280 bme(BME_CS,MOSI,MISO,SCK); // hardware SPI
// Adafruit_BME280 bme; // hardware I2C
BME280 bme; //sparkfun bme obj

// create rtc obj
RTCZero rtc;

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

const byte sec = 0;
const byte min = 00;
const byte hrs = 00;

// type grid
const int margin = 12;
const int top = 32;
const int barMargin = 2;

float msg_data[5];
int data_size = (int)( sizeof(msg_data) / sizeof(msg_data[0]));
char delim[2] = ",";

char radiopacket[80];
char str_temp[80]; // temp string for dtostrf
char bat_temp[10]; // temp string for dtostrf

int sleepDur;

//bools for radio results
bool radio_init, radio_freq, radio_reply, radio_ack;

void alarmMatch() {
  rtc.setTime(hrs,min,sec);  // reset the clock every alarm
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void setup() {
  Serial.begin(115200);
  if (digitalRead(wait_for_serial_pin) == LOW) {
    while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer
  }

  Serial.println("\n<< UP >>\n");

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RST,LOW);
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);

  pinMode(sleep_pin,INPUT);

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  rtc.begin();
  rtc.setTime(hrs,min,sec);
  if (digitalRead(sleep_pin) == HIGH) {
    rtc.setAlarmTime(0,15,00);  // 15 min
    Serial.println("Sleep: 15 min");
    sleepDur = 900;
  } else {
    rtc.setAlarmTime(0,00,30);  // 30 sec
    Serial.println("Sleep: 30 sec");
    sleepDur = 30;
  }

  rtc.enableAlarm(rtc.MATCH_MMSS);

  rtc.attachInterrupt(alarmMatch);

  display.init();
  display.setTextSize(2);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(2);

  // Serial.print("BME280 init = "); Serial.println(bme.begin(), HEX);
  bme.settings.commInterface = I2C_MODE;
  bme.settings.I2CAddress = 0x77;

  //  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
  bme.settings.runMode = 2;
  bme.settings.tStandby = 0;
  bme.settings.filter = 0; // filter off

  bme.settings.tempOverSample = 1; // 0 = skipped
  bme.settings.pressOverSample = 1; // 0 = skipped
  bme.settings.humidOverSample = 1; // 0 = skipped

  Serial.print("BME280 INIT... ");
  delay(10);  // delay to let bme init
  Serial.println(bme.begin(), HEX);

  delay(100);

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(20,40);
    display.print("RADIO INIT");
    display.setCursor(20,60);
    display.print("FAILED");
    display.update();
    while (1);
}
  Serial.println("RFM69 radio init OK!");
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(14, true); // range from 14-20 for power, 2nd arg must be true for 69HCW
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @"); Serial.print((int)RF69_FREQ); Serial.println(" MHz");

  // if (!bme.begin()) {
  //   Serial.println("\n\nCouln't find BME280!\n");
  // }
}

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t data[] = "  OK";

void loop() {
  bme.begin();

  char record[40] = ""; // set record to empty for next loop job

  Serial.println("\n<< LOOP >>\n");

  Serial.print("Seconds: ");
  Serial.println(rtc.getSeconds());
  Serial.print("Minutes: ");
  Serial.println(rtc.getMinutes());
  Serial.print("\n");

  float bat = analogRead(VBATPIN);
  bat *= 2; // mult by 2 since measurement is half
  int analog_bat = static_cast<int>(bat);
  float percent_bat = map(analog_bat, 1118, 1322, 0, 100);
  float volt_bat = bat*3.3; // mult by 3.3 the reference voltage
  volt_bat /= 1024; // convert to voltage

  if (bme.begin()) {

    msg_data[0] = bme.readTempC();
    // msg_data[1] = msg_data[0] * 9.0 / 5.0 + 32;
    msg_data[1] = bme.readTempF();
    msg_data[2] = bme.readFloatHumidity();
    // msg_data[3] = bme.readPressure()/100.00F;
    msg_data[3] = bme.readFloatPressure()/100.00F;
    // msg_data[4] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    msg_data[4] = bme.readFloatAltitudeMeters();
    // msg_data[5] = percent_bat;

    Serial.println(msg_data[0]);
    Serial.println(msg_data[1]);
    Serial.println(msg_data[2]);
    Serial.println(msg_data[3]);
    Serial.println(msg_data[4]);
    // Serial.println(msg_data[5]);
    Serial.print("\n");

    // loop through bme_data indexes and format floats to strings
    for(int i = 0; i < data_size; i++) {
      dtostrf(msg_data[i],4,2,str_temp);
      strcat(record,str_temp);
      if(i<data_size-1) {
        strcat(record,delim);
      }
      Serial.println(record);
      sprintf(radiopacket,"%s,%d,%d",record,(int)percent_bat,sleepDur);
    }

    // char radiopacket[20] = "Hello World #";
    Serial.print("\nSending "); Serial.println(radiopacket);

    // Send a message to the DESTINATION!
    if (rf69_manager.sendtoWait((uint8_t *)radiopacket, strlen(radiopacket), DEST_ADDRESS)) {
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      uint8_t from;
      radio_ack = true;
      if (rf69_manager.recvfromAckTimeout(buf, &len, 2000, &from)) {
        buf[len] = 0; // zero out remaining string

        Serial.print("Got reply from #"); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf69.lastRssi());
        Serial.print("] : ");
        Serial.println((char*)buf);
        Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks
        radio_reply = true;
      } else {
        Serial.println("No reply, is anyone listening?");
        radio_reply = false;
      }
    } else {
      Serial.println("Sending failed (no ack)");
      radio_ack = false;
    }

    // status bar
    display.fillScreen(GxEPD_WHITE);
    display.fillRect(0,0,display.width(),18,GxEPD_BLACK);
    // draw gutter line
    display.drawLine(58,0,58,display.height(),GxEPD_BLACK);
    display.drawLine(59,0,59,display.height(),GxEPD_BLACK);
    // draw battery icon
    if (percent_bat>=100) {
      display.fillRect(display.width()-(5*12),4,8,12,GxEPD_WHITE);
      display.fillRect(display.width()-(5*12)+2,2,4,1,GxEPD_WHITE);
    } else {
      display.fillRect(display.width()-(4*12),4,8,12,GxEPD_WHITE);
      display.fillRect(display.width()-(4*12)+2,2,4,1,GxEPD_WHITE);
    }
    // draw deep duration
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(barMargin,2);
    if (digitalRead(sleep_pin) == HIGH) {
      display.print("15 min");
    } else {
      display.print("30 sec");
    }
    // draw analog battery val DEV
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(display.width()-100,2);
    display.print(analog_bat);
    // draw battery %
    if (percent_bat>=100) {
      display.setCursor(display.width()-(4*12),2);
    } else {
      display.setCursor(display.width()-(3*12),2);
    }
    display.setTextColor(GxEPD_WHITE);
    display.print((int)percent_bat);
    display.println("%");
    // draw temp C
    display.setCursor(margin,top);
    display.setTextColor(GxEPD_BLACK);
    display.print("TMP  ");
    display.print(msg_data[0]);
    display.print(" C");
    // draw temp F
    display.setCursor(margin,top+20);
    display.print("     ");
    display.print(msg_data[1]);
    display.print(" F");
    // draw relHumidity
    display.setCursor(margin,top+50);
    display.print("HUM  ");
    display.print(msg_data[2]);
    display.print(" %");
    // draw pressure
    display.setCursor(margin,top+80);
    display.print("PSR  ");
    display.print(msg_data[3]);
    display.print(" mB");
    // draw alt
    display.setCursor(margin,top+110);
    display.print("ALT  ");
    display.print(msg_data[4]);
    display.print(" m");

    // bottom bar
    display.fillRect(0,display.height()-18,display.width(),18,GxEPD_BLACK);

    // display radio checks
    display.setTextColor(GxEPD_WHITE);
    display.setCursor(barMargin,display.height()-16);

    if (!radio_ack) {               // if sending failed (no ack)
      display.print("FAILED (NO ACK)");
    } else if(!radio_reply) {       // if no reply
      display.print("NO REPLY");
    } else {                        // if good send
      display.print("GOOD SEND: ");
      display.print(rf69.lastRssi());
    }

  // display error alert if bme280 not found
  } else {
    char noSenErr[10] = "NO SENSOR";
    Serial.println("\n\nCouln't find BME280!\n");
    display.fillRect(0,display.height()/2-18,display.width(),36,GxEPD_BLACK);
    display.setCursor((display.width()/2) - (strlen(noSenErr)*12)/2,display.height()/2 - 8);
    display.setTextColor(GxEPD_WHITE);
    display.print(noSenErr);
  }

  bme.writeRegister(BME280_CTRL_MEAS_REG, 0x00); //sleep the BME
  rf69.sleep(); // sleep transmitter

  Serial.print("\nE-PAPER\n");
  display.update();

  rtc.standbyMode();
}
