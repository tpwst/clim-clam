// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption, and Automatic Transmission Control
// Passes through any wireless received messages to the serial port & responds to ACKs
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
#include <creds.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <Adafruit_SSD1306.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <RTClib.h>

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        1    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
// #define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#define SERIAL_BAUD   115200

// pin defs for feather huzzah esp8266 with rfm feather wing
#define RADIO_SS 2
#define RADIO_IRQ 15
#define LED 0

// button pins
#define BUTT_B 16

#ifdef ENABLE_ATC
  RFM69_ATC radio(RADIO_SS,RADIO_IRQ,false);
#else
  RFM69 radio(RADIO_SS,RADIO_IRQ,false);
#endif

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
// NTPClient timeClient(obj,server pool, offset, update interval in ms);
// -14400 offset is EDT or EST
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 0, 60000);
RTC_Millis rtc;

Adafruit_SSD1306 oled = Adafruit_SSD1306();

// SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

// sleep duration var
DateTime sleepDur;

// mqtt payload var
char mqtt_payload[100];
char sender[6];
uint8_t mqtt_count = 0;

// chars for parsing rf payload
// char msg_data_tempC[8] = "";
// char msg_data_tempF[8] = "";
// char msg_data_hum[8] = "";
// char msg_data_pres[16] = "";
// char msg_data_alt[16] = "";
// char msg_data_bat[8] = "";
// char msg_data_slp[8] = "";

// oled vars
const int w = 128;
const int h = 32;

void Blink(byte PIN, int DELAY_MS) {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(PIN,HIGH);
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);

  oled.clearDisplay();
  oled.setTextColor(BLACK,WHITE);
  oled.print("WiFi");
  oled.display();

  WiFi.mode(WIFI_STA);

  // WiFi.begin(ssid, pass);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  oled.setTextColor(WHITE);
  oled.setCursor(0,10);
  oled.print("CONNECTED");
  oled.setCursor(0,20);
  oled.print(WiFi.localIP());
  oled.display();
}

void setup_mqtt() {
  client.setServer(server, 1883);
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "CLIM-CLAM-";
    clientId += String(random(0xffff), HEX);

    oled.clearDisplay();
    oled.setTextColor(BLACK,WHITE);
    oled.setCursor(0,0);
    oled.print("MQTT");
    oled.display();
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      oled.setTextColor(WHITE);
      oled.setCursor(0,10);
      oled.print("CONNECTED");
      oled.setCursor(0,20);
      oled.print(server);
      oled.display();
    } else {

      if (mqtt_count >= 5) {
        mqtt_count = 0;
        ESP.restart();
      }

      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      oled.setTextColor(WHITE);
      oled.setCursor(0,10);
      oled.print("FAILED");
      oled.display();

      mqtt_count++; // increment the count to reset

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_radio() {
  oled.clearDisplay();
  oled.setTextColor(BLACK,WHITE);
  oled.setCursor(0,0);
  oled.print("RADIO");
  oled.display();

  // init radio
  if (!radio.initialize(FREQUENCY,NODEID,NETWORKID)) {
    Serial.println("\n!! Radio No Init !!");
    oled.setTextColor(WHITE);
    oled.setCursor(0,10);
    oled.print("INIT FAILED");
  } else {
    Serial.println("\nRadio Init Successful");
    oled.setTextColor(WHITE);
    oled.setCursor(0,10);
    oled.print("INIT SUCCESS");
  }
  oled.display();

  #ifdef IS_RFM69HW_HCW
    radio.setHighPower(); //must include this only for RFM69HW/HCW!
  #endif

  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  //radio.setFrequency(919000000); //set frequency to some custom frequency
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  oled.setCursor(0,20);
  oled.print("@ 915 Mhz");
  oled.display();

  #ifdef ENABLE_ATC
    Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
  #endif
}

void do_ota() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0,0);
    oled.setTextColor(BLACK,WHITE);
    oled.print("OTA");
    oled.setTextColor(WHITE);
    oled.setTextSize(2);
    oled.setCursor(0,16);

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    oled.clearDisplay();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    oled.fillRect(0,15,w,h,BLACK);
    oled.setCursor(0,16);
    oled.printf("PROG: %u%%\r", (progress / (total / 100)));
    oled.display();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
      oled.print("AUTH FAIL");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
      oled.print("BEG FAIL");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
      oled.print("CONN FAIL");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
      oled.print("REC FAIL");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
      oled.print("END FAIL");
    }
  });
  ArduinoOTA.begin();
  Serial.println("<< OTA READY >>");
}

int update_time() {

  DateTime now = rtc.now();

  // int epoch = now.unixtime();

  int hours = now.hour();
  int minutes = now.minute();
  int seconds = now.second();

  // oled.clearDisplay();
  oled.setCursor((w/2) - 24,0);
  oled.printf("%02d:%02d:%02d",hours,minutes,seconds);

  return now.unixtime();

}

void do_publish(char* msg) {

  int epoch = update_time();

  sprintf(mqtt_payload,"%d,%d,%s",epoch,radio.RSSI,(char*)msg);

  Serial.print("\n");
  Serial.println(mqtt_payload);
  // publish to broker
  client.publish("climclam",mqtt_payload);
  // the MQTT borker (yes, "borker") needs a delay here
  // to be able to send published payloads to subbed clients
  // I don't know...
  delay(1);
  Serial.println("<< PUBBED >>");

}

// ========================[ SETUP ]========================

void setup() {
  Serial.begin(SERIAL_BAUD);
  // while (!Serial) { delay(1); }

  delay(100);   // delay for oled init - otherwise have to reset to get it up

  // init oled and settings
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0,0);
  oled.clearDisplay();
  oled.display();

  rtc.begin(DateTime(2014, 1, 21, 0, 0, 0));

  setup_wifi();
  delay(2000);
  setup_mqtt();
  delay(2000);
  setup_radio();
  delay(2000);

  timeClient.begin();
  timeClient.update();

  int epoch = timeClient.getEpochTime();
  rtc.adjust(DateTime(epoch));

  oled.clearDisplay();
  oled.setTextColor(BLACK,WHITE);
  oled.setCursor(0,0);
  oled.print("STBY");
  oled.display();
  oled.setTextColor(WHITE);

  do_ota();
}

byte ackCount = 0;

// ========================[ LOOP ]========================

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    setup_mqtt();
  }
  // MQTT client loop - remain connected to server
  client.loop();

  if (radio.receiveDone())
  {
    Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");

    if (promiscuousMode)
    {
      Serial.print("to [");Serial.print(radio.TARGETID, DEC);Serial.print("] ");
    }
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print((char)radio.DATA[i]);
    }
    Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);Serial.print("]");

    oled.clearDisplay();

    oled.setCursor(0,0);
    oled.setTextColor(WHITE);
    if (radio.SENDERID == 3) {
      oled.print("CLIM");
      // publish to broker server
      do_publish((char*)radio.DATA);
    } else if (radio.SENDERID == 2){
      oled.print("YLW");
      update_time();
    }

    oled.setCursor(108,0);
    oled.setTextColor(WHITE);
    oled.print(radio.RSSI);

    oled.setCursor(0,14);
    oled.setTextColor(WHITE);
    oled.print((char*)radio.DATA);

    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent.");

      // When a node requests an ACK, respond to the ACK
      // and also send a packet requesting an ACK (every 3rd one only)
      // This way both TX/RX NODE functions are tested on 1 end at the GATEWAY
      if (ackCount++%3==0)
      {
        Serial.print(" Pinging node ");
        Serial.print(theNodeID);
        Serial.print(" - ACK...");
        delay(3); //need this when sending right after reception .. ?
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("ok!");
        else Serial.print("nothing");
      }
    }
    oled.display();
    Serial.println();
    Blink(LED,3);
  }
}
