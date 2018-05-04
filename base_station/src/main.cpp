// BASE CONTROLLER
// RECEIVES DATA FROM REMOTE STATION
// THEN SENDS DATA VIA MQTT TO A SERVER
#include <Arduino.h>
#include <creds.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Adafruit_SSD1306.h>
// #include "../lib/Adafruit_ILI9341/Adafruit_ILI9341.h"
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <RTClib.h>

#if defined(ESP8266)
  #define RFM69_CS      2     // "E"
  #define RFM69_INT     15    // "B"
  #define RFM69_RST     16    // "D"
  #define LED 0
#elif defined(ESP32)
  #define RFM69_CS      14    // "E"
  #define RFM69_INT     33    // "B"
  #define RFM69_RST     32    // "D"
  #define LED 13
#endif


#define RF69_FREQ 915.0

// who am i? (server address)
#define MY_ADDRESS 1

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

Adafruit_SSD1306 display = Adafruit_SSD1306();


WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

RTC_Millis rtc;

DateTime sleepDur;
bool TXcheck = 0;

const int wait = 200;
char mqtt_msg[100];

const int h = 32;
const int w = 128;

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(ssid);

  display.print("WiFi...");
  display.display();

  WiFi.mode(WIFI_STA);

  // WiFi.begin(ssid, pass);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  display.print("CONNECTED");
  display.display();

  delay(wait);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32_Client-";
    clientId += String(random(0xffff), HEX);

    display.setTextSize(1);
    display.setCursor(0,12);
    display.print("MQTT...");
    display.display();
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      display.print("CONNECTED");
      display.display();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      display.print("FAILED");
      display.display();

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void do_ota() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(2,0);
    display.print("OTA");
    display.setTextSize(2);
    display.setCursor(2,14);
    display.printf("PROG: %u%%\r", (progress / (total / 100)));
    display.display();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  // while (!Serial) { delay(1); } // wait until serial console is open, remove if not tethered to computer

  delay(100);    // delay for oled init (have to reset to get it up...)

  display.begin(SSD1306_SWITCHCAPVCC,0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0,0);
  display.display();

  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  rtc.begin(DateTime(2014, 1, 21, 0, 0, 0));

  setup_wifi();
  timeClient.begin();
  client.setServer(server, 1883);

  if (!client.connected()) {
    reconnect();
  }

  timeClient.update();

  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  display.setCursor(0,24);
  display.print("Radio INIT...");
  display.display();

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    display.print("FAILED");
    display.display();
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  display.print("SUCCESS");
  display.display();
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  delay(1000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,10);
  display.print("STBY");
  display.display();

  do_ota();
}


// Dont put this on the stack:
uint8_t data[] = "ACK from Base Station";
// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

void loop() {
  ArduinoOTA.handle();

  if (!client.connected()) {
    reconnect();
  }
  // MQTT client loop - remain connected to server
  client.loop();

  if (rf69_manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (rf69_manager.recvfromAck(buf, &len, &from)) {
      buf[len] = 0; // zero out remaining string

      Serial.print("Got packet from #"); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf69.lastRssi());
      Serial.print("] : ");
      Serial.println((char*)buf);
      Blink(LED, 40, 3); //blink LED 3 times, 40ms between blinks

      timeClient.update();

      int epoch = timeClient.getEpochTime();
      int hours = timeClient.getHours();
      int minutes = timeClient.getMinutes();
      int seconds = timeClient.getSeconds();

      if (from == 2) {                      // if from bme280

        // set up vars for parsing
        char msg_data_tempC[20] = "";
        char msg_data_tempF[20] = "";
        char msg_data_hum[20] = "";
        char msg_data_pres[20] = "";
        char msg_data_alt[20] = "";
        char msg_data_bat[20] = "";
        char msg_data_slp[10] = "";

        //parse data from remote station
        sscanf((char*)buf, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%s",msg_data_tempC,msg_data_tempF,msg_data_hum,msg_data_pres,msg_data_alt,msg_data_bat,msg_data_slp);

        sprintf(mqtt_msg,"%d,%s",epoch,(char*)buf);
        Serial.println(mqtt_msg);

        client.publish("BME280",mqtt_msg);
        // the MQTT borker (yes, "borker") needs a delay here
        // to be able to send published payloads to subbed clients
        // I don't know...
        delay(1);

        rtc.adjust(DateTime(epoch));
        int slpInt;
        sscanf(msg_data_slp,"%d",&slpInt);
        sleepDur = epoch + slpInt;

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(2,2);
        display.print("RSSI ");
        display.print(rf69.lastRssi());

        display.setCursor(2,16);
        display.setTextSize(2);
        display.printf("%02d:%02d",hours - 4,minutes);

        display.setCursor(w/2+10,2);
        display.setTextSize(1);
        display.printf("BAT %s",msg_data_bat);
        display.print("%");
        display.setCursor(w/2+10,12);
        display.printf("%s F",msg_data_tempF);
        display.setCursor(w/2+10,22);
        display.printf("%s ",msg_data_hum);
        display.print("%");
        display.display();
      } else {
        display.print("LAST PKT:");
      }

      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(data, sizeof(data), from))
        Serial.println("Sending failed (no ack)");

      // TXcheck = 1;
    }
  } else {
    display.clearDisplay();
    display.setCursor(10,10);
    display.setTextSize(1);
    display.print("rf69_manager fail");
    display.display();
  }

  // // check to see if there's been contact with the remote station
  // if (TXcheck) {
  //   // count since last packet transmission
  //   TimeSpan countDown = sleepDur - rtc.now();
  //
  //   if (countDown.seconds() < 0 && countDown.minutes() == 0) {
  //     display.fillRect(0,12,w/2,h,BLACK);
  //     display.setTextSize(2);
  //     display.setCursor(2,14);
  //     display.print("LATE!");
  //   } else {
  //     display.fillRect(0,12,w/2,h,BLACK);
  //     display.setTextSize(2);
  //     display.setCursor(2,14);
  //     display.printf("%02u:%02u",countDown.minutes(),countDown.seconds());
  //   }
  // }

  display.display();

}
