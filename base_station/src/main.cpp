// BASE CONTROLLER
// RECEIVES DATA FROM REMOTE STATION
// THEN SENDS DATA VIA MQTT TO A SERVER
#include <Arduino.h>
#include <creds.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Adafruit_SSD1306.h>
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

Adafruit_SSD1306 oled = Adafruit_SSD1306();

WiFiClient espClient;
PubSubClient client(espClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

RTC_Millis rtc;

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

  oled.print("WiFi...");
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

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  oled.print("CONNECTED");
  oled.display();

  delay(wait);
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32_Client-";
    clientId += String(random(0xffff), HEX);

    oled.setCursor(0,12);
    oled.print("MQTT...");
    oled.display();
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      oled.print("CONNECTED");
      oled.display();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      oled.print("FAILED");
      oled.display();

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
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(2,0);
    oled.print("OTA");
    oled.setTextSize(2);
    oled.setCursor(2,14);
    oled.printf("PROG: %u%%\r", (progress / (total / 100)));
    oled.display();
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

  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.clearDisplay();
  oled.setCursor(0,0);
  oled.display();

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

  oled.setCursor(0,24);
  oled.print("Radio INIT...");
  oled.display();

  if (!rf69_manager.init()) {
    Serial.println("RFM69 radio init failed");
    oled.print("FAILED");
    oled.display();
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  oled.print("SUCCESS");
  oled.display();
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
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(2,0);
  oled.print("STBY");
  oled.display();

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

      // set up vars for parsing
      char msg_data_tempC[20] = "";
      char msg_data_tempF[20] = "";
      char msg_data_hum[20] = "";
      char msg_data_pres[20] = "";
      char msg_data_alt[20] = "";
      char msg_data_bat[20] = "";

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

      //parse data from remote station
      sscanf((char*)buf, "%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%s",msg_data_tempC,msg_data_tempF,msg_data_hum,msg_data_pres,msg_data_alt,msg_data_bat);

      sprintf(mqtt_msg,"%d,%s",epoch,(char*)buf);
      Serial.println(mqtt_msg);

      client.publish("BME280",mqtt_msg);
      // the MQTT borker (yes, "borker") needs a delay here for some reason
      // to be able to send published payloads to subbed clients
      // I don't know...
      delay(1);

      rtc.adjust(DateTime(2014, 1, 21, 0, 0, 0));

      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setCursor(2,2);
      if (from == 2) {
        oled.print("> BME280");
      } else {
        oled.print("LAST PKT:");
      }
      oled.setCursor(2,16);
      oled.setTextSize(2);
      oled.printf("%02d:%02d",minutes,seconds);

      oled.setCursor(w/2+10,2);
      oled.setTextSize(1);
      oled.printf("BAT %s",msg_data_bat);
      oled.print("%");
      oled.setCursor(w/2+10,12);
      oled.printf("%s F",msg_data_tempF);
      oled.setCursor(w/2+10,22);
      oled.printf("%s ",msg_data_hum);
      oled.print("%");
      oled.display();

      // Send a reply back to the originator client
      if (!rf69_manager.sendtoWait(data, sizeof(data), from))
        Serial.println("Sending failed (no ack)");
    }
  }

  // count since last packet transmission
  // swap out for countdown?
  DateTime now = rtc.now();

  oled.fillRect(0,12,w/2,h,BLACK);
  oled.setTextSize(2);
  oled.setCursor(2,14);
  oled.printf("%02u:%02u",now.minute(),now.second());
  oled.display();

}
