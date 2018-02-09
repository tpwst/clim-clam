# Clim-Clam

Attempt at a relatively low power "ambient climate sensor" project using MQTT.

![dumb "schematic"][header]
## Warning
This is not optimized. This is less of a guide and more for my documentation. I'm sorry.

## Hardware

### Base Station
* [Adafruit HUZZAH32 ESP32 Feather Board](https://www.adafruit.com/product/3405)
* [FeatherWing OLED - 128x32 OLED](https://www.adafruit.com/product/2900)
* [Adafruit Radio FeatherWing - RFM69HCW 900MHz - RadioFruit](https://www.adafruit.com/product/3229)
* [FeatherWing Tripler](https://www.adafruit.com/product/3417)

### Remote Station
* [Adafruit Feather M0 RFM69HCW Packet Radio - 868 or 915 MHz - RadioFruit](https://www.adafruit.com/product/3176)
* [Adafruit BME280 Temperature, Humidity and Pressure Sensor](https://www.adafruit.com/product/2652)
* [Waveshare 1.54in e-paper display (3 color - black, white + red)](https://www.amazon.com/Waveshare-1-54inch-three-color-resolution-controller/dp/B074NYX1C4/ref=pd_sbs_229_2?_encoding=UTF8&pd_rd_i=B074NYX1C4&pd_rd_r=X46A33QDZVWT6SV47NG6&pd_rd_w=NKe6A&pd_rd_wg=cCjhA&psc=1&refRID=X46A33QDZVWT6SV47NG6)  

## WiFi Credentials

You will need to include your own credential header files in the "src" directories of both stations. Only need the encryption key for the remote station.

### base_station/src/creds.h
```c
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h>
#endif

const char* ssid = "your_ssid";
const char* pass = "your_ssid_password";
IPAddress server(ip,address,of,mqtt,server);

// encryption key - must match for communicating devices
uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
```
### remote_station/src/creds.h
```c
// encryption key - must match for communicating devices
uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
```

## Wrench/Socket Sizes (for my sanity)
| Part | Size |
| - | - |
| Antenna | 5/16" |
| Aviation Connector | 5/8" |
| Momentary reset switch | 10mm |
| Toggle switch | 10mm |
| Barrel jack | 7/16" |
## To do
* more accurate battery monitoring
* parse packet message and display relevant data
* decide on "counter" direction
* reduce power consumption
* take updated pictures

## Images
![image of closed enclosure][img1]
![image of open enclosure][img2]

[header]: https://github.com/tpwst/clim-clam/blob/master/img/radio_barl-01.png
[img1]: https://github.com/tpwst/clim-clam/blob/master/img/v1_01.jpg
[img2]: https://github.com/tpwst/clim-clam/blob/master/img/v1_02.jpg
