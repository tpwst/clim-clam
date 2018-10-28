# Clim-Clam

Attempt at a relatively low power "ambient climate sensor" project using Arduino.

![dumb "schematic"][header]
## Warning
This is not optimized. This is less of a guide and more for my documentation. I'm sorry.

## Hardware

### Gateway
* [Adafruit Feather HUZZAH with ESP8266 WiFi](https://www.adafruit.com/product/2821)
* [FeatherWing OLED - 128x32 OLED](https://www.adafruit.com/product/2900)
* [Adafruit Radio FeatherWing - RFM69HCW 900MHz - RadioFruit](https://www.adafruit.com/product/3229)
* [FeatherWing Tripler](https://www.adafruit.com/product/3417)

### Weather Node
* [Moteino M0](https://lowpowerlab.com/shop/product/184)
* [SparkFun Atmospheric Sensor Breakout - BME280](https://www.sparkfun.com/products/13676)
* [Waveshare 1.54in e-paper display (3 color - black + white)](https://www.amazon.com/gp/product/B0728BJTZC/ref=oh_aui_detailpage_o07_s00?ie=UTF8&psc=1)
* [Lithium Ion Polymer Battery - 3.7v 2500mAh](https://www.adafruit.com/product/328)

## WiFi and RF Network Credentials

You will need to include your own credential header files in the "src" directories of both the gateway and the node. Only need the encryption key for the node.

### gateway/src/creds.h
```c
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h>
#endif

// exactly the same 16 chars/bytes on all nodes
#define ENCRYPTKEY "someKindOfString"

const char* ssid = "your_ssid";
const char* pass = "your_ssid_password";
IPAddress server(ip,address,of,mqtt,server);
```

### node/src/creds.h
```c
// exactly the same 16 chars/bytes on all nodes
#define ENCRYPTKEY "someKindOfString"
```

## Notes
### Power
#### (I don't have the equipment to measure such low current so this is probably all bs and should be disregarded.)
This is with sleeping the controller, sensor and transmitter.

| Period | Current Draw | Duration |
| - | - | - |
| during code execution | ~15mA | 1.2 sec |
| during sleep | ~ 610uA | 15 min |

With a 1200mAh battery will potentially run for 78 days.

### Dev DIP Switches
| Switch Position | Operation |
| - | - |
| U,D,D,D | wait for active serial monitor then run |
| D,D,D,D | 15 minute sample rate |
| D,U,D,D | 5 minute sample rate |
| D,D,U,D | 1 minute sample rate |
| D,U,U,D | 30 second sample rate|

### Wrench/Socket Sizes (for my sanity)
| Part | Size |
| - | - |
| Antenna | 5/16" |
| Aviation Connector | 5/8" |
| Momentary reset switch | 10mm |
| Toggle switch | 10mm |
| Barrel jack | 7/16" |

### To do
* more accurate battery monitoring
* ~~parse packet message and display relevant data~~
* reduce power consumption
* ~~rework expected TX counter~~
* **Better yet, replace "countdown" with a log**
* ~~debug switch (if low wait for serial monitor to run/set shorter sleep period)~~
* LWT for failed connections
* Adjust for multiple nodes
* Sparkfun lib has trouble detecting failed bme init - when it fails it just hangs (no way to send further output...need to figure out work around)

### To remember

* when programming/flashing the hex files to the m0, put it into bootloader mode (causes my machine to shutdown unexpectedly...)

### Images
#### (Don't look at the flux residue)

![closed enclosure][img1]
![open enclosure][img2]
![external enclosure ports][img3]
![base station mounted on wall displaying data from remote station][img4]
![clim-clam version 2][img5]

[header]: img/radio_barl-02.png
[img1]: img/v1_01.jpg
[img2]: img/rs_01.png
[img3]: img/rs_02.png
[img4]: img/bs_01.png
[img5]: img/v2_01.jpg
