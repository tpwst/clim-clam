#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
// 24 x 24 gridicons-arrow-down
const unsigned char gridicons_arrow_down[] PROGMEM = { /* 0X01,0X01,0XB4,0X00,0X40,0X00, */
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF,
    0xE7, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xE7, 0xFF,
    0xFF, 0xE7, 0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xE7,
    0xFF, 0xF3, 0xE7, 0xCF, 0xF1, 0xE7, 0x8F, 0xF8,
    0xE7, 0x1F, 0xFC, 0x66, 0x3F, 0xFE, 0x24, 0x7F,
    0xFF, 0x00, 0xFF, 0xFF, 0x81, 0xFF, 0xFF, 0xC3,
    0xFF, 0xFF, 0xE7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
