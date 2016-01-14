#ifndef direct_pin_read_h_
#define direct_pin_read_h_

#if defined(__AVR__) || (defined(__arm__) && defined(CORE_TEENSY))

#define IO_REG_TYPE			uint8_t
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#elif defined(__SAM3X8E__)  // || defined(ESP8266)

#define IO_REG_TYPE			uint32_t
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)

#elif defined(__PIC32MX__)

#define IO_REG_TYPE			uint32_t
#define PIN_TO_BASEREG(pin)             (portModeRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)	(((*(base+4)) & (mask)) ? 1 : 0)

/* ESP8266 v2.0.0 Arduino workaround for bug https://github.com/esp8266/Arduino/issues/1110
   Once ESP8266 Arduino v2.1.0 is released, this #elif should be removed and line 11 of this
   file should read:
   #elif defined(__SAM3X8E__) || defined(ESP8266)
*/
#elif defined(ESP8266)

#define IO_REG_TYPE			uint32_t
#define PIN_TO_BASEREG(pin)             ((volatile uint32_t *)(0x60000000+(0x318))) //Bypassing erroneous preprocessor macros
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define DIRECT_PIN_READ(base, mask)     (((*(base)) & (mask)) ? 1 : 0)
/* End of workaround */

#endif

#endif
