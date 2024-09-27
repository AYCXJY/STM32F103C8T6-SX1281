#ifndef DEVICE_NAME
#define DEVICE_NAME "ELRS 2400RX"
#endif

// GPIO pin definitions
#define GPIO_PIN_NSS         PA4
#define GPIO_PIN_MOSI        PA7
#define GPIO_PIN_MISO        PA6
#define GPIO_PIN_SCK         PA5

#define GPIO_PIN_DIO1        PB0
#define GPIO_PIN_RST         PB4
#define GPIO_PIN_BUSY        PC15

#define GPIO_PIN_TX_EN       PB7
#define GPIO_PIN_RX_EN       PB6


// #define GPIO_PIN_RCSIGNAL_RX PB7  // USART1, AFAIO
// #define GPIO_PIN_RCSIGNAL_TX PB6  // USART1, AFAIO

// #define GPIO_PIN_LED_RED     PB5

// Output Power - use default SX1280

#define POWER_OUTPUT_FIXED 13 //MAX power for 2400 RXes that doesn't have PA is 12.5dbm

