// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------

#if IOTDATA_CONFIG_CARRIER

// AE SDC carrier PCB. Avoids USB-Serial-JTAG (GPIO18/19) and the strapping pins (GPIO2/8/9).
#define PIN_E22_M0           GPIO_NUM_4  /* E22 pin (1) */
#define PIN_E22_M1           GPIO_NUM_21 /* E22 pin (2) */
#define PIN_E22_RXD          GPIO_NUM_20 /* E22 pin (3) ESP TX -> module RXD */
#define PIN_E22_TXD          GPIO_NUM_10 /* E22 pin (4) module TXD -> ESP RX */
#define PIN_E22_AUX          GPIO_NUM_7  /* E22 pin (5) */
#define PIN_E22_VCC                      /* E22 pin (6) */
#define PIN_E22_GND                      /* E22 pin (7) */
#define PIN_I2C_SDA          GPIO_NUM_5  /* I2C header pin (3) */
#define PIN_I2C_SCL          GPIO_NUM_6  /* I2C header pin (4) */
#define PIN_BATTERY_ADC      GPIO_NUM_0  /* patch header — divider midpoint (ADC1_CH0) */
#define PIN_BATTERY_EN       GPIO_NUM_1  /* patch header — divider enable, high = on    */

#define BATTERY_OFFSET_MV    155  /* this board's V_CE(sat) + divider tolerance */
#define BATTERY_LIION_MV_MAX 4170 /* what your charger leaves the M35A resting at */
#define BATTERY_LIION_MV_MIN 3200 /* where the NODE stops, not the cell's 2.5V cutoff */

#else

// Original bench wiring.
#define PIN_E22_M0      GPIO_NUM_5 /* E22 pin (1) */
#define PIN_E22_M1      GPIO_NUM_6 /* E22 pin (2) */
#define PIN_E22_RXD     GPIO_NUM_7 /* E22 pin (3) ESP TX -> module RXD */
#define PIN_E22_TXD     GPIO_NUM_8 /* E22 pin (4) module TXD -> ESP RX */
#define PIN_E22_AUX     GPIO_NUM_9 /* E22 pin (5) */
#define PIN_E22_VCC                /* E22 pin (6) */
#define PIN_E22_GND                /* E22 pin (7) */
#define PIN_I2C_SDA     GPIO_NUM_3
#define PIN_I2C_SCL     GPIO_NUM_4
#define PIN_BATTERY_ADC GPIO_NUM_0 /* divider midpoint (ADC1_CH0) -- same pair as the carrier map */
#define PIN_BATTERY_EN  GPIO_NUM_1 /* divider enable, high = on   */

#endif

#define E22_UART          UART_NUM_1
#define E22_UART_BUF_SIZE 512

/* The pack, which is a property of the cell fitted rather than of the wiring -- so it sits outside
   the carrier/bench split, unlike the divider constants above. 0 = unknown, and an unknown capacity
   is reported ABSENT rather than as zero, because zero would be a claim. */
#ifndef BATTERY_CAPACITY_MAH
#define BATTERY_CAPACITY_MAH 0 /* a Molicel INR18650-M35A is 3350 */
#endif

// -----------------------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------------------
