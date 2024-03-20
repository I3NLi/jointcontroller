/*!
 * \name        const
 * \author      Infineon Technologies AG
 * \copyright   2020-2024 Infineon Technologies AG
 * \version     1.0.0
 * \brief       macros and other const variables for the Arctos2XMC robot
 *
 * \attention
 * Ths setup reduces variable memory so that the sensorType can run on MCUs
 * with low memory by shifting global variable memory to main program memory.
 */


// For the TLE5012
#include "Arduino.h"


// Pin selection for SPI1 on X1
#define PIN_SPI1_SS0        36                         //! P0.3
#define PIN_SPI1_SS1        64                         //! P0.2
#define PIN_SPI1_SS2        66                         //! P0.4
#define PIN_SPI1_SS3        35                         //! P0.5
#define PIN_SPI1_MOSI       37                         //! P0.1
#define PIN_SPI1_MISO       63                         //! P0.0
#define PIN_SPI1_SCK        38                         //! P0.10
// Pin selection for SPI2 on X2
#define PIN_SPI2_SS0        94                         //! P0.12
#define PIN_SPI2_SS1        93                         //! P0.15
#define PIN_SPI2_SS2        71                         //! P3.14
#define PIN_SPI2_SS3        70                         //! P0.14
#define PIN_SPI2_MOSI       69                         //! P3.11
#define PIN_SPI2_MISO       95                         //! P3.12
#define PIN_SPI2_SCK        68                         //! P3.13

// IFX007 Board 1
#define PIN_PWM_U1          11                          //! P
#define PIN_PWM_V1          10                          //! P
#define PIN_PWM_W1          9                           //! P
#define PIN_PWM_EN_U1       6                           //! P
#define PIN_PWM_EN_V1       5                           //! P
#define PIN_PWM_EN_W1       3                           //! P

// IFX007 Board 2
#define PIN_PWM_U2          11                          //! P
#define PIN_PWM_V2          10                          //! P
#define PIN_PWM_W2          9                           //! P
#define PIN_PWM_EN_U2       6                           //! P
#define PIN_PWM_EN_V2       5                           //! P
#define PIN_PWM_EN_W2       3                           //! P

// IFX007 Board 3
#define PIN_PWM_U3          11                          //! P
#define PIN_PWM_V3          10                          //! P
#define PIN_PWM_W3          9                           //! P
#define PIN_PWM_EN_U3       6                           //! P
#define PIN_PWM_EN_V3       5                           //! P
#define PIN_PWM_EN_W3       3                           //! P

// IFX007 Board 4
#define PIN_PWM_U4          11                          //! P
#define PIN_PWM_V4          10                          //! P
#define PIN_PWM_W4          9                           //! P
#define PIN_PWM_EN_U4       6                           //! P
#define PIN_PWM_EN_V4       5                           //! P
#define PIN_PWM_EN_W4       3                           //! P

// IFX007 Board 5
#define PIN_PWM_U5          11                          //! P
#define PIN_PWM_V5          10                          //! P
#define PIN_PWM_W5          9                           //! P
#define PIN_PWM_EN_U5       6                           //! P
#define PIN_PWM_EN_V5       5                           //! P
#define PIN_PWM_EN_W5       3                           //! P

// IFX007 Board 6
#define PIN_PWM_U6          11                          //! P
#define PIN_PWM_V6          10                          //! P
#define PIN_PWM_W6          9                           //! P
#define PIN_PWM_EN_U6       6                           //! P
#define PIN_PWM_EN_V6       5                           //! P
#define PIN_PWM_EN_W6       3                           //! P

// Gear factors for each of the six joint axes
#define gearFactorX         13.8
#define gearFactorY         150.0
#define gearFactorZ         110.0
#define gearFactorA         6.0
#define gearFactorB         5.5
#define gearFactorC         5.5
