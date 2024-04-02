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
#define PIN_SPI1_SS0        35                         //! P0.5 X axes
#define PIN_SPI1_SS1        64                         //! P0.2 Y axes
#define PIN_SPI1_SS2        66                         //! P0.4 not used
#define PIN_SPI1_SS3        36                         //! P0.3 not used
#define PIN_SPI1_MOSI       37                         //! P0.1
#define PIN_SPI1_MISO       63                         //! P0.0
#define PIN_SPI1_SCK        38                         //! P0.10

// Pin selection for SPI2 on X2
#define PIN_SPI2_SS0        94                         //! P0.12 Z axes
#define PIN_SPI2_SS1        93                         //! P0.15 A axes
#define PIN_SPI2_SS2        92                         //! (70,P0.14) changed here to P3.3 CR axes
#define PIN_SPI2_SS3        91                         //! (71,P3.14) changed here to P0.8 CL axes
#define PIN_SPI2_MOSI       69                         //! P3.11
#define PIN_SPI2_MISO       95                         //! P3.12
#define PIN_SPI2_SCK        68                         //! P3.13

// IFX007 Board used as Shield on Arduino Header
// This is only for test
#define PIN_PWM_U_SHIELD    11                          //! P3.8
#define PIN_PWM_V_SHIELD    10                          //! P3.10
#define PIN_PWM_W_SHIELD    9                           //! P1.11
#define PIN_PWM_EN_U_SHIELD 6                           //! P1.1
#define PIN_PWM_EN_V_SHIELD 5                           //! P2.12
#define PIN_PWM_EN_W_SHIELD 3                           //! P2.11

// IFX007 Board 1 refer as X
#define PIN_PWM_U1          9                           //! P1.11
#define PIN_PWM_V1          10                          //! P3.10
#define PIN_PWM_W1          11                          //! P3.8
#define PIN_PWM_EN1         71                          //! P3.14
// #define PIN_PWM_EN_U1       4                           //! P1.8  (setting for shied 3,P1.1)
// #define PIN_PWM_EN_V1       7                           //! P1.9  (setting for shied 5,P2.12)
// #define PIN_PWM_EN_W1       8                           //! P1.10 (setting for shied 6,P2.11)

// IFX007 Board 2 refer as Y
#define PIN_PWM_U2          3                           //! P1.1
#define PIN_PWM_V2          5                           //! P2.12
#define PIN_PWM_W2          6                           //! P2.11
#define PIN_PWM_EN2         72                          //! P0.7
// #define PIN_PWM_EN_U2       2                           //! P1.0
// #define PIN_PWM_EN_V2       12                          //! P3.7
// #define PIN_PWM_EN_W2       13                          //! P3.9

// IFX007 Board 3 refer as Z
#define PIN_PWM_U3          34                          //! P3.4
#define PIN_PWM_V3          36                          //! P0.3
#define PIN_PWM_W3          66                          //! P0.6
#define PIN_PWM_EN3         73                          //! P1.2
// #define PIN_PWM_EN_U3       67                          //! P0.11
// #define PIN_PWM_EN_V3       39                          //! P3.2
// #define PIN_PWM_EN_W3       40                          //! P3.1

// IFX007 Board 4 refer as A
#define PIN_PWM_U4          51                          //! P5.11
#define PIN_PWM_V4          61                          //! P3.0
#define PIN_PWM_W4          62                          //! P0.9
#define PIN_PWM_EN4         74                          //! P6.1
// #define PIN_PWM_EN_U4       49                          //! P2.13
// #define PIN_PWM_EN_V4       50                          //! P5.10
// #define PIN_PWM_EN_W4       52                          //! P1.14

// IFX007 Board 5 refer as CL
#define PIN_PWM_U5          70                          //! P0.14
#define PIN_PWM_V5          76                          //! P6.5
#define PIN_PWM_W5          77                          //! P1.15
#define PIN_PWM_EN5         75                          //! P5.3
// #define PIN_PWM_EN_U5       71                          //! P3.14
// #define PIN_PWM_EN_V5       72                          //! P0.7
// #define PIN_PWM_EN_W5       73                          //! P1.2

// IFX007 Board 6 refer as CR
#define PIN_PWM_U6          79                          //! P5.3
#define PIN_PWM_V6          80                          //! P5.5
#define PIN_PWM_W6          81                          //! P5.7
#define PIN_PWM_EN6         82                          //! P2.6
// #define PIN_PWM_EN_U6       74                          //! P6.1
// #define PIN_PWM_EN_V6       88                          //! P6.4
// #define PIN_PWM_EN_W6       78                          //! P5.1

// IFX007 Board 7 refer as GRIPPER
// #define PIN_PWM_U6          79                          //! P5.3
// #define PIN_PWM_V6          80                          //! P5.5
// #define PIN_PWM_W6          81                          //! P5.7
// #define PIN_PWM_EN_U6       74                          //! P6.1
// #define PIN_PWM_EN_V6       88                          //! P6.4
// #define PIN_PWM_EN_W6       82                          //! P2.6

// Gear factors for each of the six joint axes
#define gearFactor_X        13.8
#define gearFactor_Y        150.0
#define gearFactor_Z        110.0
#define gearFactor_A        6.0
#define gearFactor_CR       5.5
#define gearFactor_CL       5.5

// minimal range limits
#define minLimit_X         -200.0
#define minLimit_Y         -125.0
#define minLimit_Z         -50.0
#define minLimit_A         -200.0
#define minLimit_B         -100.0
#define minLimit_C         -200.0
#define minLimit_CR        0
#define minLimit_CL        0

// minimal range limits
#define maxLimit_X         200.0
#define maxLimit_Y         75.0
#define maxLimit_Z         100.0
#define maxLimit_A         200.0
#define maxLimit_B         100.0
#define maxLimit_C         200.0
#define maxLimit_CL        0
#define maxLimit_CR        0

// default home position
#define defaultPos_X       0.0
#define defaultPos_Y       0.0
#define defaultPos_Z       0.0
#define defaultPos_A       0.0
#define defaultPos_B       0.0
#define defaultPos_C       0.0
#define defaultPos_CL      0.0
#define defaultPos_CR      0.0
