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

// IFX007 Board 2 refer as Y
#define PIN_PWM_U2          3                           //! P1.1
#define PIN_PWM_V2          5                           //! P2.12
#define PIN_PWM_W2          6                           //! P2.11
#define PIN_PWM_EN2         72                          //! P0.7

// IFX007 Board 3 refer as Z
#define PIN_PWM_U3          34                          //! P3.4
#define PIN_PWM_V3          36                          //! P0.3
#define PIN_PWM_W3          66                          //! P0.6
#define PIN_PWM_EN3         73                          //! P1.2

// IFX007 Board 4 refer as A
#define PIN_PWM_U4          51                          //! P5.11
#define PIN_PWM_V4          61                          //! P3.0
#define PIN_PWM_W4          62                          //! P0.9
#define PIN_PWM_EN4         74                          //! P6.1

// IFX007 Board 5 refer as CL
#define PIN_PWM_U5          70                          //! P0.14
#define PIN_PWM_V5          76                          //! P6.5
#define PIN_PWM_W5          77                          //! P1.15
#define PIN_PWM_EN5         82                          //! P2.6

// IFX007 Board 6 refer as CR
#define PIN_PWM_U6          79                          //! P5.3
#define PIN_PWM_V6          80                          //! P5.5
#define PIN_PWM_W6          81                          //! P5.7
#define PIN_PWM_EN6         83                          //! P5.6

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
#define minLimit_Y         -120.0 //-125 reduced no collision detecting
#define minLimit_Z         -100.0 // -50 reverse
#define minLimit_A         -200.0
#define minLimit_B         -100.0
#define minLimit_C         -200.0
#define minLimit_CR        0
#define minLimit_CL        0

// minimal range limits
#define maxLimit_X         200.0
#define maxLimit_Y         70.0 // 75 reduced
#define maxLimit_Z         50.0 // 100 reverse 
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

// default home position
#define epsilon_X          2.0
#define epsilon_Y          1.0
#define epsilon_Z          1.0
#define epsilon_A          6.0
#define epsilon_B          0.0
#define epsilon_C          0.0
#define epsilon_CL         0.0
#define epsilon_CR         0.0

/**
 * @brief GRBL and G-Code definitions
 *  
 */

#define MAX_PROG_STACK 20
#define LINE_BUFFER_SIZE 256
#define MAX_INT_DIGITS   8

// Define Grbl status codes. Valid values (0-255)
#define STATUS_OK 0
#define STATUS_EXPECTED_COMMAND_LETTER 1
#define STATUS_BAD_NUMBER_FORMAT 2
#define STATUS_INVALID_STATEMENT 3
#define STATUS_NEGATIVE_VALUE 4
#define STATUS_SETTING_DISABLED 5
#define STATUS_SETTING_STEP_PULSE_MIN 6
#define STATUS_SETTING_READ_FAIL 7
#define STATUS_IDLE_ERROR 8
#define STATUS_SYSTEM_GC_LOCK 9
#define STATUS_SOFT_LIMIT_ERROR 10
#define STATUS_OVERFLOW 11
#define STATUS_MAX_STEP_RATE_EXCEEDED 12
#define STATUS_CHECK_DOOR 13
#define STATUS_LINE_LENGTH_EXCEEDED 14
#define STATUS_TRAVEL_EXCEEDED 15
#define STATUS_INVALID_JOG_COMMAND 16
#define STATUS_SETTING_DISABLED_LASER 17

#define STATUS_GCODE_UNSUPPORTED_COMMAND 20
#define STATUS_GCODE_MODAL_GROUP_VIOLATION 21
#define STATUS_GCODE_UNDEFINED_FEED_RATE 22
#define STATUS_GCODE_COMMAND_VALUE_NOT_INTEGER 23
#define STATUS_GCODE_AXIS_COMMAND_CONFLICT 24
#define STATUS_GCODE_WORD_REPEATED 25
#define STATUS_GCODE_NO_AXIS_WORDS 26
#define STATUS_GCODE_INVALID_LINE_NUMBER 27
#define STATUS_GCODE_VALUE_WORD_MISSING 28
#define STATUS_GCODE_UNSUPPORTED_COORD_SYS 29
#define STATUS_GCODE_G53_INVALID_MOTION_MODE 30
#define STATUS_GCODE_AXIS_WORDS_EXIST 31
#define STATUS_GCODE_NO_AXIS_WORDS_IN_PLANE 32
#define STATUS_GCODE_INVALID_TARGET 33
#define STATUS_GCODE_ARC_RADIUS_ERROR 34
#define STATUS_GCODE_NO_OFFSETS_IN_PLANE 35
#define STATUS_GCODE_UNUSED_WORDS 36
#define STATUS_GCODE_G43_DYNAMIC_AXIS_ERROR 37
#define STATUS_GCODE_MAX_VALUE_EXCEEDED 38

// Define Grbl alarm codes. Valid values (1-255). 0 is reserved.
#define ALARM_HARD_LIMIT_ERROR      EXEC_ALARM_HARD_LIMIT
#define ALARM_SOFT_LIMIT_ERROR      EXEC_ALARM_SOFT_LIMIT
#define ALARM_ABORT_CYCLE           EXEC_ALARM_ABORT_CYCLE
#define ALARM_PROBE_FAIL_INITIAL    EXEC_ALARM_PROBE_FAIL_INITIAL
#define ALARM_PROBE_FAIL_CONTACT    EXEC_ALARM_PROBE_FAIL_CONTACT
#define ALARM_HOMING_FAIL_RESET     EXEC_ALARM_HOMING_FAIL_RESET
#define ALARM_HOMING_FAIL_DOOR      EXEC_ALARM_HOMING_FAIL_DOOR
#define ALARM_HOMING_FAIL_PULLOFF   EXEC_ALARM_HOMING_FAIL_PULLOFF
#define ALARM_HOMING_FAIL_APPROACH  EXEC_ALARM_HOMING_FAIL_APPROACH

// Define Grbl feedback message codes. Valid values (0-255).
#define MESSAGE_CRITICAL_EVENT 1
#define MESSAGE_ALARM_LOCK 2
#define MESSAGE_ALARM_UNLOCK 3
#define MESSAGE_ENABLED 4
#define MESSAGE_DISABLED 5
#define MESSAGE_SAFETY_DOOR_AJAR 6
#define MESSAGE_CHECK_LIMITS 7
#define MESSAGE_PROGRAM_END 8
#define MESSAGE_RESTORE_DEFAULTS 9
#define MESSAGE_SPINDLE_RESTORE 10
#define MESSAGE_SLEEP_MODE 11
