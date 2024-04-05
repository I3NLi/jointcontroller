/*!
 * Define the robot version how is running here.
 * The differences between the robots are the kind of links used and how they
 * are set on the link-controller and the distribution of the angle sensors
 * on the the two SPI interfaces. This differences will lead to different
 * controller setups but the setup of axes, links and joints will be still
 * the same, so the same model can be used.
 */

#include <SD.h>
#include <tlx5012-arduino.hpp>
#include <jointController.hpp>
#include "const.h"

using namespace tle5012;

#define LINE_FLAG_OVERFLOW                  bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES       bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON         bit(2)

tle5012::SPIClass3W tle5012::SPI3W1(1);         //!< SPI port 1 on XMC4700 X1 according HW SPI setup
tle5012::SPIClass3W tle5012::SPI3W2(2);         //!< SPI port 2 on XMC4700 X2 according HW SPI setup

static char line[LINE_BUFFER_SIZE];             //!< Line to be executed. Zero-terminated.
static char prog[LINE_BUFFER_SIZE];             //!< program to be executed. Zero-terminated.

boolean isCalibrate   = false;                  //!< Is the robot homing position calibrated?
boolean isRun         = false;                  //!< Is the robot running?
boolean isPosSpeed    = true;                   //!< if set true we use postion/speed PID with two ISR routines, otherwise we use only a single ISR with normal PID
boolean isLogging     = true;                  //!< if true than logging is switched on

uint8_t line_flags    = 0;
uint8_t char_counter  = 0;
uint8_t c;

/*!
 * Global controller array holds pointers for each joint. Each joint is a combination of
 * a TLx5012 magnetic angle sensor and an IFX007T BLDC shield. The TLx5012 communicates via SPI
 * an is initialized with the init function of the jointController. The IF007T shields uses PWM and
 * GPIO pins which where set with the init function
 */
#define jointTotalNum  6                                        //!> total number of controlled joints
jointController link[jointTotalNum] = {                         //!> joint controller array
    ( jointController( (char*)"X", isLogging ) ),
    ( jointController( (char*)"Y", isLogging ) ),
    ( jointController( (char*)"Z", isLogging ) ),
    ( jointController( (char*)"A", isLogging ) ),
    ( jointController( (char*)"B", isLogging ) ),                     //!> refer as CR
    ( jointController( (char*)"C", isLogging ) )                      //!> refer as CL
};

/**
 * @brief array with the intended angles for each joint
 * This is the result of the G-Code parser for each of the
 * six joint angles
 */
volatile double intent_angle[jointTotalNum] = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
};


// ****************************************************************************
// code functions which needs to be placed before setup and loop
// ****************************************************************************


/**
 * @brief external C functions for fetching interrupt handler
 * 
 */
extern "C"
{

    /**
     * @brief Position PID controller calculates the speed of each joint
     * and will be called slowly
     */
    void CCU42_0_IRQHandler(void) //Position control
    { 

        digitalWrite(LED2, HIGH);
        for (int8_t i=0; i<jointTotalNum; i++){
            link[i].runToAnglePOS();
        }
        digitalWrite(LED2, LOW);
    }

    /**
     * @brief Speed PID controller calculates the position and speed
     * against the actual angle. This will be called fast.
     */
    void CCU40_0_IRQHandler(void) //Speed control
    { 
        digitalWrite(LED1, HIGH);
        for (int8_t i=0; i<jointTotalNum; i++){
            link[i].runToAngleSpeed();
        }
        digitalWrite(LED1, LOW);

        while (Serial.available() != 0)
        {
            uint8_t c = Serial.read();
            if ((c == '\n') || (c == '\r'))  // End of line reached
            {

                // -> TODO main read
                // protocol_execute_realtime(); // Runtime command check point.

                line[char_counter] = 0; // Set string termination character.

                // Direct and execute one line of formatted input, and report status of execution.
                if (line_flags & LINE_FLAG_OVERFLOW) {
                    // Report line overflow error.
                    // report_status_message(STATUS_OVERFLOW);
                    Serial.print("Status overflow: ");
                    Serial.println(STATUS_OVERFLOW);
                } else if (line[0] == 0) {
                    // Empty or comment line. For syncing purposes.
                    // report_status_message(STATUS_OK);
                    Serial.print("Status: ");
                    Serial.println(STATUS_OK);
                } else if (line[0] == '$') {
                    Serial.println((char*)line);
                    // Grbl '$' system command
                    simpleSystemParser(line);
                }else{
                    Serial.println((char*)line);
                    simpleGCodeParser(line);
                    for (int8_t i=0; i<jointTotalNum; i++){
                        Serial.println(intent_angle[i]);
                        while(link[i].checkStatus()!=NONE){};
                        link[i].moveTo(intent_angle[i]);
                    }
                }

                // Reset tracking data for next line.
                line_flags = 0;
                char_counter = 0;

            } else {

                if (line_flags) {
                    // Throw away all (except EOL) comment characters and overflow characters.
                    if (c == ')') {
                        // End of '()' comment. Resume line allowed.
                        if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
                    }
                } else {
                    if (c <= ' ') {
                        // Throw away whitespace and control characters
                    } else if (c == '/') {
                        // Block delete NOT SUPPORTED. Ignore character.
                        // NOTE: If supported, would simply need to check the system if block delete is enabled.
                    } else if (c == '(') {
                        // Enable comments flag and ignore all characters until ')' or EOL.
                        // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
                        // In the future, we could simply remove the items within the comments, but retain the
                        // comment control characters, so that the g-code parser can error-check it.
                        line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
                    } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
                        // Detect line buffer overflow and set flag.
                        line_flags |= LINE_FLAG_OVERFLOW;
                    } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
                        line[char_counter++] = c-'a'+'A';
                    } else {
                        line[char_counter++] = c;
                    }
                }

            }
        }
    }
}


/**
 * @brief 
 * 
 */
void timerSetupSingle(int16_t prescaler1)
{
    //Setup Interrupt settings 1
    XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = {0};
    pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
    pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;

    XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
    XMC_CCU4_EnableClock(CCU40, 3);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, prescaler1); // Adjust last Value or Prescaler
    /* Enable compare match and period match events */
    XMC_CCU4_SLICE_EnableEvent(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
    /* Connect period match event to SR0 */
    XMC_CCU4_SLICE_SetInterruptNode(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0);
    /* Configure NVIC */
    /* Set priority */
    NVIC_SetPriority(CCU40_0_IRQn, 5);
    /* Enable IRQ */
    NVIC_EnableIRQ(CCU40_0_IRQn); 
    XMC_CCU4_EnableShadowTransfer(CCU40, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
    XMC_CCU4_SLICE_StartTimer(CCU40_CC43);
}


/**
 * @brief Double timer setup if position and speed PID are used.
 * 
 */
void timerSetupDouble(int16_t prescaler1, int16_t prescaler2)
{
    //Setup Interrupt settings
    XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = {0};
    pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
    pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;

    // interrupt 1
    XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
    XMC_CCU4_EnableClock(CCU40, 3);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, prescaler1); // Adjust last Value or Prescaler
    /* Enable compare match and period match events */
    XMC_CCU4_SLICE_EnableEvent(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
    /* Connect period match event to SR0 */
    XMC_CCU4_SLICE_SetInterruptNode(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0);
    /* Configure NVIC */
    /* Set priority */
    NVIC_SetPriority(CCU40_0_IRQn, 10);
    /* Enable IRQ */
    NVIC_EnableIRQ(CCU40_0_IRQn); 
    XMC_CCU4_EnableShadowTransfer(CCU40, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
    XMC_CCU4_SLICE_StartTimer(CCU40_CC43);

    // interrupt 2
    XMC_CCU4_Init(CCU42, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SLICE_CompareInit(CCU42_CC43, &pwm_config);
    XMC_CCU4_EnableClock(CCU42, 3);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU42_CC43, prescaler2); // Adjust last Value or Prescaler
    /* Enable compare match and period match events */
    XMC_CCU4_SLICE_EnableEvent(CCU42_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
    /* Connect period match event to SR0 */
    XMC_CCU4_SLICE_SetInterruptNode(CCU42_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0); 
    /* Configure NVIC */
    /* Set priority */
    NVIC_SetPriority(CCU42_0_IRQn, 5);
    /* Enable IRQ */
    NVIC_EnableIRQ(CCU42_0_IRQn); 
    XMC_CCU4_EnableShadowTransfer(CCU42, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
    XMC_CCU4_SLICE_StartTimer(CCU42_CC43);
}


/**
 * @brief Blinks the given LED with the given end state
 *
 * @param led integer of the LED which to blink
 * @param state boolean end state of the LED, either HIGH or LOW
 */
void blink(int8_t led, boolean state)
{
    for (int8_t i=0;i<5;i++)
    {
        digitalWrite(led, !state);
        delay(10);
        digitalWrite(led, state);
    }
    delay(50);
}


/**
 * @brief Initialized all joints with sensor and shield, set basic values for the gear factor, 
 * the range limits and startups the jointController
 * 
 */
void jointInit()
{
    // link[0] = X axis = base; SPI1,CS1,Slave0
    link[0].initSensor(SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
    link[0].initShield(PIN_PWM_U1,PIN_PWM_V1,PIN_PWM_W1,PIN_PWM_EN1,PIN_PWM_EN1,PIN_PWM_EN1);
    link[0].setRangeLimits(minLimit_X,maxLimit_X,defaultPos_X);
    link[0].setGearFactor(gearFactor_X);
    link[0].begin();

    // link[1] = Y axis, SPI1, CS1,Slave1
    link[1].initSensor(SPI3W1, PIN_SPI1_SS1, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S1);
    link[1].initShield(PIN_PWM_U2,PIN_PWM_V2,PIN_PWM_W2,PIN_PWM_EN2,PIN_PWM_EN2,PIN_PWM_EN2);
    link[1].setRangeLimits(minLimit_Y,maxLimit_Y,defaultPos_Y);
    link[1].setGearFactor(gearFactor_Y);
    link[1].begin();

    // // link[2] = Z axis, SPI2, CS1,Slave0
    link[2].initSensor(SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
    link[2].initShield(PIN_PWM_U3,PIN_PWM_V3,PIN_PWM_W3,PIN_PWM_EN3,PIN_PWM_EN3,PIN_PWM_EN3);
    link[2].setRangeLimits(minLimit_Z,maxLimit_Z,defaultPos_Z);
    link[2].setGearFactor(gearFactor_Z);
    link[2].begin();

    // link[3] = A axis, SPI2, CS2,Slave1
    link[3].initSensor(SPI3W2, PIN_SPI2_SS1, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S1);
    link[3].initShield(PIN_PWM_U4,PIN_PWM_V4,PIN_PWM_W4,PIN_PWM_EN4,PIN_PWM_EN4,PIN_PWM_EN4);
    link[3].setRangeLimits(minLimit_A,maxLimit_A,defaultPos_A);
    link[3].setGearFactor(gearFactor_A);
    link[3].begin();

    // link[4] = B axis, SPI2, CS2,Slave2
    link[4].initSensor(SPI3W2, PIN_SPI2_SS2, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S2);
    link[4].initShield(PIN_PWM_U5,PIN_PWM_V5,PIN_PWM_W5,PIN_PWM_EN5,PIN_PWM_EN5,PIN_PWM_EN5);
    link[4].setRangeLimits(minLimit_CR,maxLimit_CR,defaultPos_CR);
    link[4].setGearFactor(gearFactor_CR);
    link[4].begin();

    // link[5] = C axis, SPI2, CS3,Slave3
    link[5].initSensor(SPI3W2, PIN_SPI2_SS3, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S3);
    link[5].initShield(PIN_PWM_U6,PIN_PWM_V6,PIN_PWM_W6,PIN_PWM_EN6,PIN_PWM_EN6,PIN_PWM_EN6);
    link[5].setRangeLimits(minLimit_CL,maxLimit_CL,defaultPos_CL);
    link[5].setGearFactor(gearFactor_CL);
    link[5].begin();

    return;
}


/**
 * @brief checks status of XMC4700 Button 1
 * The button 1 starts and stops the robot in normal communication mode
 *
 */
 void checkButton1(void)
 {
    if (!isRun)
    {
        blink(LED1,HIGH);
        if (isPosSpeed)
        {
            timerSetupDouble(500,15000);
            Serial.println("Timer double interrupt for postion and speed added");
        }else{
            timerSetupSingle(200);
            Serial.println("Timer single interrupt added");
        }
        isRun = true;
        digitalWrite(LED1, LOW);

    }else{
        for (int8_t i=0; i<jointTotalNum; i++){
            link[i].switchShieldOnOff(LOW);

            Serial.print("Switch off all shields: ");
            Serial.println(link[i].jointName);

        }
        isRun = false;
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
    }

 }


/**
 * @brief checks status of XMC4700 Button 2
 * calibrates the offset of each joint.
 * This is needed for soft homing.
 */
void checkButton2(void)
{
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    if (!isCalibrate) {

        for (int8_t i=0; i<jointTotalNum; i++){
            double home = link[i].setHomingPosition(0.0);

            Serial.print("Homing position set for joint: ");
            Serial.print(link[i].jointName);
            Serial.print(" = ");
            Serial.println(home);

            blink(LED1,HIGH);
            blink(LED2,HIGH);
        }
        isCalibrate = true;
        delay(1000);
        Serial.println("All Homing positions set");

    }
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
}


/**
 * @brief Set new angle positions.
 * This functions works async to the PID controllers
 * and inserts a new angle whenever a new angle is available
 */
void moveTo()
{
    for (int8_t i=0; i<jointTotalNum; i++){
        link[i].moveTo(intent_angle[0]);
    }
}


// ****************************************************************************
// arduino main setup and loop
// ****************************************************************************

/**
 * @brief Arduino setup function
 * 
 */
void setup()
{
    // Serial port communication
    Serial.begin(115200);
    while (!Serial) {}
    delay(5000);
    Serial.flush();
    Serial.println("\nARCTOS 2 XMC begin setup\n");

    // set LED pin to output, used to blink when writing
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);

    // Setup jointController for each joint
    jointInit();
    Serial.println("All joints are ready");
    delay(2000);

    // // Setup first timer
    // if(isPosSpeed)
    // {
    //     timerSetupDouble(500,1000);
    //     Serial.println("Timer double interrupt for postion and speed added");
    // }else{
    //     timerSetupSingle(200);
    //     Serial.println("Timer single interrupt added");
    // }

    // setup done
    Serial.println("\nARCTOS 2 XMC ready for calibration\n");

}


/**
 * @brief 
 * 
 */
void loop()
{
    // Calibrate Button pressed
    if (!isCalibrate)
    {
        if (digitalRead(BUTTON2) == LOW)
        {
            checkButton2();
        }
    }

    // Start run
    if (isCalibrate && !isRun)
    {
        if (digitalRead(BUTTON1) == LOW)
        {
            checkButton1();
        }
    }

    // Run G-code
    if (isCalibrate && isRun)
    {
    }

}


/**
 * @brief 
 * Extracts a floating point value from a string. The following code is based loosely on
 * the avr-libc strtod() function by Michael Stumpf and Dmitry Xmelkov and many freely
 * available conversion method examples, but has been highly optimized for Grbl. For known
 * CNC applications, the typical decimal value is expected to be in the range of E0 to E-4.
 * Scientific notation is officially not supported by g-code, and the 'E' character may
 * be a g-code word on some CNC systems. So, 'E' notation will not be recognized.
 * NOTE: Thanks to Radu-Eosif Mihailescu for identifying the issues with using strtod().
 * 
 */
uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)
{
    char *ptr = line + *char_counter;
    unsigned char c;

    // Grab first character and increment pointer. No spaces assumed in line.
    c = *ptr++;

    // Capture initial positive/minus character
    bool isnegative = false;
    if (c == '-') {
        isnegative = true;
        c = *ptr++;
    } else if (c == '+') {
        c = *ptr++;
    }

    // Extract number into fast integer. Track decimal in terms of exponent value.
    uint32_t intval = 0;
    int8_t exp = 0;
    uint8_t ndigit = 0;
    bool isdecimal = false;
    while(1) {
        c -= '0';
        if (c <= 9) {
            ndigit++;
            if (ndigit <= MAX_INT_DIGITS) {
                if (isdecimal) { exp--; }
                intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
            } else {
                if (!(isdecimal)) { exp++; }  // Drop overflow digits
            }
            } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
                isdecimal = true;
            } else {
                break;
            }
            c = *ptr++;
        }

    // Return if no digits have been read.
    if (!ndigit) { return(false); };

    // Convert integer into floating point.
    float fval;
    fval = (float)intval;

    // Apply decimal. Should perform no more than two floating point multiplications for the
    // expected range of E0 to E-4.
    if (fval != 0) {
        while (exp <= -2) {
            fval *= 0.01;
            exp += 2;
        }
        if (exp < 0) {
            fval *= 0.1;
        } else if (exp > 0) {
            do {
                fval *= 10.0;
            } while (--exp > 0);
        }
  }

    // Assign floating point value with correct sign.
    if (isnegative) {
        *float_ptr = -fval;
    } else {
        *float_ptr = fval;
    }

    *char_counter = ptr - line - 1; // Set char_counter to next statement

    return(true);
}


/**
 * @brief Simple G-Code Parser parser gcode lines
 * only for G90 and A/B/C and X/Y/Z axes and M97 for
 * gripper.
 * 
 * @param line 
 */
void simpleGCodeParser(char *line)
{

    uint8_t char_counter    = 0;
    char letter;
    float value;
    uint8_t int_value       = 0;
    uint16_t mantissa       = 0;

    while (line[char_counter] != 0) { // Loop until no more g-code words in line.

        // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.
        letter = line[char_counter];
        if((letter < 'A') || (letter > 'Z')) { 
            Serial.print("Expected command letter");
            Serial.println(STATUS_EXPECTED_COMMAND_LETTER);
        }
        char_counter++;
        if (!read_float(line, &char_counter, &value)) { 
            Serial.print("Bad number format");
            Serial.println(STATUS_BAD_NUMBER_FORMAT);
        }

        int_value = trunc(value);
        mantissa =  round(100*(value - int_value)); // Compute mantissa for Gxx.x commands.

        switch(letter) {
            case 'G':
                Serial.println("G-Code");
                switch(int_value) {
                    case 10: case 28: case 30: case 92:
                        Serial.println("G10/G28/G30/G92 not implemented");
                    case 4: case 53:
                        Serial.println("G4/G53 not implemented");
                        break;
                    case 0: case 1: case 2: case 3: case 38:
                        Serial.println("G0/G1/G2/G3/G38 not implemented");
                    case 80:
                        Serial.println("G80 not implemented");
                        break;
                    case 17: case 18: case 19:
                        Serial.println("G17/G18/G19 not implemented");
                        break;
                    case 90: case 91:
                        // if (mantissa == 0) {
                        //     dword_bit = MODAL_GROUP_G3;
                        //     gc_block.modal.distance = int_value - 90;
                        // } else {
                        //     dword_bit = MODAL_GROUP_G4;
                        //     if ((mantissa != 10) || (int_value == 90)) { FAIL(STATUS_GCODE_UNSUPPORTED_COMMAND); } // [G90.1 not supported]
                        //     mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        //     // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
                        // }
                        break;
                    case 93: case 94:
                        Serial.println("G93/G94 not implemented");
                        break;
                    case 20: case 21:
                        Serial.println("G20/G21 not implemented");
                        break;
                    case 40:
                        Serial.println("G40 not implemented");
                        break;
                    case 43: case 49:
                        Serial.println("G43/G49 not implemented");
                        break;
                    case 54: case 55: case 56: case 57: case 58: case 59:
                        Serial.println("G54/G55/G56/G57/G58/G59 not implemented");
                        break;
                    case 61:
                        Serial.println("G61 not implemented");
                        break;
                default: 
                    Serial.print("Unsupported command letter");
                    Serial.println(STATUS_GCODE_UNSUPPORTED_COMMAND);
                }
                break;

            case 'M':
                switch(int_value) {
                    case 0: case 1: case 2: case 30:
                        Serial.println("M0/M1/M2/M30 not implemented");
                        break;
                    case 3: case 4: case 5:
                        Serial.println("M3/M4/M5 not implemented");
                        break;
                    case 7: case 8: case 9:
                        Serial.println("M7/M8/M9 not implemented");
                        break;
                    case 96: case 97: case 98:
                        Serial.println("Servo M96/M97/M98 not implemented");
                        // servo.cmdParse=int_value;
                        // for(uint8_t i=0; i<numberOfServos; i++){
                        //     servo.parseInuse[i]=0;
                        // }
                        break;
                    default: 
                        Serial.print("Unsupported command letter");
                        Serial.println(STATUS_GCODE_UNSUPPORTED_COMMAND);
                }
            break;

            default:
                switch(letter){
                    case 'A':
                        intent_angle[3] = value;
                    break;
                    case 'B':
                        intent_angle[4] = value;
                    break;
                    case 'C':
                        intent_angle[5] = value;
                    break;
                    case 'X':
                        intent_angle[0] = value;
                    break;
                    case 'Y':
                        intent_angle[1] = value;
                    break;
                    case 'Z':
                        intent_angle[2] = value;
                    break;
                }


        }

    }

}


/**
 * @brief Construct a new simple System Parser object
 * 
 * @param line 
 */
void simpleSystemParser(char *line)
{
    uint8_t char_counter = 1;
    uint8_t helper_var = 0; // Helper variable
    float parameter, value;

    switch( line[char_counter] ) {
        case 0 : 
            Serial.println("grbl help report not implemented");
            break;
        case 'J' : // Jogging
            Serial.println("Jogging");
            break;
        case '$': case 'G': case 'C': case 'S': case 'X':
            if ( line[2] != 0 ) { 
                Serial.print("Invalid statement");
                Serial.println(STATUS_INVALID_STATEMENT);
             }
            switch( line[1] ) {
                case '$' : // Prints Grbl settings
                    Serial.println("grbl settings report not implemented");
                    break;
                case 'G' : // Prints gcode parser state
                    Serial.println("gcode parser status not implemented");
                    break;
                case 'C' : // Set check g-code mode [IDLE/CHECK]
                    Serial.println("Reset, shield PWM are switched on");
                    for (int8_t i=0; i<jointTotalNum; i++){
                        link[i].switchShieldOnOff(HIGH);
                    }
                    break;
                case 'S' : // Immediate stop
                    Serial.println("Stop Stop Stop, all shield PWM switched off");
                    for (int8_t i=0; i<jointTotalNum; i++){
                        link[i].switchShieldOnOff(LOW);
                    }
                    break;
                case 'X' : // Disable alarm lock [ALARM]
                    Serial.println("Disable alarm lock not implemented");
                    break;
            }
            break;

        default:
            switch( line[1] ) {
                case '#' : // Print Grbl NGC parameters
                    Serial.println("NGS parameter not implemented");
                    break;
                case 'H' : // Perform homing cycle [IDLE/ALARM]
                    if (line[2] == 0) {
                        Serial.println("Homing all joints");
                        for (int8_t i=0; i<jointTotalNum; i++){
                            link[i].homing();
                        }
                    } else if (line[3] == 0) {
                       switch (line[2]) {
                            case 'X': 
                                Serial.println("Homing joint X");
                                link[0].homing();
                                break;
                            case 'Y':
                                Serial.println("Homing joint Y");
                                link[1].homing();
                                break;
                            case 'Z':
                                Serial.println("Homing joint Z");
                                link[2].homing();
                                break;
                            case 'A':
                                Serial.println("Homing joint A");
                                link[3].homing();
                                break;
                            case 'B':
                                Serial.println("Homing joint B");
                                link[4].homing();
                                break;
                            case 'C':
                                Serial.println("Homing joint C");
                                link[5].homing();
                                break;
                            default:
                                Serial.println("Invalid Statement");
                       }
                    }
                    break;
                case 'P' : // runs gcode programs from SD card
                    if (line[2] == 0) {
                        Serial.println("Run default program");
                        runProgram("default");
                    } else if (line[3] == 0) {
                       switch (line[2]) {
                            case '1': 
                                runProgram("prog1.gcode");
                                break;
                            case '2': 
                                runProgram("prog2.gcode");
                                break;
                            case '3': 
                                runProgram("prog3.gcode");
                                break;
                            case '4': 
                                runProgram("prog4.gcode");
                                break;
                            case '5': 
                                runProgram("prog5.gcode");
                                break;
                            case '6': 
                                runProgram("prog6.gcode");
                                break;
                            case '7': 
                                runProgram("prog7.gcode");
                                break;
                            case '8': 
                                runProgram("prog8.gcode");
                                break;
                            case '9': 
                                runProgram("prog9.gcode");
                                break;
                           }
                    }
                    break;
                case 'S' : // Puts Grbl to sleep [IDLE/ALARM]
                    Serial.println("Sleep mode not implemented");
                    break;
                case 'I' : // Print or store build info. [IDLE/ALARM]
                    Serial.println("Store build info not implemented");
                    break;
                case 'R' : // Restore defaults [IDLE/ALARM]
                    Serial.println("Stop interrupt timer");
                    isRun = false;
                    break;
                case 'N' : // Startup lines. [IDLE/ALARM]
                    Serial.println("Soft reset");
                    isRun = false;
                    isCalibrate = false;
                    break;

            }

    }

}

/**
 * @brief Function read a new gcode program from SD card
 * and runs ot
 * 
 * @param filename 
 */
void runProgram(String filename)
{
    if (!SD.begin()) {
        Serial.println("Card failed, or not present");
        return;
    }else{
        Serial.print("Load program ");
        Serial.println(filename);
        uint8_t prog_counter  = 0;
        // try to open the file for writing
        File txtFile = SD.open(filename);

        if (txtFile) {
            while (txtFile.available()) {
                uint8_t p = txtFile.read();
                if ((p == '\n') || (p == '\r'))  // End of line reached
                {
                    prog[prog_counter] = 0; // Set string termination character.
                    Serial.println((char*)prog);
                    simpleGCodeParser(prog);
                    for (int8_t i=0; i<jointTotalNum; i++){
                        Serial.println(intent_angle[i]);
                        link[i].moveTo(intent_angle[i]);
                    }
                    // Reset tracking data for next line.
                    prog_counter = 0;
                } else {
                    if (p <= ' ') {
                        // Throw away whitespace and control characters
                    } else if (p >= 'a' && p <= 'z') { // Upcase lowercase
                        prog[prog_counter++] = p-'a'+'A';
                    } else {
                        prog[prog_counter++] = p;
                    }
                }
            }

            txtFile.close();
        
        }else{
            Serial.print("error opening ");
            Serial.println(filename);
        }
    }
}