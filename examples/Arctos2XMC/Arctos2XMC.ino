/*!
 * Define the robot version how is running here.
 * The differences between the robots are the kind of links used and how they
 * are set on the link-controller and the distribution of the angle sensors
 * on the the two SPI interfaces. This differences will lead to different
 * controller setups but the setup of axes, links and joints will be still
 * the same, so the same model can be used.
 */

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
   ( jointController( (char*)"X", !isLogging ) ),
   ( jointController( (char*)"Y", !isLogging ) ),
   ( jointController( (char*)"Z", !isLogging ) ),
   ( jointController( (char*)"A", !isLogging ) ),
   ( jointController( (char*)"B", !isLogging ) ),                     //!> refer as CR
   ( jointController( (char*)"C", !isLogging ) )                      //!> refer as CL
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
            Serial.println(c);
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
                    // Grbl '$' system command
                    Serial.println("$ command");
                    Serial.println((char*)line);
                    // -> TODO $ commands
                    //report_status_message(system_execute_line(line));
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
        moveTo();
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
    NVIC_SetPriority(CCU40_0_IRQn, 5);
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
    NVIC_SetPriority(CCU42_0_IRQn, 10);
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

    // link[2] = Z axis, SPI2, CS1,Slave0
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
            timerSetupDouble(1000,2000);
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
    delay(2000);
    Serial.println("\nARCTOS 2 XMC ready\n");

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


