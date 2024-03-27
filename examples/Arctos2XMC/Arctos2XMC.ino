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

tle5012::SPIClass3W tle5012::SPI3W1(1);         //!< SPI port 1 on XMC4700 X1 according HW SPI setup
tle5012::SPIClass3W tle5012::SPI3W2(2);         //!< SPI port 2 on XMC4700 X2 according HW SPI setup

boolean  isCalibrate   = false;

/*!
 * Global controller array holds pointers for each joint. Each joint is a combination of
 * a TLx5012 magnetic angle sensor and an IFX007T BLDC shield. The TLx5012 communicates via SPI
 * an is initialized with the init function of the jointController. The IF007T shields uses PWM and
 * GPIO pins which where set with the init function
 */
#define jointTotalNum  6                                     //!> total number of controlled joints
jointController link[jointTotalNum] = {                      //!> joint controller array
   ( jointController( (char*)"X" ) ),
   ( jointController( (char*)"Y" ) ),
   ( jointController( (char*)"Z" ) ),
   ( jointController( (char*)"A" ) ),
   ( jointController( (char*)"B" ) ),
   ( jointController( (char*)"C" ) )
};

/**
 * @brief array with the intended angles for each joint
 * 
 */
volatile double intent_angle[jointTotalNum] = {
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
};



/**
 * @brief timer interrupt prototype
 * 
 */
void timerISR1(void);


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
    link[0].initShield(PIN_PWM_U1,PIN_PWM_V1,PIN_PWM_W1,PIN_PWM_EN_U1,PIN_PWM_EN_V1,PIN_PWM_EN_W1);
    link[0].setRangeLimits(minLimit_X,maxLimit_X,defaultPos_X);
    link[0].setGearFactor(gearFactor_X);
    link[0].begin();

    // link[1] = Y axis, SPI1, CS1,Slave1
    link[1].initSensor(SPI3W1, PIN_SPI1_SS1, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S1);
    link[1].initShield(PIN_PWM_U2,PIN_PWM_V2,PIN_PWM_W2,PIN_PWM_EN_U2,PIN_PWM_EN_V2,PIN_PWM_EN_W2);
    link[1].setRangeLimits(minLimit_Y,maxLimit_Y,defaultPos_Y);
    link[1].setGearFactor(gearFactor_Y);
    link[1].begin();

    // link[2] = Z axis, SPI2, CS1,Slave0, index=3, Z axis
    link[2].initSensor(SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
    link[2].initShield(PIN_PWM_U3,PIN_PWM_V3,PIN_PWM_W3,PIN_PWM_EN_U3,PIN_PWM_EN_V3,PIN_PWM_EN_W3);
    link[2].setRangeLimits(minLimit_Z,maxLimit_Z,defaultPos_Z);
    link[2].setGearFactor(gearFactor_Z);
    link[2].begin();

    // link[3] = A axis, SPI2, CS2,Slave1, index=4, A axis
    link[3].initSensor(SPI3W2, PIN_SPI2_SS1, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S1);
    link[3].initShield(PIN_PWM_U4,PIN_PWM_V4,PIN_PWM_W4,PIN_PWM_EN_U4,PIN_PWM_EN_V4,PIN_PWM_EN_W4);
    link[3].setRangeLimits(minLimit_A,maxLimit_A,defaultPos_A);
    link[3].setGearFactor(gearFactor_A);
    link[3].begin();

    // link[4] = B axis, SPI2, CS2,Slave2, index=5, B axis
    link[4].initSensor(SPI3W2, PIN_SPI2_SS2, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S2);
    link[4].initShield(PIN_PWM_U5,PIN_PWM_V5,PIN_PWM_W5,PIN_PWM_EN_U5,PIN_PWM_EN_V5,PIN_PWM_EN_W5);
    link[4].setRangeLimits(minLimit_CR,maxLimit_CR,defaultPos_CR);
    link[4].setGearFactor(gearFactor_CR);
    link[4].begin();

    // link[5] = C axis, SPI2, CS3,Slave3, index=6, C axis
    link[5].initSensor(SPI3W2, PIN_SPI2_SS3, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S3);
    link[5].initShield(PIN_PWM_U6,PIN_PWM_V6,PIN_PWM_W6,PIN_PWM_EN_U6,PIN_PWM_EN_V6,PIN_PWM_EN_W6);
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
 }

/**
 * @brief checks status of XMC4700 Button 2
 * calibrates the offset of each joint.
 * THis is needed for soft homing
 */
void checkButton2(void)
{
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
    if (!isCalibrate) {

        for (int8_t i=0; i<jointTotalNum; i++){
            link[i].setHomingPosition(0.0);

            Serial.print("Homing position set for joint: ");
            Serial.println(link[i].jointName);
            blink(LED1,HIGH);
            blink(LED2,HIGH);
        }

    }
    isCalibrate = true;

//   digitalWrite(LED1, HIGH);
//   digitalWrite(LED2, HIGH);

//   if (!isOffset)
//   {
//     axesCalibrate();
//     isOffset = true;
//     Serial.println("Offset calibrated");
//     blink(LED1,HIGH);
//     blink(LED2,HIGH);
//     return;
//   }
    


}



/**
 * @brief Arduino setup function
 * 
 */
void setup()
{
    // Serial port communication
    Serial.begin(115200);
    while (!Serial) {}

    // set LED pin to output, used to blink when writing
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);

    // Setup jointController for each joint
    jointInit();
    Serial.println("all joints are ready");
    delay(5000);

    // Timer Interrupt settings
    uint8_t idTimer = addTask( *timerISR1 );
    uint8_t okTimer = setInterval( idTimer, 10 );
    if (okTimer > 0){
        startTask(idTimer);
        Serial.println("\nTimer interrupt started"); 
    }

    // // Setup Interrupt settings
    // XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = {0};
    // pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
    // pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;
    // XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    // XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
    // XMC_CCU4_EnableClock(CCU40, 3);
    // XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, 200); // Adjust last Value or Prescaler
    // /* Enable compare match and period match events */
    // XMC_CCU4_SLICE_EnableEvent(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
    // /* Connect period match event to SR0 */
    // XMC_CCU4_SLICE_SetInterruptNode(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0);
    // /* Configure NVIC */
    // /* Set priority */
    // NVIC_SetPriority(CCU40_0_IRQn, 10);
    // /* Enable IRQ */
    // NVIC_EnableIRQ(CCU40_0_IRQn); 
    // XMC_CCU4_EnableShadowTransfer(CCU40, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
    // XMC_CCU4_SLICE_StartTimer(CCU40_CC43);

    Serial.println("timer interrupt sequence started");
    delay(5000);

}


/**
 * @brief 
 * 
 */
void loop()
{

    if (digitalRead(BUTTON2) == LOW) {
        checkButton2();
        delay(100);
    }else{
        digitalWrite(LED2, LOW);
    }
}

/**
 * @brief set joint end positions for all joints
 * 
 */
void jointPosition()
{
    for (int8_t i=0; i<jointTotalNum; i++){
        link[i].runToAngle(intent_angle[i]);
    }
}


/**
 * @brief Timer callback function runs joints to position
 * 
 */
void timerISR1(void){
    jointPosition();
}





