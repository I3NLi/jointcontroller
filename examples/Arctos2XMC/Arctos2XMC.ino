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

errorTypes sensorError = NO_ERROR;
errorTypes shieldError = NO_ERROR;

/*!
 * Global controller array holds pointers for each joint. Each joint is a combination of
 * a TLx5012 magnetic angle sensor and an IFX007T BLDC shield. The TLx5012 communicates via SPI
 * an is initialized with the init function of the jointController. The IF007T shields uses PWM and
 * GPIO pins which where set with the init function
 */
#define jointTotalNum  6                                     //!> total number of controlled joints
jointController link[jointTotalNum] = {                      //!> joint controller array
   ( jointController( (char*)"1" ) ),
   ( jointController( (char*)"2" ) ),
   ( jointController( (char*)"3" ) ),
   ( jointController( (char*)"4" ) ),
   ( jointController( (char*)"5" ) ),
   ( jointController( (char*)"6" ) )
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
 * @brief Interrupt function calls all joints in a loop with there intended position
 * 
 */
extern "C" {
    void CCU40_0_IRQHandler(void)
    { 
        for (int8_t i=0; i<jointTotalNum; i++){
            link[i].runToAngle(intent_angle[i]);
        }
    }
}


/**
 * @brief Initialized all joints with sensor and shield, set basic values for the gear factor, 
 * the range limits and startups the jointController
 * 
 */
void jointInit()
{
    // link[0] = X axis = base; SPI1,CS1,Slave0,index=1
    sensorError = link[0].initSensor(SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
    shieldError = link[0].initShield(PIN_PWM_U1,PIN_PWM_V1,PIN_PWM_W1,PIN_PWM_EN_U1,PIN_PWM_EN_V1,PIN_PWM_EN_W1);
    link[0].setGearFactor(gearFactorX);
    link[0].begin();

    Serial.print("Joint 0 Sensor: ");  Serial.println(sensorError,HEX);
    Serial.print("Joint 0 shield: ");  Serial.println(shieldError,HEX);


    // link[1] = Y axis, SPI1, CS1,Slave1, index=2, Y axis
    sensorError = link[1].initSensor(SPI3W1, PIN_SPI1_SS1, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S1);
    shieldError = link[1].initShield(PIN_PWM_U2,PIN_PWM_V2,PIN_PWM_W2,PIN_PWM_EN_U2,PIN_PWM_EN_V2,PIN_PWM_EN_W2);
    link[1].setGearFactor(gearFactorY);
    link[1].begin();

    Serial.print("Joint 1 Sensor: ");  Serial.println(sensorError,HEX);
    Serial.print("Joint 1 shield: ");  Serial.println(shieldError,HEX);


    // link[2] = Z axis, SPI1, CS2,Slave2, index=3, Z axis
    sensorError = link[2].initSensor(SPI3W1, PIN_SPI1_SS2, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S2);
    shieldError = link[2].initShield(PIN_PWM_U3,PIN_PWM_V3,PIN_PWM_W3,PIN_PWM_EN_U3,PIN_PWM_EN_V3,PIN_PWM_EN_W3);
    link[2].setGearFactor(gearFactorZ);
    link[2].begin();

    Serial.print("Joint 2 Sensor: ");  Serial.println(sensorError,HEX);
    Serial.print("Joint 2 shield: ");  Serial.println(shieldError,HEX);


    // link[3] = A axis, SPI1, CS3,Slave3, index=4, A axis
    sensorError = link[3].initSensor(SPI3W1, PIN_SPI1_SS3, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S3);
    shieldError = link[3].initShield(PIN_PWM_U4,PIN_PWM_V4,PIN_PWM_W4,PIN_PWM_EN_U4,PIN_PWM_EN_V4,PIN_PWM_EN_W4);
    link[3].setGearFactor(gearFactorA);
    link[3].begin();

    Serial.print("Joint 3 Sensor: ");  Serial.println(sensorError,HEX);
    Serial.print("Joint 3 shield: ");  Serial.println(shieldError,HEX);


    // link[4] = B axis, SPI2, CS1,Slave0, index=5, B axis
    sensorError = link[4].initSensor(SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
    shieldError = link[4].initShield(PIN_PWM_U5,PIN_PWM_V5,PIN_PWM_W5,PIN_PWM_EN_U5,PIN_PWM_EN_V5,PIN_PWM_EN_W5);
    link[4].setGearFactor(gearFactorB);
    link[4].begin();

    Serial.print("Joint 4 Sensor: ");  Serial.println(sensorError,HEX);
    Serial.print("Joint 4 shield: ");  Serial.println(shieldError,HEX);


    // link[5] = C axis, SPI2, CS1,Slave1, index=6, C axis
    sensorError = link[5].initSensor(SPI3W2, PIN_SPI2_SS1, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S1);
    shieldError = link[5].initShield(PIN_PWM_U6,PIN_PWM_V6,PIN_PWM_W6,PIN_PWM_EN_U6,PIN_PWM_EN_V6,PIN_PWM_EN_W6);
    link[5].setGearFactor(gearFactorC);
    link[5].begin();

    Serial.print("Joint 5 Sensor: ");  Serial.println(sensorError,HEX);
    Serial.print("Joint 5 shield: ");  Serial.println(shieldError,HEX);

    return;
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
    pinMode(LED_BUILTIN, OUTPUT);

    // Setup jointController for each joint
    jointInit();
    Serial.println("all joints are ready");
    delay(5000);

    // Setup Interrupt settings
    XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = {0};
    pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
    pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;
    XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
    XMC_CCU4_EnableClock(CCU40, 3);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, 500); // Adjust last Value or Prescaler
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

    Serial.println("timer interrupt sequence started");
    delay(5000);

}


/**
 * @brief 
 * 
 */
void loop()
{

}








