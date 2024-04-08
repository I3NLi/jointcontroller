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

uint8_t idTimer1;                                //!< Timer interrupt flags for normal PID or position
uint8_t okTimer1;

/*!
 * Global controller array holds pointers for each joint. Each joint is a combination of
 * a TLx5012 magnetic angle sensor and an IFX007T BLDC shield. The TLx5012 communicates via SPI
 * an is initialized with the init function of the jointController. The IF007T shields uses PWM and
 * GPIO pins which where set with the init function
 */
#define jointTotalNum  1                                     //!> total number of controlled joints
jointController link[jointTotalNum] = {                      //!> joint controller array
    // ( jointController( (char*)"X", true ) ),
    // ( jointController( (char*)"Y", true ) ),
    // ( jointController( (char*)"Z", true ) ),
    ( jointController( (char*)"A", true ) ),
    // ( jointController( (char*)"B", true ) ),
    // ( jointController( (char*)"C", true ) )
};

/**
 * @brief array with the intended angles for each joint
 * 
 */
volatile double intent_angle[jointTotalNum] = {
    0.0,
    // 0.0,
    // 0.0,
    // 0.0,
    // 0.0,
    // 0.0
};

double testAngle = 0.0;



/**
 * @brief Initialized all joints with sensor and shield, set basic values for the gear factor, 
 * the range limits and startups the jointController
 * 
 */
void jointInit()
{
    // link[0] = X axis = base; SPI1,CS1,Slave0
    // link[0].initSensor(SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
    // link[0].initShield(PIN_PWM_U1,PIN_PWM_V1,PIN_PWM_W1,PIN_PWM_EN1,PIN_PWM_EN1,PIN_PWM_EN1);
    // link[0].setRangeLimits(minLimit_X,maxLimit_X,defaultPos_X);
    // link[0].setGearFactor(gearFactor_X);
    // link[0].begin();

    // // link[0] = Y axis, SPI1, CS1,Slave1
    // link[1].initSensor(SPI3W1, PIN_SPI1_SS1, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S1);
    // link[1].initShield(PIN_PWM_U2,PIN_PWM_V2,PIN_PWM_W2,PIN_PWM_EN2,PIN_PWM_EN2,PIN_PWM_EN2);
    // link[1].setRangeLimits(minLimit_Y,maxLimit_Y,defaultPos_Y);
    // link[1].setGearFactor(gearFactor_Y);
    // link[1].setEpsilon(epsilon_Y);
    // link[1].begin(ENABLED);

    // // link[2] = Z axis, SPI2, CS1,Slave0, index=3, Z axis
    // link[2].initSensor(SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
    // link[2].initShield(PIN_PWM_U3,PIN_PWM_V3,PIN_PWM_W3,PIN_PWM_EN3,PIN_PWM_EN3,PIN_PWM_EN3);
    // link[2].setRangeLimits(minLimit_Z,maxLimit_Z,defaultPos_Z);
    // link[2].setGearFactor(gearFactor_Z);
    // link[2].setEpsilon(epsilon_Z);
    // link[2].begin(ENABLED);

    // // link[3] = A axis, SPI2, CS2,Slave1, index=4, A axis
    link[0].initSensor(SPI3W2, PIN_SPI2_SS1, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S1);
    link[0].initShield(PIN_PWM_U4,PIN_PWM_V4,PIN_PWM_W4,PIN_PWM_EN4,PIN_PWM_EN4,PIN_PWM_EN4);
    link[0].setRangeLimits(minLimit_A,maxLimit_A,defaultPos_A);
    link[0].setGearFactor(gearFactor_A);
    link[0].setEpsilon(epsilon_A);
    link[0].begin(ENABLED);

    // // link[4] = B axis, SPI2, CS2,Slave2, index=5, B axis
    // link[4].initSensor(SPI3W2, PIN_SPI2_SS2, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S2);
    // link[4].initShield(PIN_PWM_U5,PIN_PWM_V5,PIN_PWM_W5,PIN_PWM_EN5,PIN_PWM_EN5,PIN_PWM_EN5);
    // link[4].setRangeLimits(minLimit_CR,maxLimit_CR,defaultPos_CR);
    // link[4].setGearFactor(gearFactor_CR);
    // link[4].setEpsilon(epsilon_B);
    // link[4].begin(ENABLED);

    // // link[5] = C axis, SPI2, CS3,Slave3, index=6, C axis
    // link[5].initSensor(SPI3W2, PIN_SPI2_SS3, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S3);
    // link[5].initShield(PIN_PWM_U6,PIN_PWM_V6,PIN_PWM_W6,PIN_PWM_EN6,PIN_PWM_EN6,PIN_PWM_EN6);
    // link[5].setRangeLimits(minLimit_CL,maxLimit_CL,defaultPos_CL);
    // link[5].setGearFactor(gearFactor_CL);
    // link[5].setEpsilon(epsilon_C);
    // link[5].begin(ENABLED);

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
    delay(5000);
    Serial.println("\nARCTOS 2 XMC begin setup\n");

    // set LED pin to output, used to blink when writing
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    analogWriteResolution(12);


    // Setup jointController for each joint
    jointInit();
    Serial.println("All joints are ready");
    delay(2000);

    timerSetupSingle(500);
    Serial.println("Timer single interrupt added");

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


}



/**
 * @brief 
 * 
 */
extern "C" {

    /**
     * @brief 
     * 
     */
    void CCU40_0_IRQHandler(void) //Position control
    { 
        digitalWrite(LED2, HIGH);
        for (int8_t i=0; i<jointTotalNum; i++){
            link[i].motorRunTest();
        }
        Serial.println("");
        digitalWrite(LED2, LOW);


        while (Serial.available() != 0)
        {
            int input = 0;
            input = Serial.read();

            // + sign raises the PhaseShift
            if (input == 43)
            {
            link[0].mMotor.phaseShift +=1;
            Serial.print("\t"); Serial.println(link[0].mMotor.phaseShift);
            }

            // - sign lowers the PhaseShift
            if (input == 45)
            {
            link[0].mMotor.phaseShift -=1;
            Serial.print("\t"); Serial.println(link[0].mMotor.phaseShift);
            }

            if (input == 111)
            {
                link[0].jointEnable(DISABLED);
                link[0].switchShieldOnOff(HIGH);
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

