/**
 * @file MotorCalibrate.ino
 * @author your name (you@domain.com)
 * @brief Program allows easy setup and calibration of multiple BLDC motors
 * for a 6 axis robot. Can also used for other setups and BLDC motor types.
 * @version 0.1
 * @date 2024-06-17
 * 
 * @copyright Copyright (c) 2024 Infineon technologies AG
 * 
 */
#include <SD.h>
#include <tlx5012-arduino.hpp>
#include <jointController.hpp>
#include "const.h"

using namespace tle5012;

tle5012::SPIClass3W tle5012::SPI3W1(1);         //!< SPI port 1 on XMC4700 X1 according HW SPI setup
tle5012::SPIClass3W tle5012::SPI3W2(2);         //!< SPI port 2 on XMC4700 X2 according HW SPI setup

#define POLEPAIRS 4                             //! Number of pole pairs of the motor (number of poles / 2 )
#define PHASE_DELAY_1 (double)2.094395102       //! 120° offset
#define PHASE_DELAY_2 (double)4.188790205       //! 240° offset
#define jointTotalNum   6

boolean isLogging           = true;
jointController link[jointTotalNum] = {
    ( jointController( (char*)"X", isLogging ) ),
    ( jointController( (char*)"Y", isLogging ) ),
    ( jointController( (char*)"Z", isLogging ) ),
    ( jointController( (char*)"A", isLogging ) ),
    ( jointController( (char*)"B", isLogging ) ),
    ( jointController( (char*)"C", isLogging ) )
};

boolean isCalibrate         = false;
boolean isUncontrolledRun   = false;
boolean isControlledRun     = false;
boolean isSensorTesting     = false;
boolean isValidate          = false;
boolean isValid             = true;
int16_t bytes2k             = 2048;
int8_t idx                  = 0;

double angle_raw            = 0.0;
float angle_rad             = 0.0;
int duty                    = 700;
int16_t upper_phase         = 0;
int16_t lower_phase         = 0;


/**
 * @brief SDcard write routine prints out the PWM array into a text file.
 * Writes data in chunks. Before writing always remove older files and
 * blink LED during write.
 *
 * @param filename filename to write the PWM array into
 */
void writePWMArray(String filename)
{
    String buffer;
    File txtFile;

    // init the SD card
    if (!SD.begin()) {
        Serial.println("Card failed, or not present");
        return;
    }else{
        SD.remove(filename);
        txtFile = SD.open(filename, FILE_WRITE);
        if (txtFile) {
            for (int a=0;a<arraySize;a++)
            {
                buffer += link[idx].mPWM.PWM_U_values[a]; buffer += ",";
                buffer += link[idx].mPWM.PWM_V_values[a]; buffer += ",";
                buffer += link[idx].mPWM.PWM_W_values[a]; buffer += "\r\n";
                unsigned int chunkSize = txtFile.availableForWrite();
                if (chunkSize && buffer.length() >= chunkSize)
                {
                    digitalWrite(LED_BUILTIN, HIGH);
                    txtFile.write(buffer.c_str(), chunkSize);
                    digitalWrite(LED_BUILTIN, LOW);
                    buffer.remove(0, chunkSize);
                }
            }
            txtFile.write(buffer.c_str(),buffer.length());
            Serial.print("PWM array finished with number of values: "); Serial.println(arraySize);
            Serial.print("to filename: "); Serial.println(filename);
            txtFile.close();
        }else{
            Serial.print("error opening ");
            Serial.println(filename);
        }
    }
    return;
}

/**
 * @brief SDcard write routine for calibration values offset and PhaseShift.
 * Before writing always remove older files and blink LED during write.
 *
 * @param filename filename to write the calibration data into
 */
void writeCalibration(String filename)
{
    File txtFile;

    if (!SD.begin()) {
        Serial.println("Card failed, or not present");
        return;
    }else{
        SD.remove(filename);
        txtFile = SD.open(filename, FILE_WRITE);
        if (txtFile) {
            digitalWrite(LED_BUILTIN, HIGH);
            txtFile.println(link[idx].mMotor.offset);
            txtFile.println(link[idx].mMotor.phaseShift);
            txtFile.println(link[idx].mMotor.dutycycle);
            txtFile.println(link[idx].mMotor.epsilon);
            txtFile.println(link[idx].mMotor.sensorOffset);
            digitalWrite(LED_BUILTIN, LOW);
            Serial.print(link[idx].jointName);
            Serial.print(":\toffset: ");     Serial.print(link[idx].mMotor.offset);
            Serial.print("\tphaseshift: ");  Serial.print(link[idx].mMotor.phaseShift);
            Serial.print("\tdutycycle: ");   Serial.print(link[idx].mMotor.dutycycle);
            Serial.print("\tepsilon: ");     Serial.print(link[idx].mMotor.epsilon);
            Serial.print("\nto filename: "); Serial.println(filename);
            txtFile.close();
        }else{
            Serial.print("error opening ");
            Serial.println(filename);
        }
    }
    return;
}


/**
 * @brief 
 * 
 */
void serialInput()
{
    while (Serial.available() != 0)
    {
        int input = 0;
        input = Serial.read();

        /* Select Motor */
        if (input == 'x')
        {
            link[idx].switchShieldOnOff(LOW);
            idx = 0;
            Serial.println("Set motor X on index 0");
        }
        if (input == 'y')
        {
            link[idx].switchShieldOnOff(LOW);
            idx = 1;
            Serial.println("Set motor Y on index 1");
        }
        if (input == 'z')
        {
            link[idx].switchShieldOnOff(LOW);
            idx = 2;
            Serial.println("Set motor Z on index 2");
        }
        if (input == 'a')
        {
            link[idx].switchShieldOnOff(LOW);
            idx = 3;
            Serial.println("Set motor A on index 3");
        }
        if (input == 'b')
        {
            link[idx].switchShieldOnOff(LOW);
            idx = 4;
            Serial.println("Set motor B on index 4");
        }
        if (input == 'c')
        {
            link[idx].switchShieldOnOff(LOW);
            idx = 5;
            Serial.println("Set motor C on index 5");
        }

        /* Raise/Lower  duty cycle */
        if (input == '1')
        {
            link[idx].mMotor.dutycycle -= 10;
            Serial.print("\t duty: ");
            Serial.println(link[idx].mMotor.dutycycle);
        }
        if (input == '2')
        {
            link[idx].mMotor.dutycycle = 1000;
            Serial.print("\t duty: ");
            Serial.println(link[idx].mMotor.dutycycle);
        }
        if (input == '3')
        {
            link[idx].mMotor.dutycycle += 10;
            Serial.print("\t duty: ");
            Serial.println(link[idx].mMotor.dutycycle);
        }

        /* Raise/Lower  phaseshift */
        if (input == '4')
        {
            link[idx].mMotor.phaseShift -= 1;
            Serial.print("\t phaseshift: ");
            Serial.println(link[idx].mMotor.phaseShift);
        }
        if (input == '5')
        {
            link[idx].mMotor.phaseShift = 250;
            Serial.print("\t phaseshift: ");
            Serial.println(link[idx].mMotor.phaseShift);
        }
        if (input == '6')
        {
            link[idx].mMotor.phaseShift += 1;
            Serial.print("\t phaseshift: ");
            Serial.println(link[idx].mMotor.phaseShift);
        }

        /* Raise/Lower  offset */
        if (input == '7')
        {
            link[idx].mMotor.offset -= 1;
            Serial.print("\t offset: ");
            Serial.println(link[idx].mMotor.offset);
        }
        if (input == '8')
        {
            link[idx].mMotor.offset = 0;
            Serial.print("\t offset: ");
            Serial.println(link[idx].mMotor.offset);
        }
        if (input == '9')
        {
            link[idx].mMotor.offset += 1;
            Serial.print("\t offset: ");
            Serial.println(link[idx].mMotor.offset);
        }


        /* Run controller */
        if (!isControlledRun && input == 'u')
        {
            link[idx].switchShieldOnOff(HIGH);
            isUncontrolledRun = true;
            isControlledRun = false;
            isValidate = false;
            isSensorTesting = false;
            Serial.println("uncontrolled Run, collect PWM values");
        }

        if (!isUncontrolledRun && input == 'r')
        {
            link[idx].switchShieldOnOff(HIGH);
            isControlledRun = true;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;
            Serial.println("controlled Run, find optimal offset and phase shift values");
        }

        /* Validating PWM */
        if (input == 'v')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = true;
            isSensorTesting = false;
            Serial.println("validating PWM arrays");
        }

        /* Shield switch off and start sensor output*/
        if (input == 's')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = true;
        }

        /* Help */
        if (input == 'h')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;
            help();
        }

        /* PWM read and write functions*/
        if (input == 'p')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;
            writePWMArray( String("PWM-C-") + String(link[idx].jointName) + String(".csv") );
        }

        if (input == 'l')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;
            link[idx].readFromSD(
                String("PWM-C-") + String(link[idx].jointName) + String(".csv"),
                String("CAL_C_") + String(link[idx].jointName) + String(".txt") );
        }

        if (input == 'k')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;
            link[idx].readFromSD(
                String("PWM") + String(link[idx].jointName) + String(".csv"),
                String("CAL") + String(link[idx].jointName) + String(".txt") );
        }

        if (input == 'w')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;
            writeCalibration( String("CAL_C_") + String(link[idx].jointName) + String(".txt") );
        }

        /* phaseshift and offset calibration functions*/
        if (input == '+')
        {
            upper_phase = link[idx].mMotor.phaseShift;
            Serial.print("\tUpper Phaseshift: ");
            Serial.println(upper_phase);
        }
        if (input == '*')
        {
            link[idx].switchShieldOnOff(LOW);
            Serial.print("\tToggle Phaseshift: ");
            Serial.print(link[idx].mMotor.phaseShift);
            link[idx].mMotor.phaseShift = -1*link[idx].mMotor.phaseShift;
            Serial.print(" -> to: ");
            Serial.print(link[idx].mMotor.phaseShift);
            link[idx].switchShieldOnOff(HIGH);
        }
        if (input == '-')
        {
            lower_phase = link[idx].mMotor.phaseShift;
            Serial.print("\tLower Phaseshift: ");
            Serial.println(lower_phase);
        }

        if (input == 'i')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;

            Serial.print(link[idx].jointName);
            Serial.print(":\tphaseshift: ");
            Serial.print(link[idx].mMotor.phaseShift);
            Serial.print("\toffset:");
            Serial.print(link[idx].mMotor.offset);
            Serial.print("\tdutycycle: ");
            Serial.print(link[idx].mMotor.dutycycle);
            Serial.println("\n");

        }

        if (input == 'o')
        {
            link[idx].switchShieldOnOff(LOW);
            isControlledRun = false;
            isUncontrolledRun = false;
            isValidate = false;
            isSensorTesting = false;

            Serial.print(link[idx].jointName);
            Serial.print(":\tphaseshift: ");
            Serial.print(link[idx].mMotor.phaseShift);
            Serial.print("\toffset:");
            Serial.print(link[idx].mMotor.offset);
            Serial.print("\tdutycycle: ");
            Serial.print(link[idx].mMotor.dutycycle);
            Serial.println("");

            Serial.print(link[idx].jointName);
            Serial.print(" cal:\tphaseshift: ");
            link[idx].mMotor.phaseShift = (upper_phase - lower_phase) / 2;
            link[idx].mMotor.offset = (upper_phase + lower_phase) / 2;
            Serial.print(link[idx].mMotor.phaseShift);
            Serial.print("\toffset: ");
            Serial.print(link[idx].mMotor.offset);
            Serial.println("\n");

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
    // link[0] = X axis = base; SPI1,CS1,Slave0
    link[0].initSensor(SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
    link[0].initShield(PIN_PWM_U1,PIN_PWM_V1,PIN_PWM_W1,PIN_PWM_EN1,PIN_PWM_EN1,PIN_PWM_EN1);
    link[0].setPWMResolution(2048);
    link[0].begin(DISABLED);
    link[0].switchShieldOnOff(LOW);
    link[0].jointEnable(true);

    // link[1] = Y axis, SPI1, CS1,Slave1
    link[1].initSensor(SPI3W1, PIN_SPI1_SS1, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S1);
    link[1].initShield(PIN_PWM_U2,PIN_PWM_V2,PIN_PWM_W2,PIN_PWM_EN2,PIN_PWM_EN2,PIN_PWM_EN2);
    link[1].setPWMResolution(2048);
    link[1].begin(DISABLED);
    link[1].switchShieldOnOff(LOW);
    link[1].jointEnable(true);

    // // link[2] = Z axis, SPI2, CS1,Slave0
    link[2].initSensor(SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
    link[2].initShield(PIN_PWM_U3,PIN_PWM_V3,PIN_PWM_W3,PIN_PWM_EN3,PIN_PWM_EN3,PIN_PWM_EN3);
    link[2].setPWMResolution(2048);
    link[2].begin(DISABLED);
    link[2].switchShieldOnOff(LOW);
    link[2].jointEnable(true);

    // link[3] = A axis, SPI2, CS2,Slave1
    link[3].initSensor(SPI3W2, PIN_SPI2_SS1, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S1);
    link[3].initShield(PIN_PWM_U4,PIN_PWM_V4,PIN_PWM_W4,PIN_PWM_EN4,PIN_PWM_EN4,PIN_PWM_EN4);
    link[3].setPWMResolution(2048);
    link[3].begin(DISABLED);
    link[3].switchShieldOnOff(LOW);
    link[3].jointEnable(true);

    // link[4] = B axis, SPI2, CS2,Slave2
    link[4].initSensor(SPI3W2, PIN_SPI2_SS2, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S2);
    link[4].initShield(PIN_PWM_U5,PIN_PWM_V5,PIN_PWM_W5,PIN_PWM_EN5,PIN_PWM_EN5,PIN_PWM_EN5);
    link[4].setPWMResolution(2048);
    link[4].begin(DISABLED);
    link[4].switchShieldOnOff(LOW);
    link[4].jointEnable(true);

    // link[5] = C axis, SPI2, CS3,Slave3
    link[5].initSensor(SPI3W2, PIN_SPI2_SS3, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S3);
    link[5].initShield(PIN_PWM_U6,PIN_PWM_V6,PIN_PWM_W6,PIN_PWM_EN6,PIN_PWM_EN6,PIN_PWM_EN6);
    link[5].setPWMResolution(2048);
    link[5].begin(DISABLED);
    link[5].switchShieldOnOff(LOW);
    link[5].jointEnable(true);

    return;
}



extern "C"
{
    void CCU40_0_IRQHandler(void)
    {

        if (isUncontrolledRun){
            angle_rad += 0.0174532925;

            int16_t pwmOne   = bytes2k * sin(angle_rad);
            int16_t pwmTwo   = bytes2k * sin(angle_rad + PHASE_DELAY_1);
            int16_t pwmThree = bytes2k * sin(angle_rad + PHASE_DELAY_2);
            analogWrite(link[idx].mPin.pin_U, bytes2k + link[idx].mMotor.dutycycle * pwmOne   / bytes2k);
            analogWrite(link[idx].mPin.pin_V, bytes2k + link[idx].mMotor.dutycycle * pwmTwo   / bytes2k);
            analogWrite(link[idx].mPin.pin_W, bytes2k + link[idx].mMotor.dutycycle * pwmThree / bytes2k);


            link[idx].sensor->getAngleValue(angle_raw);

            double angle = angle_raw >= 0
                ? angle_raw
                : 360 + angle_raw;

            int16_t angle_out = angle * resolution;

            link[idx].mPWM.PWM_U_values[angle_out] = pwmOne;
            link[idx].mPWM.PWM_V_values[angle_out] = pwmTwo;
            link[idx].mPWM.PWM_W_values[angle_out] = pwmThree;

        }else

        if (isValidate){
            isValid = true;
            for (int16_t i=0;i<arraySize;i++)
            {
                Serial.print(link[idx].mPWM.PWM_U_values[i]);
                Serial.print(" ");
                Serial.print(link[idx].mPWM.PWM_V_values[i]);
                Serial.print(" ");
                Serial.print(link[idx].mPWM.PWM_W_values[i]);
                Serial.println("");
                if(link[idx].mPWM.PWM_U_values[i] == 0.0 && link[idx].mPWM.PWM_V_values[i] == 0.0 && link[idx].mPWM.PWM_W_values[i] == 0.0)
                {
                    isValid = false;
                }
            }
            if(isValid)
            {
                Serial.println("all PWM values fetched and none zero, array is valid");
            }else{
                Serial.println("some PWM values are missing, try to run the uncontrolled mode again and longer");
            }
            isValidate = false;
        }else

        if (isControlledRun){
            link[idx].sensor->getAngleValue(angle_raw);

            double angle = angle_raw >= 0
                ? angle_raw
                : 360 + angle_raw;

            int angle_table = angle * resolution + link[idx].mMotor.phaseShift + link[idx].mMotor.offset;
            if (angle_table >= arraySize )
            {
                angle_table -= arraySize;
            }
            if (angle_table < 0000)
            {
                angle_table += arraySize;
            }

            analogWrite(link[idx].mPin.pin_U, bytes2k + link[idx].mMotor.dutycycle *  link[idx].mPWM.PWM_U_values[angle_table] / bytes2k );
            analogWrite(link[idx].mPin.pin_V, bytes2k + link[idx].mMotor.dutycycle *  link[idx].mPWM.PWM_V_values[angle_table] / bytes2k );
            analogWrite(link[idx].mPin.pin_W, bytes2k + link[idx].mMotor.dutycycle *  link[idx].mPWM.PWM_W_values[angle_table] / bytes2k );

        }

        if (isSensorTesting){
            for(int8_t i=0;i<jointTotalNum;i++)
            {
                link[i].sensor->getAngleValue(angle_raw);
                Serial.print("\t\t");
                Serial.print(link[i].jointName);
                Serial.print(": ");
                Serial.print(angle_raw);
            }
            Serial.println("");
            delay(100);
        }

        serialInput();

    }

}

/**
 * @brief 
 * 
 */
void help()
{
    Serial.println("\n\t Help  ");
    Serial.println("\t To start first we have to run in uncontrolled mode to collect the PWM array.");
    Serial.println("\t Then this array should be check if all cells are filled");
    Serial.println("\t and last the offset and phase shift setting have to be found.");
    Serial.println("\t You can run each step for each motor separately but always in the order uncontrolled, validate, controlled calibration");
    Serial.println("\n\t First of all select a motor:");
    Serial.println("\t x = motor link 0 = default");
    Serial.println("\t y = motor link 1");
    Serial.println("\t z = motor link 2");
    Serial.println("\t a = motor link 3");
    Serial.println("\t b = motor link 4");
    Serial.println("\t c = motor link 5");
    Serial.println("\n\t To start a mode insert");
    Serial.println("\t u = uncontrolled mode, the motor is running and collects all PWM data");
    Serial.println("\t v = validate checks ");
    Serial.println("\t r = controlled calibration run");
    Serial.println("\t s = sensor test prints out angles from all sensors");
    Serial.println("\t h = help, printout this help text");
    Serial.println("\n\t To read/write calibrated PWM and CAL data");
    Serial.println("\t p = save calibrated PWM array, SD card needed");
    Serial.println("\t l = load calibrated PWM array, SD card needed");
    Serial.println("\t k = load calibrated PWM-C array, SD card needed");
    Serial.println("\t w = save calibrated offset and phase shift, SD card needed");
    Serial.println("\n\t Find phaseshift and offset values");
    Serial.println("\t i = printout actual calibration values");
    Serial.println("\t o = printout actual calibration values and calibrate them");
    Serial.println("\t + = save upper phaseshift value for calibration");
    Serial.println("\t * = toggle phaseshift value to the opposite");
    Serial.println("\t - = save lower phaseshift value for calibration");
    Serial.println("\n\t Set duty cycle");
    Serial.println("\t 1 = lower duty cycle by -10");
    Serial.println("\t 2 = set default duty cycle to 1000");
    Serial.println("\t 3 = raise duty cycle by +10");
    Serial.println("\n\t Set phaseshift value");
    Serial.println("\t 4 = lower phaseshift by -1");
    Serial.println("\t 5 = set default phaseshift to 250");
    Serial.println("\t 6 = raise phaseshift by +1");
    Serial.println("\n\t Set offset value");
    Serial.println("\t 7 = lower offset by -1");
    Serial.println("\t 8 = set default offset to 0");
    Serial.println("\t 9 = raise duty by +1");
    Serial.println("\n");
}


void setup()
{
    Serial.begin(115200);
    while (!Serial){};
    delay (5000);

    analogWriteResolution(12);
    pinMode(LED_BUILTIN, OUTPUT);
    jointInit();
    Serial.println("All joints are ready");
    delay(1000);


    // Setup Interrupt settings
    XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = {0};
    pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
    pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;
    // Setup interrupt1
    XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
    XMC_CCU4_EnableClock(CCU40, 3);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, 200); // Adjust last Value or Prescaler
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

    Serial.println("Ready to calibrate");
    help();
}

void loop()
{
    serialInput();
    delay(10);
}