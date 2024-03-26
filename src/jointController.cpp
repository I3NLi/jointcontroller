/*!
 * @file        jointController.cpp
 * @date        March 2024
 * @copyright   Copyright (c) 2019-2024 Infineon Technologies AG
 * @brief
 *
 * SPDX-License-Identifier: MIT
 */


#include "jointController.hpp"


/**
 * @brief Construct a new joint Controller::joint Controller object
 * set all internal variables and structures to default.
 *
 */
jointController::jointController(char *idx)
{
    this->sensor        = NULL;
    index               = *idx;

    mGearFactor         = 1.0;
    mPWMResolution      = 2048;

    mPid.P              = 0.0;
    mPid.I              = 0.0;
    mPid.D              = 0.0;

    mLimits.minLimit    = 0.0;
    mLimits.maxLimit    = 0.0;
    mLimits.startPos    = 0.0;

    mMotor.offset       = 0;
    mMotor.phaseShift   = 0;

}

/**
 * @brief Destroy the joint Controller::joint Controller object
 *
 */
jointController::~jointController()
{
}

/**
 * @brief begin function starts the motor and sensor object if they are set
 *
 */
void jointController::begin()
{
    char filename_pwm[] = "pwmX.txt";           //! Filename for motor PWM array <-- change this for different motors
    char filename_pid[] = "pidX.txt";           //! Filename for motor calibration values <-- change this for different motors
    filename_pwm[3] = index;
    filename_pid[3] = index;

    _readPWMArray(filename_pwm);
    _readMotorCalibration(filename_pid);
}

/**
 * @brief construct a TLE5012 sensor for the jointController
 *
 * @param bus      void pointer to the object representing the SPI class
 * @param csPin    pin number of the CS pin
 * @param misoPin  MISO pin for the SPI/SSC interface
 * @param mosiPin  MOSI pin for the SPI/SSC interface
 * @param sckPin   system clock pin for external sensor clock setting
 * @param slave    Tle5012b optional sensor slave setting
 +
 * @return errorTypes returns an error form TLx5012 lib if sensor ist ok
 */
errorTypes jointController::initSensor(SPIClass3W &bus, uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sckPin, Tle5012Ino::slaveNum slave)
{
    this->sensor = new Tle5012Ino(&bus, csPin, misoPin, mosiPin, sckPin, slave);
    return sensor->begin();
}

/**
 * @brief sets the IFX007 PWM and GPIO pins, than sets the PWM frequency
 * and the GPIO mode, so that we can use control the BLDC half bridges.
 *
 * @param U         pin must be PWM capable
 * @param V         ditto for V
 * @param W         ditto for W
 * @param EN_U      GPIO pins to switch on and off
 * @param EN_V      ditto
 * @param EN_W      ditto
 *
 * @return errorTypes returns an error if not working
 */
 errorTypes jointController::initShield(uint8_t U,uint8_t V,uint8_t W,uint8_t EN_U,uint8_t EN_V,uint8_t EN_W)
 {
    pin_U     = U;
    pin_V     = V;
    pin_W     = W;
    pin_EN_U  = EN_U;
    pin_EN_V  = EN_V;
    pin_EN_W  = EN_W;

    pinMode(pin_U, OUTPUT);
    pinMode(pin_V, OUTPUT);
    pinMode(pin_W, OUTPUT);

    setAnalogWriteFrequency(pin_U,PWMFrequency);
    setAnalogWriteFrequency(pin_V,PWMFrequency);
    setAnalogWriteFrequency(pin_W,PWMFrequency);

    pinMode(pin_EN_U, OUTPUT);
    pinMode(pin_EN_V, OUTPUT);
    pinMode(pin_EN_W, OUTPUT);

    digitalWrite(pin_EN_U, HIGH);
    digitalWrite(pin_EN_V, HIGH);
    digitalWrite(pin_EN_W, HIGH);

    return NO_ERROR;
 }

/**
 * @brief Set the Gear Factor object
 * The gear factor between the external movement and the motor/sensor
 *
 * @param gearFactor the gear ratio between sensor and joint
 */
void jointController::setGearFactor(double gearFactor)
{
    mGearFactor = gearFactor;
}

/**
 * @brief Set the limits object

 * The limits define the maximum rotation on positive/negative direction
 * in degree which is possible on this joint axis as well as the default
 * position of the joint
 *
 * @param minLimit      the minimal position in external degree (without gear factor)
 * @param maxLimit      the maximal position in external degree (without gear factor)
 * @param startPos      the default start position in external degree (without gear factor)
 */
void jointController::setRangeLimits(double minLimit, double maxLimit, double startPos)
{
    mLimits.minLimit = minLimit;
    mLimits.maxLimit = maxLimit;
    mLimits.startPos = startPos;
    return;
}

/**
 * @brief Checks if the inserted angle is inside the range Limits aof the joint
 * 
 * @param rawAngle external angle in deg
 * @return double external angle in deg and inside range limits
 */
double jointController::angleInsideRangeLimits(double rawAngle)
{
    return(constrain(rawAngle,mLimits.minLimit,mLimits.maxLimit));
}

/**
 * @brief Set the PID values into the internal PID structure
 *
 * @param P     the PID error value
 * @param I     the PID integrator value
 * @param D     the PID damping value
 */
void jointController::setPID(double P, double I, double D)
{
    mPid.P = P;
    mPid.I = I;
    mPid.D = D;
    return;
}

/**
 * @brief Set the motor calibration Values for offset and phase shift
 *
 * @param off       the offset value
 * @param phase     the phase shift value
 */
void jointController::setMotorCal(int16_t off, int16_t phase)
{
    mMotor.offset = off;
    mMotor.phaseShift = phase;
    mMotor.sensorOffset = 0.0;
    return;
}

/**
 * @brief Set the soft homing position by resetting the sensor, which will delete all
 * revolution values and leave only the sensor angle which will be set as 0 deg
 * homing position.
 * 
 * @param startPos      the default start position in external degree (without gear factor)
 */
void jointController::setHomingPosition(double startPos)
{
    errorTypes cA = sensor->resetFirmware();
    mLimits.startPos = startPos;
    mMotor.sensorOffset = calculateAngle() * -1;
}

/**
 * @brief function returns the actual position
 * The raw angle is fetched from the sensor as -180-180 degree,
 * which has to be recalculated to 0-350 degree and added with
 * the number of revolutions.
 * If no gear factor is set, than the internal angle is calculated,
 * otherwise the external angle will be calculated
 * 
 * @param gf the optional gear factor to be used (default 1.0)
 * @return double the calculated angle
 */
double jointController::calculateAngle(double gf)
{
    double  rawAngle;            //!> raw angle from the sensor in degree
    int16_t revolution;          //!> number of revolutions from sensor
    errorTypes cA = sensor->getAngleValue(rawAngle);
    errorTypes cR = sensor->getNumRevolutions(revolution);

    //!> Sensor measures the angle for -180-180 deg, so recalculate to 0-360 deg
    double angle = revolution * 360 +
        (rawAngle >=0
            ? rawAngle
            : 360 + rawAngle );

    return(angle * gf);
}

/**
 * @brief Read the PWM array from SD card into the internal arrays
 * for PWM_U/V/W
 *
 */
void jointController::_readPWMArray(char* filename)
{
    if (!SD.begin()) {
        Serial.println("Card failed, or not present");
        return;
    }else{
        // try to open the file for writing
        txtFile = SD.open(filename);
        if (txtFile) {
            int idx = 0;
            digitalWrite(LED1, HIGH);
            while (txtFile.available() && idx < arraySize) {
                PWM_U_values[idx] = txtFile.parseInt();
                PWM_V_values[idx] = txtFile.parseInt();
                PWM_W_values[idx] = txtFile.parseInt();
                idx++;
            }
            txtFile.close();
            digitalWrite(LED1, LOW);
            Serial.print("PWM read finished from: ");
            Serial.println(filename);
        }else{
            Serial.print("error opening ");
            Serial.println(filename);
        }
    }
    return;
}

/**
 * @brief reads all calibration values from a SD card file, which are
 * the PID values, the limits and the motor offset and phase shift.
 *
 */
void jointController:: _readMotorCalibration(char* filename)
{
    if (!SD.begin()) {
        Serial.println("Card failed, or not present");
        return;
    }else{
        // open pid
        txtFile = SD.open(filename);
        if (txtFile) {
            digitalWrite(LED1, HIGH);
            setPID(txtFile.parseInt(),txtFile.parseFloat(),txtFile.parseFloat());
            setRangeLimits(txtFile.parseFloat(),txtFile.parseFloat(),txtFile.parseFloat());
            setMotorCal(txtFile.parseInt(),txtFile.parseInt());
            txtFile.close();
            digitalWrite(LED1, LOW);
            Serial.print("PID read finished from : ");
            Serial.println(filename);
        }else{
            Serial.print("error opening ");
            Serial.println(filename);
        }
    }
    return;
}


/**
 * @brief the run controller moves the motor to the intent position, using the PWM array and calibration for the
 * motor and the PID controller to
 *
 * @param target_angle the angle we want to reach
 */
void jointController::runToAngle(double target_angle)
{
    double raw_angle = 0.0;                                                                 //! raw angle value from -180 deg to 180 deg
    int16_t revolutions = 0;                                                                //! number of revolutions counted as +/- 360 deg
    int16_t phaseShift = mMotor.phaseShift;                                                 //! with positive phaseShift turn clockwise

    // angle calculation
    sensor->getAngleValue(raw_angle);                                                       //! fetch raw angle from sensor
    sensor->getNumRevolutions(revolutions);                                                 //! fetch number of revolutions from sensor
    double angle360 = raw_angle >= 0                                                        //! this will synchronize real ange to intended angle
          ? raw_angle
          : 360 + raw_angle;
    double angle = revolutions * 360 + angle360;
    intentAngle = mGearFactor * target_angle + mMotor.sensorOffset;                         //! calculate target with gear factor and sensorOffset

    // PID calculation
    double error = intentAngle - angle;                                                     //! calculate position error
    double speed = lastAngle-angle;                                                         //! calculate speed damping
    lastAngle = angle;                                                                      //! update "last angle"
    mIntegrator = constrain( mIntegrator + error, -100, 100);                               //! integral factor
    double control = error * mPid.P + mIntegrator * mPid.I + speed * mPid.D;                //! calculate PID controller

    // duty cycle calculation
    if (control < 0){phaseShift *= -1;}                                                     //! turn counterclockwise
    int16_t duty = constrain(abs(control),0,255);                                           //! Limit output to meaningful range

    // PWM table fetch
    int16_t angleTable = angle360 * resolution + phaseShift + mMotor.offset;                //! find the PWM array position
    if (angleTable >= arraySize) {angleTable -= 3600;}                                      //! round trip in the  PWM array
    if (angleTable < 0000) {angleTable += 3600;}

    // motor setting
    analogWrite(pin_U, duty * PWM_U_values[angleTable] / mPWMResolution );                  //! set the PWM values to the U/V/W pins
    analogWrite(pin_V, duty * PWM_V_values[angleTable] / mPWMResolution );
    analogWrite(pin_W, duty * PWM_W_values[angleTable] / mPWMResolution );
}
