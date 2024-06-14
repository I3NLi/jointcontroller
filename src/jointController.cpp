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
jointController::jointController(char *name,boolean debugPrint)
{
    this->sensor                = NULL;
    jointName                   = *name;

    mPWMResolution              = 2048;
    mSpeed                      = 1.0;
    mIntegrator                 = 0.0;
    mIntegratorPos              = 0.0;
    mIntegratorSpeed            = 0.0;
    
    lastAngle                   = 0.0;
    lastPos                     = 0.0;
    newPos                      = 0.0;

    mPID.P                      = 0.0;
    mPID.I                      = 0.0;
    mPID.D                      = 0.0;

    mPID.P_pos                  = 0.0;
    mPID.I_pos                  = 0.0;
    mPID.D_pos                  = 0.0;

    mPID.P_speed                = 0.0;
    mPID.I_speed                = 0.0;
    mPID.D_speed                = 0.0;

    mLimits.minLimit            = 0.0;
    mLimits.maxLimit            = 0.0;
    mLimits.startPos            = 0.0;

    mMotor.offset               = 0;
    mMotor.phaseShift           = 0;
    mMotor.sensorOffset         = 0;
    mMotor.epsilon              = 0;

    mAnglePos.gearFactor        = 1.0;
    mAnglePos.targetAngle       = 0.0;
    mAnglePos.jointTargetAngle  = 0.0;
    mAnglePos.jointActualAngle  = 0.0;
    mAnglePos.jointActualPos    = 0.0;

    isLogging                   = debugPrint;
    isEnabled                   = ENABLED;
    mDirection                  = 1;
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
 * @param enable true/false setting
 */
void jointController::begin(boolean enable)
{
    jointEnable(enable);
    String cal_file = String("CAL") + String(jointName) + String(".txt");
    String pwm_file = String("PWM") + String(jointName) + String(".txt");

    if (isEnabled)
    {
        _readPWMArray(pwm_file);
        _readMotorCalibration( cal_file  );
    }

    switchShieldOnOff(HIGH);

}

/**
 * @brief Function enables/disables joint.
 * A disabled joint switches off the motor and does not 
 * set a status other than NONE
 * 
 * @param enable true/false setting
 */
void jointController::jointEnable(boolean enable)
{
    isEnabled = enable;
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
    sensorError = sensor->begin();
    this->sensor->reg.enableSpikeFilter();

    if (isLogging)
    {
        Serial.print(jointName);
        Serial.print(" sensor init: ");
        Serial.println(sensorError);
    }
    return sensorError;
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
    mPin.pin_U     = U;
    mPin.pin_V     = V;
    mPin.pin_W     = W;
    mPin.pin_EN_U  = EN_U;
    mPin.pin_EN_V  = EN_V;
    mPin.pin_EN_W  = EN_W;

    pinMode(mPin.pin_U, OUTPUT);
    pinMode(mPin.pin_V, OUTPUT);
    pinMode(mPin.pin_W, OUTPUT);

    setAnalogWriteFrequency(mPin.pin_U,PWMFrequency);
    setAnalogWriteFrequency(mPin.pin_V,PWMFrequency);
    setAnalogWriteFrequency(mPin.pin_W,PWMFrequency);

    pinMode(mPin.pin_EN_U, OUTPUT);
    pinMode(mPin.pin_EN_V, OUTPUT);
    pinMode(mPin.pin_EN_W, OUTPUT);

    shieldError = NO_ERROR;
    if (isLogging)
    {
        Serial.print(jointName);
        Serial.print(" PWM Pin: ");
        Serial.print(mPin.pin_U); Serial.print(":");
        Serial.print(mPin.pin_V); Serial.print(":");
        Serial.print(mPin.pin_W);
        Serial.print(" EN Pin: ");
        Serial.print(mPin.pin_EN_U); Serial.print(":");
        Serial.print(mPin.pin_EN_V); Serial.print(":");
        Serial.println(mPin.pin_EN_W);
    }
    return shieldError;
 }

/**
 * @brief Switches the shield enable pin HIGH or LOW
 * and therefore the BLDC on or off.
 * A disabled joint will stay off even if switched on.
 * 
 * @param status true/false setting
 */
void jointController::switchShieldOnOff(int8_t onoff)
{
    if (!isEnabled)
    {
        onoff = LOW;
    }
    digitalWrite(mPin.pin_EN_U, onoff);
    digitalWrite(mPin.pin_EN_V, onoff);
    digitalWrite(mPin.pin_EN_W, onoff);
    status = NONE;
}

/**
 * @brief 
 * 
 * @param dir 
 */
void jointController::setDirection(int8_t dir)
{
    mDirection = dir;
}

/**
 * @brief Set the Gear Factor object
 * The gear factor between the external movement and the motor/sensor
 *
 * @param gearFactor the gear ratio between sensor and joint
 */
void jointController::setGearFactor(double gearFactor)
{
    mAnglePos.gearFactor = gearFactor;
}


/**
 * @brief Set the analog pin bit resolution
 * 
 * @param resolution resolution value in bit, default = 2048
 */
void jointController::setPWMResolution(int16_t resolution)
{
    mPWMResolution = resolution;
    return;
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
 * @brief Set the PID values for a normal one stage PID 
 * motor controller
 *
 * @param P     the PID error value
 * @param I     the PID integrator value
 * @param D     the PID damping value
 */
void jointController::setPID(double P, double I, double D)
{
    mPID.P = P;
    mPID.I = I;
    mPID.D = D;
    return;
}


/**
 * @brief Set the epsilon range where inside a position is good enough
 * to be the target position. It will be recalculated to internal
 * degree by multiply with the gear factor.
 * 
 * @param epsilonRange the range in external degree
 */
 void jointController::setEpsilon(double epsilonRange)
 {
    mMotor.epsilon = epsilonRange * mAnglePos.gearFactor;
 }


/**
 * @brief Set the PID values for a position motor controller. This 
 * needs also the speed part
 *
 * @param P_pos     the PID position error value
 * @param I_pos     the PID position integrator value
 * @param D_pos     the PID position damping value
 */
void jointController::setPIDpos(double P_pos, double I_pos, double D_pos)
{
    mPID.P_pos = P_pos;
    mPID.I_pos = I_pos;
    mPID.D_pos = D_pos;
    return;
}

/**
 * @brief Set the PID values for the speed part of a position/speed PID motor
 * controller
 *
 * @param P_speed     the PID speed error value
 * @param I_speed     the PID speed integrator value
 * @param D_speed     the PID speed damping value
 */
void jointController::setPIDspeed(double P_speed, double I_speed, double D_speed)
{
    mPID.P_speed = P_speed;
    mPID.I_speed = I_speed;
    mPID.D_speed = D_speed;
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
double jointController::setHomingPosition(double startPos)
{
    errorTypes cA = sensor->resetFirmware();
    delay(50); //wait until Firmware reset comes back safely
    mLimits.startPos = startPos;
    mMotor.sensorOffset = calculateAngle();
    return mMotor.sensorOffset;
}

/**
 * @brief 
 * 
 * @return double 
 */
double jointController::getActualAngle()
{
    return mAnglePos.jointActualAngle / mAnglePos.gearFactor;
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
void jointController::_readPWMArray(String filename)
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
                mPWM.PWM_U_values[idx] = txtFile.parseInt();
                mPWM.PWM_V_values[idx] = txtFile.parseInt();
                mPWM.PWM_W_values[idx] = txtFile.parseInt();
                idx++;
            }
            txtFile.close();
            if(isLogging)
            {
                Serial.print("PWM read finished from: ");
                Serial.println(filename);
            }
            digitalWrite(LED1, LOW);
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
void jointController:: _readMotorCalibration(String filename)
{
    if (!SD.begin()) {
        Serial.println("Card failed, or not present");
        return;
    }else{
        // open pid
        txtFile = SD.open(filename);
        if (txtFile) {
            digitalWrite(LED1, HIGH);
            setMotorCal(txtFile.parseInt(),txtFile.parseInt());
            setPIDpos(txtFile.parseInt(),txtFile.parseFloat(),txtFile.parseFloat());
            setPIDspeed(txtFile.parseFloat(),txtFile.parseFloat(),txtFile.parseFloat());
            setPID(txtFile.parseInt(),txtFile.parseFloat(),txtFile.parseFloat());
            txtFile.close();
            if(isLogging)
            {
                Serial.print("PID read finished from : ");
                Serial.print(filename);
                Serial.print("; offset=");Serial.print(mMotor.offset);
                Serial.print("; phaseShift=");Serial.print(mMotor.phaseShift);
                Serial.print("; P=");Serial.print(mPID.P);
                Serial.print("; I=");Serial.print(mPID.I);
                Serial.print("; D=");Serial.print(mPID.D);
                Serial.print("; P_pos=");Serial.print(mPID.P_pos);
                Serial.print("; I_pos=");Serial.print(mPID.I_pos);
                Serial.print("; D_pos=");Serial.print(mPID.D_pos);
                Serial.print("; P_speed=");Serial.print(mPID.P_speed);
                Serial.print("; I_speed=");Serial.print(mPID.I_speed);
                Serial.print("; D_speed=");Serial.print(mPID.D_speed);
                Serial.println();
            }
            digitalWrite(LED1, LOW);
        }else{
            Serial.print("error opening ");
            Serial.println(filename);
        }
    }
    return;
}


/**
 * @brief This function runs the BLDC motor with given offset and phase shift value.
 * Therefore is will run the motor with highest speed and less current.
 * BLDC motors should run silent and stay cold when offset and phase shift are
 * optimal calibrated. Also no clicking sound should be hearable.
 * This function is for debugging of the motor run parameter.
 * 
 */
void jointController::motorRunTest()
{
    double angle_raw;
    int16_t duty = 820;
    sensor->getAngleValue(angle_raw);
    double angle = angle_raw >=0     //! this will synchronize real ange to intended angle
        ? angle_raw
        : 360 + angle_raw;

    int angleTable = angle * resolution + mMotor.phaseShift + mMotor.offset;
    if (angleTable >= arraySize ) { angleTable -= arraySize; }
    if (angleTable < 0)           { angleTable += arraySize; }

    analogWrite(mPin.pin_U, 2048 + duty * mPWM.PWM_U_values[angleTable] / 2048 );
    analogWrite(mPin.pin_V, 2048 + duty * mPWM.PWM_V_values[angleTable] / 2048 );
    analogWrite(mPin.pin_W, 2048 + duty * mPWM.PWM_W_values[angleTable] / 2048 );

    if (isLogging) {
        Serial.print(jointName);
        Serial.print("  a: ");Serial.print(angle);
        Serial.print("; ");
    }

}


/**
 * @brief function checks the running status of the joint.
 * If a target angle is inside the epsilon range, than joint can
 * be released for new settings, otherwise it is in
 * RUNNING or HOMING status.
 * 
 * @return eStatus 
 */
eStatus jointController::checkStatus()
{
    if ( abs(mAnglePos.targetAngle - mMotor.epsilon) >= mAnglePos.jointActualAngle &&
         abs(mAnglePos.targetAngle + mMotor.epsilon) <= mAnglePos.jointActualAngle )
    {
        status = NONE;
    } 
    return status;
}


/**
 * @brief Function starts the soft homing procedure.
 * The will be moved back to its default start position.
 * 
 */
eStatus jointController::homing()
{
    mAnglePos.targetAngle = mLimits.startPos;
    mAnglePos.jointTargetAngle = mLimits.startPos * mAnglePos.gearFactor;
    if(isEnabled)
    {
        status = HOMING;
    } 
    return status;
}


/**
 * @brief Set the target angle for this joint inside the
 * given min/max angles as we jet not have hard boundaries.
 * The internal target angle is calculated with the gear factor and
 * the sensor offset.
 * 
 * @param target_angle the angle we want to reach
 */
eStatus jointController::moveTo(double target_angle)
{
    if (status != HOMING){
        target_angle = target_angle * mDirection;
        mAnglePos.targetAngle = constrain( target_angle, mLimits.minLimit, mLimits.maxLimit);
        mAnglePos.jointTargetAngle = mAnglePos.targetAngle * mAnglePos.gearFactor;
        status = RUNNING;
    }
    if (!isEnabled){
        status = NONE;
    }
    return status;
}


/**
 * @brief the run controller moves the motor to the intent position, using the PWM array and calibration for the
 * motor and the PID controller to
 *
 */
void jointController::runToAnglePID()
{
    double raw_angle = 0.0;                                                                     //! raw angle value from -180 deg to 180 deg
    int16_t revolutions = 0;                                                                    //! number of revolutions counted as +/- 360 deg
    int16_t phaseShift = mMotor.phaseShift;                                                     //! with positive phaseShift turn clockwise
 
    // angle calculation
    sensor->getAngleValue(raw_angle);
    sensor->getNumRevolutions(revolutions);
     mAnglePos.jointActualPos = raw_angle >= 0                                                  //! this will synchronize real angle to intended angle
          ? raw_angle
          : 360 + raw_angle;
    mAnglePos.jointActualAngle = revolutions * 360 +  mAnglePos.jointActualPos;
 
    // PID calculation
    double error = mAnglePos.jointTargetAngle - mAnglePos.jointActualAngle;                     //! calculate position error
    double speed = lastAngle - mAnglePos.jointActualAngle;                                      //! calculate speed damping
    lastAngle = mAnglePos.jointActualAngle;
    mIntegrator = constrain( mIntegrator + error, -100, 100);
    double control = error * mPID.P + mIntegrator * mPID.I + speed * mPID.D;                    //! calculate PID controller
 
    // duty cycle calculation
    if (control < 0){phaseShift *= -1;}                                                         //! turn counterclockwise
    int16_t duty = constrain(abs(control),0,mPWMResolution);                                    //! Limit output to meaningful range
 
    // PWM table fetch
    int16_t angleTable =  mAnglePos.jointActualPos * resolution + phaseShift + mMotor.offset;
    if (angleTable >= arraySize) { angleTable -= arraySize; }
    if (angleTable < 0)          { angleTable += arraySize; }
 
    // motor setting
    analogWrite(mPin.pin_U, mPWMResolution + duty * mPWM.PWM_U_values[angleTable] / mPWMResolution );     //! set the PWM values to the U/V/W pins
    analogWrite(mPin.pin_V, mPWMResolution + duty * mPWM.PWM_V_values[angleTable] / mPWMResolution );
    analogWrite(mPin.pin_W, mPWMResolution + duty * mPWM.PWM_W_values[angleTable] / mPWMResolution );

    // if (isLogging) {
    //     Serial.print(jointName);
    //     Serial.print("  a: ");Serial.print(mAnglePos.jointActualAngle);
    //     Serial.print("  t: ");Serial.print(mAnglePos.jointTargetAngle);
    //     Serial.println("\t");
    // }
}


/**
 * @brief Position part of the double speed/pos PID controller.
 * This part should run slower than the speed part.
 */
void jointController::runToAnglePOS()
{
    double error_pos = 0.0;

    if(abs(lastAngle - mAnglePos.jointActualAngle ) < 350.0)
    {
        error_pos = lastAngle - mAnglePos.jointActualAngle;
    }
    lastAngle = mAnglePos.jointActualAngle;
    if( newPos < mAnglePos.jointTargetAngle ){ newPos += 1 * mAnglePos.gearFactor; }
    if( newPos > mAnglePos.jointTargetAngle ){ newPos -= 1 * mAnglePos.gearFactor; }

    // PID pos calculation
    error_pos = newPos - mAnglePos.jointActualAngle;
    mIntegratorPos = constrain(mIntegratorPos + error_pos,-100, 100);
    mSpeed = error_pos * mPID.P_pos + mIntegratorPos * mPID.I_pos + error_pos * mPID.D_pos; 

    // if (isLogging) {
    //     Serial.print(" Pos: ");
    //     Serial.print(jointName);
    //     Serial.println(mSpeed);
    //     // Serial.print(" Pos s: ");Serial.print(mSpeed);
    //     // Serial.print("  i: ");Serial.print(mIntegratorPos);
    //     // Serial.print("  e: ");Serial.print(error_pos);
    //     // Serial.print("  n: ");Serial.print(newPos);
    //     // Serial.print("  l: ");Serial.print(lastAngle);
    //     // Serial.print("\n");
    // }
}


/**
 * @brief Speed part of the double speed/pos PID controller
 * This part must run faster and more often than the pos part to reduce the
 * speed near the target angle.
 * this prevents the motor from stopping very hard.
 * 
 */
void jointController::runToAngleSpeed()
{
    double raw_angle = 0.0;                                                                     //! raw angle value from -180 deg to 180 deg
    int16_t revolutions = 0;                                                                    //! number of revolutions counted as +/- 360 deg
    int16_t phaseShift = abs(mMotor.phaseShift);                                                //! with positive phaseShift turn clockwise

    double speed = 0.0;

    // angle calculation
    sensor->getAngleValue(raw_angle);
    sensor->getNumRevolutions(revolutions);
    mAnglePos.jointActualPos = raw_angle >= 0                                                   //! this will synchronize real angle to intended angle
          ? raw_angle
          : 360 + raw_angle;
    mAnglePos.jointActualAngle = revolutions * 360 + mAnglePos.jointActualPos - mMotor.sensorOffset;

    // speed damping near target position
    if(abs( lastPos - mAnglePos.jointActualPos ) < 350.0)
    {
        speed = lastPos - mAnglePos.jointActualPos;
        speed = constrain (speed, -0.5, 0.5);
    }
    lastPos = mAnglePos.jointActualPos;

    // PID speed calculation
    double error_speed = mSpeed - speed;
    mIntegratorSpeed = constrain(mIntegratorSpeed + error_speed,-100, 100);
    double control_speed = error_speed * mPID.P_speed + mIntegratorSpeed * mPID.I_speed + speed * mPID.D_speed; 

    // duty cycle calculation
    if (control_speed <  0){ phaseShift *= mMotor.phaseShift * -1;}
    int16_t duty = constrain(abs(control_speed),0,mPWMResolution);

    // PWM table fetch
    int16_t angleTable = mAnglePos.jointActualPos * resolution + phaseShift + mMotor.offset;
    if (angleTable >= arraySize) { angleTable -= arraySize; }
    if (angleTable < 0)          { angleTable += arraySize; }
 
    // motor setting
    analogWrite(mPin.pin_U, mPWMResolution + duty * mPWM.PWM_U_values[angleTable] / mPWMResolution );
    analogWrite(mPin.pin_V, mPWMResolution + duty * mPWM.PWM_V_values[angleTable] / mPWMResolution );
    analogWrite(mPin.pin_W, mPWMResolution + duty * mPWM.PWM_W_values[angleTable] / mPWMResolution );

    // if (isLogging) {
    //     Serial.print(" Speed ");
    //     Serial.print(jointName);
    //     Serial.print(mAnglePos.jointActualAngle);
    //     Serial.print("  p: ");Serial.print(mAnglePos.jointActualPos);
    //     Serial.print("  t: ");Serial.print(mAnglePos.jointTargetAngle);
    //     Serial.print("  d: ");Serial.print(duty);
    //     Serial.print("  s: ");Serial.print(speed);
    //     Serial.print("  l: ");Serial.print(lastPos);
    //     Serial.print("\n");
    // }
}



/**
 * TODO
 * 
 * differential B/C joints
 */
// X:
// P_P=78 I_P=10.27 D=-0.2 P_S=0.08 P_I=0.14 D_I=0
// Y:
// P_P=45 I_P=6.26 D=-0.48 P_S=0.03 P_I=0.06 D_I=0
// Z:
// P_P=43 I_P=4.94   D=-0.2 P_S=0.03 P_I=0.06 D_I=0
// A:
// P_P=35 I_P=3.97 D=-0.48 P_S=0.03 P_I=0.06 D_I=0
// CR:
// P_P=32 I_P=5.7  D=-0.32 P_S=0.03 P_I=0.06 D_I=0
// CL:
// P_P=30 I_P=5.8  D=-0.28 P_S=0.03 P_I=0.06 D_I=0
