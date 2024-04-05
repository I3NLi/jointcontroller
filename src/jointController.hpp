/*!
 * @file        jointController.hpp
 * @date        March 2024
 * @copyright   Copyright (c) 2019-2024 Infineon Technologies AG
 * @brief
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef JOINTCONTROLLER_HPP
#define JOINTCONTROLLER_HPP

#include <SD.h>
#include "Arduino.h"
#include <tlx5012-arduino.hpp>

using namespace tle5012;

#define PWMFrequency   20000                                //! 20kHz PHM frequency so that motor does not fiep

const int resolution = 10;                                  //!> array resolution 10 = 0.1 deg <-- change array size here
const int arraySize  = 360 * resolution;                    //!> array size for 360 deg

/*!
* The status flag list
* This flags are used to indicate the actual operation of the motor
*/
enum eStatus
{
    NONE=0,                                         //!> NONE motor has no flag
    RUNNING,                                        //!> RUNNING motor is running without any other status
    HOMING,                                         //!> Running to home position
    LIMIT                                           //!> LIMIT motor has reached its soft limit
}; 


class jointController
{
    public:

        char jointName;                                     //! name of the joint
        Tle5012Ino *sensor;                                 //!> pointer to the tlX501 sensor
        typedef struct pidParm {
            double P;                                       //!> motor P value
            double I;                                       //!> motor integrator I value
            double D;                                       //!> motor derivative D value
            double P_pos;                                   //!> motor position P value
            double I_pos;                                   //!> motor position integrator I value
            double D_pos;                                   //!> motor position derivative D value
            double P_speed;                                 //!> motor speed P value
            double I_speed;                                 //!> motor speed integrator I value
            double D_speed;                                 //!> motor speed derivative D value
        } pidParam_t;

        typedef struct posLimits {
            double minLimit;                                //!> the maximal position in negative direction in degree
            double maxLimit;                                //!> the maximal position in positive direction in degree
            double startPos;                                //!> variance where |actualPos-setpointPos| <= epsilon is, position is reached
        } posLimits_t;

        typedef struct motorSetup {
            int16_t offset;                                 //!> motor offset value for the electrical field
            int16_t phaseShift;                             //!> motor phase shift value for the electrical field 
            double sensorOffset;                            //!> sensor offset for the mechanical 0 deg
            double epsilon;                                 //!> epsilon range around a target position which is still ok
        } motorSetup_t;

        typedef struct anglePos {
            double targetAngle;                             //!> external target angle to reach
            double gearFactor;                              //!> gear factor setting
            double jointTargetAngle;                        //!> internal joint target angle to reach including gear factor and sensor offset
            double jointActualAngle;                        //!> internal actual angle reached
            double jointActualPos;                          //!> internal actual position 0-360° reached
        } anglePos_t;


        jointController(char *name,boolean debugPrint);
        ~jointController();

        void begin();

        errorTypes initSensor(SPIClass3W &bus, uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sckPin, Tle5012Ino::slaveNum slave);
        errorTypes initShield(uint8_t U,uint8_t V,uint8_t W,uint8_t EN_U,uint8_t EN_V,uint8_t EN_W);

        void setPID(double P, double I, double D);
        void setPIDpos(double P_pos, double I_pos, double D_pos);
        void setPIDspeed(double P_speed, double I_speed, double D_speed);
        void setEpsilon(double epsilonRange);
        void setRangeLimits(double minLimit=0.0, double maxLimit=360.0, double startPos=0.0);
        void setPWMResolution(int16_t resolution=2048);
        void setMotorCal(int16_t off,int16_t phase);
        void setGearFactor(double gearFactor=1.0);
        void readFromSD(const char* filename);

        double setHomingPosition(double startPos=0.0);
        double angleInsideRangeLimits(double rawAngle);
        double calculateAngle(double gf=1.0);

        void switchShieldOnOff(int8_t status);

        eStatus checkStatus();
        eStatus homing();
        eStatus moveTo(double target_angle);

        void runToAnglePID();
        void runToAnglePOS();
        void runToAngleSpeed();
        void motorRunTest();

        eStatus status = NONE;


    private:

        File txtFile;                                   //! File object to represent file
        boolean isLogging = false;                      //! debug printing

        errorTypes sensorError = NO_ERROR;              //!> error type for sensors
        errorTypes shieldError = NO_ERROR;              //!> error type for shields

        uint8_t pin_U;                                  //!> U wave for BLDC
        uint8_t pin_V;                                  //!> ditto V 120 deg ahead
        uint8_t pin_W;                                  //!> ditto W again 120 deg ahead
        uint8_t pin_EN_U;                               //!> switch on U channel
        uint8_t pin_EN_V;                               //!> ditto V channel
        uint8_t pin_EN_W;                               //!> ditto W channel

        posLimits_t     mLimits;                        //!> limits structure
        pidParam_t      mPid;                           //!> PID values structure
        anglePos_t      mAnglePos;
        motorSetup_t    mMotor;                         //!> motor setup values structure

        double          mPWMResolution;                 //!> bit resolution of the analog pins
        volatile double mIntegrator;                    //!> this will sum all the errors for i part of pid controller
        volatile double mIntegratorSpeed;               //!> speed Integrator for speed/pos PID
        volatile double mIntegratorPos;                 //!> position Integrator for speed/pos PID
        volatile double mSpeed;                         //!> the intent speed to reach
        volatile double lastAngle;                      //!> last angle position with full revolution
        volatile double lastPos;                        //!> last position in 0.360 deg
        volatile double newPos;                         //!> new postion  for speed/pos PID

        int16_t PWM_U_values[arraySize];                //!> PWM predefined values for U
        int16_t PWM_V_values[arraySize];                //!> PWM predefined values for V
        int16_t PWM_W_values[arraySize];                //!> PWM predefined values for W

        void _readPWMArray(String filename);
        void _readMotorCalibration(String filename);

};

#endif //JOINTCONTROLLER_HPP