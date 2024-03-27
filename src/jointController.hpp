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

#define PWMFrequency   20000                                //! 20kHz PHM frequency so that motor does not fiepe

const int resolution = 10;                                  //!> array resolution 10 = 0.1 deg <-- change array size here
const int arraySize  = 360 * resolution;                    //!> array size for 360 deg

class jointController
{
    public:
        char jointName;                                     //! name of the joint
        Tle5012Ino *sensor;                                 //!> pointer to the tlX501 sensor
        typedef struct pidParm {
            double P;                                       //!> motor P value
            double I;                                       //!> motor integrator value
            double D;                                       //!> motor derivative value
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
        } motorSetup_t;

        jointController(char *name);
        ~jointController();

        void begin();

        errorTypes initSensor(SPIClass3W &bus, uint8_t csPin, uint8_t misoPin, uint8_t mosiPin, uint8_t sckPin, Tle5012Ino::slaveNum slave);
        errorTypes initShield(uint8_t U,uint8_t V,uint8_t W,uint8_t EN_U,uint8_t EN_V,uint8_t EN_W);

        void setPID(double P, double I, double D);
        void setRangeLimits(double minLimit=0.0, double maxLimit=360.0, double startPos=0.0);
        void setPWMResolution(int16_t resolution=2048);
        void setMotorCal(int16_t off,int16_t phase);
        void setGearFactor(double gearFactor=1.0);
        void setHomingPosition(double startPos=0.0);
        void readFromSD(const char* filename);

        double angleInsideRangeLimits(double rawAngle);
        double calculateAngle(double gf=1.0);

        void runToAngle(double target_angle);


    private:

        File txtFile;                                   //! File object to represent file
        String buffer;                                  //! string to buffer output

        errorTypes sensorError = NO_ERROR;              //!> error type for sensors
        errorTypes shieldError = NO_ERROR;              //!> error type for shields

        uint8_t pin_U;                                  //!>
        uint8_t pin_V;                                  //!>
        uint8_t pin_W;                                  //!>
        uint8_t pin_EN_U;                               //!>
        uint8_t pin_EN_V;                               //!>
        uint8_t pin_EN_W;                               //!>

        motorSetup_t    mMotor;                         //!> motor setup values structure
        posLimits_t     mLimits;                        //!> limits structure
        pidParam_t      mPid;                           //!> PID values structure
        double          mGearFactor;                    //!> gear factor setting
        double          mIntegrator;                    //!> this will sum all the errors for i part of pid controller
        double          mPWMResolution;                 //!> bit resolution of the analog pins
        double          lastAngle;                      //!> last Position for pid controller
        double          intentAngle;                    //!> last Position for pid controller

        int16_t PWM_U_values[arraySize];                //!> PWM predefined values for U
        int16_t PWM_V_values[arraySize];                //!> PWM predefined values for V
        int16_t PWM_W_values[arraySize];                //!> PWM predefined values for W

        void _readPWMArray(String filename);
        void _readMotorCalibration(String filename);

};

#endif //JOINTCONTROLLER_HPP