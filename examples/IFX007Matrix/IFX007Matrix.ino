
#include <SD.h>
#include <tlx5012-arduino.hpp>
#include "const.h"


using namespace tle5012;

// switch on SD Card on XMC700 Relax Kit
const char filename_pwm[] = "pwm1.txt";           //! Filename for motor PWM array <-- change this for different motors
const char filename_cal[] = "cal1.txt";           //! Filename for motor calibration values <-- change this for different motors
File txtFile;                                     //! File object to represent file
String buffer;                                    //! string to buffer output

// For the TLE5012

// Pin selection for SPI1 on X1
#define PIN_SPI1_SS0        35                         //! P0.5 X axes
#define PIN_SPI1_SS1        64                         //! P0.2 Y axes
#define PIN_SPI1_SS2        66                         //! P0.4 not used
#define PIN_SPI1_SS3        36                         //! P0.3 not used
#define PIN_SPI1_MOSI       37                         //! P0.1
#define PIN_SPI1_MISO       63                         //! P0.0
#define PIN_SPI1_SCK        38                         //! P0.10

// Pin selection for SPI2 on X2
#define PIN_SPI2_SS0        94                         //! P0.12 Z axes
#define PIN_SPI2_SS1        93                         //! P0.15 A axes
#define PIN_SPI2_SS2        92                         //! (70,P0.14) changed here to P3.3 CR axes+++++++++++++++++++++++
#define PIN_SPI2_SS3        91                         //! (71,P3.14) changed here to P0.8 CL axes
#define PIN_SPI2_MOSI       69                         //! P3.11
#define PIN_SPI2_MISO       95                         //! P3.12
#define PIN_SPI2_SCK        68                         //! P3.13

// Shield on top test
tle5012::SPIClass3W tle5012::SPI3W1(1);         //!< SPI port 1 on XMC4700 X1 according HW SPI setup
tle5012::SPIClass3W tle5012::SPI3W2(2);
Tle5012Ino Tle5012Sensor = Tle5012Ino(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
// Tle5012Ino Tle5012Sensor = Tle5012Ino(&SPI3W2, PIN_SPI2_SS3, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
// const int U    = PIN_PWM_U_SHIELD;
// const int V    = PIN_PWM_V_SHIELD;
// const int W    = PIN_PWM_W_SHIELD;
// const int EN_U = PIN_PWM_EN_U_SHIELD;
// const int EN_V = PIN_PWM_EN_V_SHIELD;
// const int EN_W = PIN_PWM_EN_W_SHIELD;

const int U = 11;
const int V = 10;
const int W = 9;
const int EN_U = 8;
const int EN_V = 8;
const int EN_W = 8;



errorTypes checkError = NO_ERROR;
int i = 0;
int duty = 100;                                   //! duty cycle 0-127


// motor settings
#define POLEPAIRS 4                               //! Number of pole pairs of the motor (number of poles / 2 )
#define PHASE_DELAY_1 (double)2.094395102         // 120° offset
#define PHASE_DELAY_2 (double)4.188790205         // 240° offset

int pwmOne = 0;
int pwmTwo = 0;
int pwmThree = 0;

double amplitudeOne;
double amplitudeTwo;
double amplitudeThree;

double angle = 0.0;                               //! corrected angle from sensor as 0-360 deg
double angle_raw = 0.0;                           //! raw value from sensor as -180-180 deg
float angle_rad = 0.0;                            //! angle in radians

int offset = 0;                                   //! this value has to found <-- start here always with 0
int PhaseShift = 0;                               //! this value can be variated

//! PWM array variables
int n = 0;                                        //! counter for the array
const int resolution = 10;                        //! array resolution 10 = 0.1 deg <-- change array size here
const int arraySize = 360 * resolution;           //! array size for 360 deg

int myPWM_U_values[arraySize];
int myPWM_V_values[arraySize];
int myPWM_W_values[arraySize];

extern "C"
{
  void CCU40_0_IRQHandler(void)
  {
    // TIER1: run motor uncontrolled
    if (millis() < 20000)
    {
      angle_rad += 0.0174532925;

      pwmOne   = 2048 * sin(angle_rad);
      pwmTwo   = 2048 * sin(angle_rad + PHASE_DELAY_1);
      pwmThree = 2048 * sin(angle_rad + PHASE_DELAY_2);
      analogWrite(U, 2048 + duty * pwmOne   / 2048);
      analogWrite(V, 2048 + duty * pwmTwo   / 2048);
      analogWrite(W, 2048 + duty * pwmThree / 2048);

      Tle5012Sensor.getAngleValue(angle_raw);
      double angle = angle_raw >=0     //! this will synchronize real ange to intended angle
          ? angle_raw
          : 360 + angle_raw;

      int angle_out = angle * resolution;         // 0-3600
      myPWM_U_values[angle_out] = pwmOne;
      myPWM_V_values[angle_out] = pwmTwo;
      myPWM_W_values[angle_out] = pwmThree;
    }

    // TIER2: print out PWM_U/V/W
    if (millis() > 20005 && n < arraySize)
    {
      Serial.print(n); Serial.print(", ");
      Serial.print(myPWM_U_values[n]); Serial.print(", ");
      Serial.print(myPWM_V_values[n]); Serial.print(", ");
      Serial.print(myPWM_W_values[n]); Serial.println();

      n++;
    }

    // TIER3: run motor controlled
    if (millis() > 25000)
    {
      Tle5012Sensor.getAngleValue(angle_raw);
      double angle = angle_raw >=0     //! this will synchronize real ange to intended angle
          ? angle_raw
          : 360 + angle_raw;

      int angle_table = angle * resolution + PhaseShift + offset;
      if (angle_table >= arraySize )
      {
        angle_table -= arraySize;
      }
      if (angle_table < 0000)
      {
        angle_table += arraySize;
      }

      analogWrite(U, 2048 + duty * myPWM_U_values[angle_table] / 2048 );
      analogWrite(V, 2048 + duty * myPWM_V_values[angle_table] / 2048 );
      analogWrite(W, 2048 + duty * myPWM_W_values[angle_table] / 2048 );
    }
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial){};
  // Init SPI channel for TLE5012
  checkError = Tle5012Sensor.begin();

  // reserve 1kB for String used as a buffer
  buffer.reserve(2048);
  // set LED pin to output, used to blink when writing
  pinMode(LED_BUILTIN, OUTPUT);

  // Init GPIO for IFX007, 20kHz does not produce any hearable sound
  analogWriteResolution(12);
  pinMode(U, OUTPUT); setAnalogWriteFrequency(U, 20000);
  pinMode(V, OUTPUT); setAnalogWriteFrequency(V, 20000);
  pinMode(W, OUTPUT); setAnalogWriteFrequency(W, 20000);
  pinMode(EN_U, OUTPUT); digitalWrite(EN_U, HIGH);
  pinMode(EN_V, OUTPUT); digitalWrite(EN_V, HIGH);
  pinMode(EN_W, OUTPUT); digitalWrite(EN_W, HIGH);


  delay (5000);

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


}

void loop()
{
    // put your main code here, to run repeatedly:
  while (Serial.available() != 0)
  {
    int input = 0;
    input = Serial.read();

    // + sign raises the PhaseShift
    if (input == 43)
    {
      PhaseShift += 1;
      Serial.print("\t"); Serial.println(PhaseShift);
    }

    // - sign lowers the PhaseShift
    if (input == 45)
    {
      PhaseShift -= 1;
      Serial.print("\t"); Serial.println(PhaseShift);
    }

    // o = set offset
    if (input == 111)
    {
      offset = PhaseShift;
      PhaseShift = 0;
      Serial.print("Offset\t"); Serial.println(offset);
    }

        // set motor D value with f/d
        if(input == 102)
        {
            duty+=1;
            Serial.print("duty\t"); Serial.println(duty);
        }
        if(input == 100)
        {
            duty-=1;
            Serial.print("duty\t"); Serial.println(duty);
        }

    // w = writes the PWM array and calibration values
    if (input == 119)
    {
      writePWMArray(filename_pwm);
      writeCalibration(filename_cal);
    }

    // c = write calibration values
    if (input == 99)
    {
      writeCalibration(filename_cal);
    }

  }
}


/**
 * @brief SDcard write routine prints out the PWM array into a text file.
 * Writes data in chunks. Before writing always remove older files and
 * blink LED during write.
 *
 * @param filename filename to write the PWM array into
 */
void writePWMArray(const char* filename)
{
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
        buffer += myPWM_U_values[a]; buffer += ",";
        buffer += myPWM_V_values[a]; buffer += ",";
        buffer += myPWM_W_values[a]; buffer += "\r\n";
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
void writeCalibration(const char* filename)
{
  // init the SD card
  if (!SD.begin()) {
    Serial.println("Card failed, or not present");
    return;
  }else{
    SD.remove(filename);
    txtFile = SD.open(filename, FILE_WRITE);
    if (txtFile) {
      digitalWrite(LED_BUILTIN, HIGH);
      txtFile.println(offset);
      txtFile.println(PhaseShift);
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("Offset\t");      Serial.println(offset);
      Serial.print("PhaseShift\t");  Serial.println(PhaseShift);
      Serial.print("to filename: "); Serial.println(filename);
      txtFile.close();
    }else{
      Serial.print("error opening ");
      Serial.println(filename);
    }
  }
  return;
}
