
#include <SD.h>
#include <tlx5012-arduino.hpp>
#include "const.h"

using namespace tle5012;

// switch on SD Card on XMC700 Relax Kit
const char filename_pwm[] = "pwm1.txt";           //! Filename for motor PWM array <-- change this for different motors
const char filename_cal[] = "cal1.txt";           //! Filename for motor calibration values <-- change this for different motors
const char filename_pid[] = "pid1.txt";           //! Filename for motor calibration values <-- change this for different motors
File txtFile;                                     //! File object to represent file
String buffer;                                    //! string to buffer output

// For the TLE5012

// tle5012::SPIClass3W tle5012::SPI3W1(1);           //!< SPI port 1 on XMC4700 X1 according HW SPI setup
// Tle5012Ino Tle5012SensorSPI2 = Tle5012Ino(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
tle5012::SPIClass3W tle5012::SPI3W2(2); //!< SPI port 2 on XMC4700 X2 according HW SPI setup
Tle5012Ino Tle5012SensorSPI2 = Tle5012Ino(&SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
const int U    = PIN_PWM_U_SHIELD;
const int V    = PIN_PWM_V_SHIELD;
const int W    = PIN_PWM_W_SHIELD;
const int EN_U = PIN_PWM_EN_U_SHIELD;
const int EN_V = PIN_PWM_EN_V_SHIELD;
const int EN_W = PIN_PWM_EN_W_SHIELD;


errorTypes checkError = NO_ERROR;

volatile double angle = 0.0;
volatile double last_angle = 0.0;
volatile double intent_speed = 0;
double integrator_speed = 0;
double integrator_pos = 0;

int intent_pos = 168.0;
int intent_pos_new = 0;


//! PWM array variables
int PWMresolution = 2048;
const int resolution = 10;                        //! array resolution 10 = 0.1 deg <-- change array size here
const int arraySize = 360 * resolution;           //! array size for 360 deg

// motor PID
double P_S = 1.0;
double I_S = 0.0;
double D_S = 0.0;
double P_P = 18;
double I_P = 0.0;
double D_P = 0.0;

// values to read from SD card array from pwm1.txt and offset/phaseshift from cal1.txt
int myPWM_U_values[arraySize];
int myPWM_V_values[arraySize];
int myPWM_W_values[arraySize];
int offset = 0;
int PhaseShift = 0;
int debug = 0;

extern "C" {

  void CCU42_0_IRQHandler(void) //Position control
  { 
    double error_pos = 0;

    if(abs(last_angle - angle) < 350.0)
    {
      error_pos = last_angle - angle;
    }
    last_angle = angle;  

    if(intent_pos_new < intent_pos)
    {
       intent_pos_new = intent_pos_new + 1;
    }
    if(intent_pos_new > intent_pos)
    {
       intent_pos_new = intent_pos_new - 1;
    }
    error_pos = intent_pos_new - angle;
    integrator_pos = constrain(integrator_pos + error_pos,-100, 100);
    intent_speed= error_pos * P_P + integrator_pos * I_P + error_pos * D_P; 
  }


  void CCU40_0_IRQHandler(void)
  { 
    double raw_angle = 0.0;
    int16_t revolutions = 0;
    int16_t phaseShift = PhaseShift;
    double speed = 0;

    Tle5012SensorSPI2.getAngleValue(raw_angle);
    Tle5012SensorSPI2.getNumRevolutions(revolutions);

    double angle360 = raw_angle >=0
          ? raw_angle
          : 360 + raw_angle;
    angle = revolutions * 360 + angle360;

    speed = last_angle - angle360;
    if(abs(last_angle - angle360) < 350.0)
    {
      speed = constrain ( last_angle - angle360, -0.5, 0.5 );
    }
    last_angle = angle360;
    double error_speed = intent_speed - speed;
    integrator_speed = constrain(integrator_speed + error_speed,-100, 100);
    double control_speed = error_speed * P_S + integrator_speed * I_S + speed * D_S; 



    if (control_speed < 0){phaseShift *= -1;}                                                     //! turn counterclockwise
    int16_t duty = constrain(abs(control_speed),0,2048);                                           //! Limit output to meaningful range

    int16_t angle_table = angle360 * resolution + phaseShift + offset;
    if (angle_table >= arraySize) {angle_table -=3600;} 
    if (angle_table < 0000) {angle_table +=3600;} 

    analogWrite(U,  2048 + duty * myPWM_U_values[angle_table] / PWMresolution ); 
    analogWrite(V,  2048 + duty * myPWM_V_values[angle_table] / PWMresolution ); 
    analogWrite(W,  2048 + duty * myPWM_W_values[angle_table] / PWMresolution );  

    debug++;
    if (debug>1000) {
      printDebug(angle, angle360, revolutions, duty, angle_table );
    }
  }

}


void setup() { 
  Serial.begin(115200);
  while (!Serial){};

  // set LED pin to output, used to blink when writing
  pinMode(LED_BUILTIN, OUTPUT);

  checkError = Tle5012SensorSPI2.begin();
  Tle5012SensorSPI2.resetFirmware();

  analogWriteResolution(12);
  pinMode(U, OUTPUT);  setAnalogWriteFrequency(U,20000);
  pinMode(V, OUTPUT);  setAnalogWriteFrequency(V,20000);
  pinMode(W, OUTPUT);  setAnalogWriteFrequency(W,20000);

  pinMode(EN_U, OUTPUT);  digitalWrite(EN_V, HIGH);
  pinMode(EN_V, OUTPUT);  digitalWrite(EN_V, HIGH);
  pinMode(EN_W, OUTPUT);  digitalWrite(EN_W, HIGH);

  Serial.println("init done");
  delay(1000);
  // read values from SD card
  readCalibration(filename_cal);
  readPWMArray(filename_pwm);
  Serial.println("values read");
  delay(5000);


  //Setup Interrupt settings
  XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = {0};
  pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
  pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;

  //interrupt 1
  XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
  XMC_CCU4_EnableClock(CCU40, 3);
  XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, 500); // Adjust last Value or Prescaler
  XMC_CCU4_SLICE_EnableEvent(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  XMC_CCU4_SLICE_SetInterruptNode(CCU40_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0);
  NVIC_SetPriority(CCU40_0_IRQn, 10);
  NVIC_EnableIRQ(CCU40_0_IRQn); 
  XMC_CCU4_EnableShadowTransfer(CCU40, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
  XMC_CCU4_SLICE_StartTimer(CCU40_CC43);

  // //interrupt 2
  // XMC_CCU4_Init(CCU42, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  // XMC_CCU4_SLICE_CompareInit(CCU42_CC43, &pwm_config);
  // XMC_CCU4_EnableClock(CCU42, 3);
  // XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU42_CC43, 1000); // Adjust last Value or Prescaler
  // XMC_CCU4_SLICE_EnableEvent(CCU42_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  // XMC_CCU4_SLICE_SetInterruptNode(CCU42_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0); 
  // NVIC_SetPriority(CCU42_0_IRQn, 10);
  // NVIC_EnableIRQ(CCU42_0_IRQn); 
  // XMC_CCU4_EnableShadowTransfer(CCU42, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
  // XMC_CCU4_SLICE_StartTimer(CCU42_CC43);


  delay(1000);
}

void loop() {
   
  //Setting Phaseshift
  while (Serial.available() != 0)
    {
        int input = 0;
        input = Serial.read();

        // set motor P value with p/o
        if(input == 'p') { P_S += 1; }
        if(input == 'o') { P_S -= 1; }

        // set motor I value with k/l
        if(input == 'l') { I_S += 0.1; }
        if(input == 'k') { I_S -= 0.1; }

        // set motor D value with n/m
        if(input == 'm') { D_S += 0.1; }
        if(input == 'n') { D_S -= 0.1; }

        // set motor P value with w/e
        if(input == 'e') { P_P += 1; }
        if(input == 'w') { P_P -= 1; }

        // set motor I value with i/u
        if(input == 'd') { I_P += 0.1; }
        if(input == 's') { I_P -= 0.1; }

        // set motor D value with f/d
        if(input == 'c') { D_P += 0.1; }
        if(input == 'x') { D_P -= 0.1; }

        // set motor D value with f/d
        if(input == '2') { intent_pos += 1; }
        if(input == '3') { intent_pos -= 1; }

        if(input == '6') { intent_pos += 10; }
        if(input == '4') { intent_pos -= 10; }

        if(input == '9') { intent_pos += 1000; }
        if(input == '7') { intent_pos -= 1000; }

        if(input == '0') { intent_pos = 0; }

        // w = writes the PID, border angles and default starting angle
        if (input == 'g') { writePID(filename_pid); }


    }
}


void printDebug(double angle, double angle360,int16_t revolutions,int16_t duty,int16_t angle_table)
{
  Serial.print("   P_P:"); Serial.print(P_P); 
  Serial.print("   I_P:"); Serial.print(I_P);
  Serial.print("   D_P:"); Serial.print(D_P);
  Serial.print("   P_S:"); Serial.print(P_S); 
  Serial.print("   I_S:"); Serial.print(I_S);
  Serial.print("   D_S:"); Serial.print(D_S);
  
  Serial.print("\td:"); Serial.print(duty);
  Serial.print("\ta:"); Serial.print(angle360);
  Serial.print("\tr:"); Serial.print(revolutions);
  Serial.print("\t");   Serial.print(angle);
  Serial.print("\t");   Serial.print(intent_pos);
  Serial.print("\t");   Serial.print(angle_table);

  Serial.println("");
  //Serial.print("\t");    Serial.println(intent_angle);
  debug = 0;
}


/**
 * @brief Read the PWM array from SD card into the internal arrays
 * for PWM_U/V/W
 * 
 * @param filename Filename to read from
 */
void readPWMArray(const char* filename)
{
  if (!SD.begin()) {
    Serial.println("Card failed, or not present");
    return;
  }else{
    // try to open the file for writing
    txtFile = SD.open(filename);
    if (txtFile) {
      int idx = 0;
      digitalWrite(LED_BUILTIN, HIGH);
      while (txtFile.available() && idx < arraySize) {
        myPWM_U_values[idx] = txtFile.parseInt();
        myPWM_V_values[idx] = txtFile.parseInt();
        myPWM_W_values[idx] = txtFile.parseInt();
        idx++;
      }
      txtFile.close();
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("PWM array read finished with number of values: "); Serial.println(arraySize);
      Serial.print("from filename: "); Serial.println(filename);
    }else{
      Serial.print("error opening ");
      Serial.println(filename);
    }
  }
  return;
}

/**
 * @brief Read the PWM array from SD card into the internal arrays
 * for PWM_U/V/W
 * 
 * @param filename Filename to read from
 */
void readCalibration(const char* filename)
{
  if (!SD.begin()) {
    Serial.println("Card failed, or not present");
    return;
  }else{
    // try to open the file for writing
    txtFile = SD.open(filename);
    if (txtFile) {
      digitalWrite(LED_BUILTIN, HIGH);
      offset     = txtFile.parseInt();
      PhaseShift = txtFile.parseInt();
      txtFile.close();
      digitalWrite(LED_BUILTIN, LOW);
      Serial.print("offset read: ");     Serial.println(offset);
      Serial.print("PhaseShift read: "); Serial.println(PhaseShift);
    }else{
      Serial.print("error opening ");
      Serial.println(filename);
    }
  }
  return;
}


/**
 * @brief SDcard write routine for PID values, as well as lower, upper and default angle
 * Before writing always remove older files and blink LED during write.
 *
 * @param filename filename to write the calibration data into
 */
void writePID(const char* filename)
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
      txtFile.println(P_S);
      txtFile.println(I_S);
      txtFile.println(D_S);
      txtFile.println(P_P);
      txtFile.println(I_P);
      txtFile.println(D_P);
      txtFile.println(offset);
      txtFile.println(PhaseShift);
      digitalWrite(LED_BUILTIN, LOW);
      txtFile.close();
      Serial.print("P_S: ");Serial.println(P_S);
      Serial.print("I_S: ");Serial.println(I_S);
      Serial.print("D_S: ");Serial.println(D_S);
      Serial.print("P_P: ");Serial.println(P_P);
      Serial.print("I_P: ");Serial.println(I_P);
      Serial.print("D_P: ");Serial.println(D_P);
    }else{
      Serial.print("error opening ");
      Serial.println(filename);
    }
  }
  return;
}
