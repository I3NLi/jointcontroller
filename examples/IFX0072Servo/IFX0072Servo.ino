
#include <SD.h>
#include <tlx5012-arduino.hpp>

using namespace tle5012;

// switch on SD Card on XMC700 Relax Kit
const char filename_pwm[] = "pwm1.txt";           //! Filename for motor PWM array <-- change this for different motors
const char filename_cal[] = "cal1.txt";           //! Filename for motor calibration values <-- change this for different motors
const char filename_pid[] = "pid1.txt";           //! Filename for motor calibration values <-- change this for different motors
File txtFile;                                     //! File object to represent file
String buffer;                                    //! string to buffer output

// For the TLE5012

// Pin selection for SPI1 on X1
#define PIN_SPI1_SS0   36                         //! P0.3
#define PIN_SPI1_SS1   64                         //! P0.2
#define PIN_SPI1_SS2   66                         //! P0.4
#define PIN_SPI1_SS3   35                         //! P0.5
#define PIN_SPI1_MOSI  37                         //! P0.1
#define PIN_SPI1_MISO  63                         //! P0.0
#define PIN_SPI1_SCK   38                         //! P0.10
// Pin selection for SPI2 on X2
#define PIN_SPI2_SS0  94                          //! P0.12
#define PIN_SPI2_SS1  93                          //! P0.15
#define PIN_SPI2_SS2  71                          //! P3.14
#define PIN_SPI2_SS3  70                          //! P0.14
#define PIN_SPI2_MOSI 69                          //! P3.11
#define PIN_SPI2_MISO 95                          //! P3.12
#define PIN_SPI2_SCK  68                          //! P3.13

// tle5012::SPIClass3W tle5012::SPI3W1(1);           //!< SPI port 1 on XMC4700 X1 according HW SPI setup
// Tle5012Ino Tle5012SensorSPI2 = Tle5012Ino(&SPI3W1, PIN_SPI1_SS0, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S0);
tle5012::SPIClass3W tle5012::SPI3W2(2); //!< SPI port 2 on XMC4700 X2 according HW SPI setup
Tle5012Ino Tle5012SensorSPI2 = Tle5012Ino(&SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
errorTypes checkError = NO_ERROR;

#define POLEPAIRS 4

#define PHASE_DELAY_1     (double)  2.094395102 //120°
#define PHASE_DELAY_2     (double)  4.188790205 //240°

const int U = 11;
const int V = 10; 
const int W = 9;
const int EN_U = 6;
const int EN_V = 5;
const int EN_W = 3;

volatile double last_angle = 0.0;
volatile double intent_angle = 168.0; //(upright position)
double integrator = 0;       //for PID - I controller

double lowerBorderAngle = 0.0;
double upperBorderAngle = 0.0;
double defaultAngle = 0.0;


//! PWM array variables
const int resolution = 10;                        //! array resolution 10 = 0.1 deg <-- change array size here
const int arraySize = 360 * resolution;           //! array size for 360 deg

// motor PID
int    P = 8;                                     //! initial P value can be changed with p = +1, o = -1
double I = 0;                                     //! initial I value can be changed with i = +0.01, u = -0.01
double D = 0;                                     //! initial D value can be changed with f = +1, d = -1

// values to read from SD card array from pwm1.txt and offset/phaseshift from cal1.txt
int myPWM_U_values[arraySize];
int myPWM_V_values[arraySize];
int myPWM_W_values[arraySize];
int offset = 0;
int PhaseShift = 0;
int debug = 0;

extern "C" {
  void CCU40_0_IRQHandler(void)
  { 
    double raw_angle = 0.0;
    int16_t revolutions = 0;
    int16_t phaseShift = PhaseShift;

    Tle5012SensorSPI2.getAngleValue(raw_angle);
    Tle5012SensorSPI2.getNumRevolutions(revolutions);
    double angle360 = raw_angle >=0     //! this will synchronize real ange to intended angle
          ? raw_angle
          : 360 + raw_angle;
    double angle = revolutions * 360 + angle360;

    double error = intent_angle - angle;
    double speed = last_angle - angle;                      //! calculate speed
    last_angle = angle;                            //! update "last angle"

    integrator = constrain(integrator + error,-100, 100);
    double control = error * P + integrator * I + speed * D; 

    if (control < 0){phaseShift *= -1;}                                                     //! turn counterclockwise
    int16_t duty = constrain(abs(control),0,255);                                           //! Limit output to meaningful range

    int16_t angle_table = angle360 * resolution + phaseShift + offset;
    if (angle_table >= arraySize) {angle_table -=3600;} 
    if (angle_table < 0000) {angle_table +=3600;} 

    analogWrite(U, duty * myPWM_U_values[angle_table] / 100); 
    analogWrite(V, duty * myPWM_V_values[angle_table] / 100); 
    analogWrite(W, duty * myPWM_W_values[angle_table] / 100);  

    debug++;
    if (debug>1000) {
      printDebug(angle, error, speed, control, revolutions, duty);
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

  pinMode(U, OUTPUT);  setAnalogWriteFrequency(U,20000);
  pinMode(V, OUTPUT);  setAnalogWriteFrequency(V,20000);
  pinMode(W, OUTPUT);  setAnalogWriteFrequency(W,20000);

  pinMode(EN_U, OUTPUT);  digitalWrite(EN_U, HIGH);
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

  delay(1000);
}

void loop() {
   
  //Setting Phaseshift
  while (Serial.available() != 0)
    {
        int input = 0;
        input = Serial.read();

        // run back to default start angle
        if(input == 48)
        {
            intent_angle = defaultAngle;
        }

        // Set intended angle with +/-
        if(input == 55)
        {
            intent_angle+=10000;
        }
        if(input == 56)
        {
            intent_angle-=10000; 
        }

        // Set intended angle with +/-
        if(input == 43)
        {
            intent_angle+=1;
        }
        if(input == 45)
        {
            intent_angle-=1; 
        }

        // set motor P value with p/o
        if(input == 112)
        {
            P+=1;
        }
        if(input == 111)
        {
            P-=1; 
        }

        // set motor D value with f/d
        if(input == 102)
        {
            D+=1;
        }
        if(input == 100)
        {
            D-=1;
        }

        // set motor I value with i/u
        if(input == 105)
        {
            I+=0.01;
        }
        if(input == 117)
        {
            I-=0.01;
        }

        // set motor offset
        if(input == 115)
        {
            offset+=1;
        }
        if(input == 97)
        {
            offset-=1;
        }

        // set motor offset
        if(input == 119)
        {
            PhaseShift+=1;
        }
        if(input == 113)
        {
            PhaseShift-=1;
        }

        // set actual intent angle as lower border angle
        if(input == 52)
        {
            lowerBorderAngle = intent_angle;
        }
        // set actual intent angle as default start angle
        if(input == 53)
        {
            defaultAngle = intent_angle;
        }
        // set actual intent angle as upper border angle
        if(input == 54)
        {
            upperBorderAngle = intent_angle;
        }

        // w = writes the PID, border angles and default starting angle
        if (input == 101)
        {
            writePID(filename_pid);
        }


    }
}


void printDebug(double angle, double error,double speed,double control,int16_t revolutions,int16_t duty)
{
  Serial.print("\tP: "); Serial.print(P); 
  Serial.print("\tI: "); Serial.print(I);
  Serial.print("\tD: "); Serial.print(D);
  Serial.print("\te: "); Serial.print(error*P);
  Serial.print("\ti: "); Serial.print(integrator * I);
  Serial.print("\ts: "); Serial.print(speed * D);
  Serial.print("\tc: "); Serial.print(control);
  Serial.print("\td: "); Serial.print(duty);
  Serial.print("\tr: "); Serial.print(revolutions);
  Serial.print("\t");    Serial.print(angle);
  Serial.print("\t");    Serial.println(intent_angle);
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
      offset = txtFile.parseInt();
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
      txtFile.println(P);
      txtFile.println(I);
      txtFile.println(D);
      txtFile.println(lowerBorderAngle);
      txtFile.println(upperBorderAngle);
      txtFile.println(defaultAngle);
      digitalWrite(LED_BUILTIN, LOW);
      txtFile.close();
      Serial.print("P: ");Serial.println(P);
      Serial.print("I: ");Serial.println(I);
      Serial.print("D: ");Serial.println(D);
      Serial.print("lower border angle: ");Serial.println(lowerBorderAngle);
      Serial.print("upper border angle: ");Serial.println(upperBorderAngle);
      Serial.print("default start angle:");Serial.println(defaultAngle);
    }else{
      Serial.print("error opening ");
      Serial.println(filename);
    }
  }
  return;
}
