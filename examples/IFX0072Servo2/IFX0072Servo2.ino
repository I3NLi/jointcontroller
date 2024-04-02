
#include <SD.h>
#include <tlx5012-arduino.hpp>

using namespace tle5012;

// switch on SD Card on XMC700 Relax Kit
const char filename_pwm[] = "PWMY:TXT";           //! Filename for motor PWM array <-- change this for different motors
const char filename_cal[] = "CALX.TXT";           //! Filename for motor calibration values <-- change this for different motors
const char filename_pid[] = "pidx.txt";           //! Filename for motor calibration values <-- change this for different motors
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
#define PIN_SPI2_SS2        92                         //! (70,P0.14) changed here to P3.3 CR axes
#define PIN_SPI2_SS3        91                         //! (71,P3.14) changed here to P0.8 CL axes
#define PIN_SPI2_MOSI       69                         //! P3.11
#define PIN_SPI2_MISO       95                         //! P3.12
#define PIN_SPI2_SCK        68                         //! P3.13


#define Regulierungsfaktor 0.5

 tle5012::SPIClass3W tle5012::SPI3W1(1);           //!< SPI port 1 on XMC4700 X1 according HW SPI setup
 Tle5012Ino Tle5012SensorSPI2 = Tle5012Ino(&SPI3W1, PIN_SPI1_SS1, PIN_SPI1_MISO, PIN_SPI1_MOSI, PIN_SPI1_SCK, Tle5012Ino::TLE5012B_S1);

errorTypes checkError = NO_ERROR;

#define POLEPAIRS 4

#define PHASE_DELAY_1     (double)  2.094395102 //120°
#define PHASE_DELAY_2     (double)  4.188790205 //240°

const int U = 3;
const int V = 5;
const int W = 6;
const int EN_U = 72;
const int EN_V = 72;
const int EN_W = 72;

int duty = 40;

volatile double angle = 0.0;
volatile double last_angle = 0.0;
volatile double intent_angle = 168.0; //(upright position)
double integrator = 0;       //for PID - I controller

double speed;
float speed_init = 0.003;
double error = 0.0;
double control = 0.0; 

double lowerBorderAngle = 0.0;
double upperBorderAngle = 0.0;
double defaultAngle = 0.0;


//! PWM array variables
const int resolution = 10;                        //! array resolution 10 = 0.1 deg <-- change array size here
const int arraySize = 360 * resolution;           //! array size for 360 deg

// motor PID
double P_S = 0.03;                                     //! initial P value can be changed with p = +0.01, o = -0.01
double I_S = 0.05;                                     //! initial I value can be changed with i = +0.01, u = -0.01
double D_S = 0;                                     //! initial D value can be changed with f = +1, d = -1
double Merken=0;
// values to read from SD card array from pwm1.txt and offset/phaseshift from cal1.txt
int myPWM_U_values[arraySize];
int myPWM_V_values[arraySize];
int myPWM_W_values[arraySize];
int offset = 0;
int PhaseShift = 0;
int debug = 0;
double integrator_speed=0;
double intent_speed=1;
double error_speed=0;
double control_speed=0;
double raw_angle = 0.0;   

double integrator_pos=0;
double error_pos=0;
double speed_pos=0;
double last_pos=0;
int intent_pos=0;
double P_P=18;             //! initial P value can be changed with "1" = +1, "2" = -1
double I_P=4.27;             //! initial I value can be changed with "3" = +0.01, "4" = -0.01
double D_P=-0.48;              //! initial D value can be changed with "5" = +1, "6" = -1
int intent_pos_new=0;
int debug2 =0;
 
extern "C" {
 void CCU42_0_IRQHandler(void) //Position control
  { 
    if(abs(last_pos-angle)<350.0)
    {
    error_pos = last_pos-angle;                      //! calculate speed

    }
    last_pos = angle;  
   // if(debug2%2==0)
  //  {
      if(intent_pos_new<intent_pos)
      {
       intent_pos_new=intent_pos_new+1;
      }
      if(intent_pos_new>intent_pos)
      {
       intent_pos_new=intent_pos_new-1;
      }
     // debug2=1;
   // }
    error_pos = intent_pos_new-angle;
    //d_pos=(error_pos-pre_error);  //für D stimmt etwas nicht
    integrator_pos = constrain(integrator_pos + error_pos,-100, 100);
    intent_speed= error_pos * P_P + integrator_pos * I_P + error_pos * D_P; 
    //pre_error=error_pos;
    //debug2++;

  }
  double merken[1000000];
  void CCU40_0_IRQHandler(void) //Speed control
  { 
                        
    int16_t revolutions = 0;                       
    
    
    Tle5012SensorSPI2.getAngleValue(raw_angle);          //! raw angle value from -180deg to 180deg
    Tle5012SensorSPI2.getNumRevolutions(revolutions);     //! number of revolutions

    double angle360 = raw_angle >=0     //! this will synchronize real ange to intended angle
          ? raw_angle
          : 360 + raw_angle;

    
    angle = revolutions * 360 + angle360;
    
   if(abs(last_angle-angle360)<350.0)
    {
    speed = last_angle-angle360;                      //! calculate speed
    speed=constrain (speed, -0.5, 0.5);
    }
    last_angle = angle360;  
   // intent_speed=constrain (speed, -0.5, 0.5);
    error_speed = intent_speed-speed;
    //peed_d=error_speed-speed_pre;
    integrator_speed = constrain(integrator_speed + error_speed,-100, 100);
    control_speed = error_speed * P_S + integrator_speed * I_S + speed * D_S; 
    //speed_pre=error_speed;
    if (control_speed >= 0){PhaseShift = abs(PhaseShift)*1;}   //turn clockwise
    if (control_speed < 0){PhaseShift = abs(PhaseShift) * -1;}   //turn counterclockwise
    
    duty = constrain(abs(control_speed),0,255); //Limit output to meaningful range
    
    int angle_table = angle360*10+PhaseShift+offset;
   /* if (angle_table >= 295 && angle_table <= 300) {
    angle_table=angle_table+20;
    }*/
    if (angle_table >= arraySize) {angle_table -=3600;} 
    if (angle_table < 0000) {angle_table +=3600;} 

    int pwmOne   = myPWM_U_values[angle_table];
    int pwmTwo   = myPWM_V_values[angle_table];
    int pwmThree = myPWM_W_values[angle_table];

    analogWrite(U, duty*pwmOne   / 100); 
    analogWrite(V, duty*pwmTwo   / 100); 
    analogWrite(W, duty*pwmThree / 100);  
    
    
    /*if(button==1)
    {
    merken[debug]=last_pos;
     }*/
      while (Serial.available() != 0)
        {
        int input = 0;
        input = Serial.read();
        if(input == 'p')
        {
            P_S+=0.01;
        }
        if(input == 'o')
        {
            P_S-=0.01; 
        }
        if (input=='i')
        {
          I_S+=0.01;
        }

        if (input=='u')
        {
          I_S-=0.01;
        }
        if (input=='d')
        {
          D_S-=0.01;
        }
        
        if (input=='f')
        {
          D_S+=0.01;
        }
      
        if(input == '1')
        {
            P_P+=1;
        }
        if(input == '2')
        {
            P_P-=1; 
        }
        if (input=='3')
        {
          I_P+=0.01;
        }

        if (input=='4')
        {
          I_P-=0.01;
        }
        if (input=='5')
        {
          D_P-=0.01;
        }
        
        if (input=='6')
        {
          D_P+=0.01;
        }

        if(input=='+')
        {
          intent_pos+=1;
        }

        if(input=='-')
        {
          intent_pos-=1;
        }

        if(input=='7')
        {
          intent_pos+=10;
        }

        if(input=='8')
        {
          intent_pos-=10;
        }
       /* int hallo=digitalRead(25)==HIGH;
        if(digitalRead(25)==HIGH)
  	    {
          digitalWrite(LED_BUILTIN,hallo);
        }*/
        
    }


    debug++;
    if (debug>2000) {
     printDebug();
    }
  }
}


void setup() { 
  pinMode(25, INPUT);
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


  //Setup Interrupt settings 1
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
    NVIC_SetPriority(CCU40_0_IRQn, 5);
    /* Enable IRQ */
    NVIC_EnableIRQ(CCU40_0_IRQn); 
    XMC_CCU4_EnableShadowTransfer(CCU40, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
    XMC_CCU4_SLICE_StartTimer(CCU40_CC43);

  //interrupt 2
    XMC_CCU4_Init(CCU42, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
    XMC_CCU4_SLICE_CompareInit(CCU42_CC43, &pwm_config);
    XMC_CCU4_EnableClock(CCU42, 3);
    XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU42_CC43, 1000); // Adjust last Value or Prescaler
    /* Enable compare match and period match events */
    XMC_CCU4_SLICE_EnableEvent(CCU42_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
    /* Connect period match event to SR0 */
    XMC_CCU4_SLICE_SetInterruptNode(CCU42_CC43, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, XMC_CCU4_SLICE_SR_ID_0); 
    /* Configure NVIC */
    /* Set priority */
    NVIC_SetPriority(CCU42_0_IRQn, 10);
    /* Enable IRQ */
    NVIC_EnableIRQ(CCU42_0_IRQn); 
    XMC_CCU4_EnableShadowTransfer(CCU42, (CCU4_GCSS_S0SE_Msk << (4 * 3)));
    XMC_CCU4_SLICE_StartTimer(CCU42_CC43);

  delay(1000);
}

void loop() {
   
   
    //this is commentent out, because the interrupts will destroy the serial inputs in the loop


  //Setting Phaseshift
  /*while (Serial.available() != 0)
    {
        int input = 0;
        input = Serial.read();

        if(input == ',')
        {
          	intent_speed=intent_speed+0.5;
        }

        
        if(input == '.')
        {
          	intent_speed=intent_speed-0.5;
        }
        // run back to default start angle
        if(input == 48)
        {
            intent_angle = defaultAngle;
        }

        // Set intended angle with +/-
        if(input == 55)
        {
          int i=0;
          for(i=0;i<=5;i++){
        intent_angle+=2;
        delay(5);
          }
            
           
        }
        if(input == 56)
        {
            intent_angle-=10; 
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
            P_S+=1;
        }
        if(input == 111)
        {
            P_S-=1; 
        }

        // set motor D value with f/d
        if(input == 102)
        {
            D_S+=1;
        }
        if(input == 100)
        {
            D_S-=1;
        }

        // set motor I value with i/u
        if(input == 105)
        {
            I_S+=0.01;
        }
        if(input == 117)
        {
            I_S-=0.01;
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
        */
        // w = writes the PID, border angles and default starting angle
      /*  if (input == 119)
        {
            writePID(filename_pid);
        }
      */

  //  }
}


void printDebug()
{

  
  Serial.print(P_P); Serial.print("\t");
  Serial.print(I_P); Serial.print("\t");
  Serial.print(D_P); Serial.print("\t");
  Serial.print(error_pos*P_P); Serial.print("\t");
  Serial.print(integrator_pos * I_P); Serial.print("\t");
  Serial.print(speed * D_P); Serial.print("\t");
  Serial.print(intent_pos); Serial.print("\t");

  Serial.print(P_S); Serial.print("\t");
  Serial.print(I_S); Serial.print("\t");
  Serial.print(D_S); Serial.print("\t");
  Serial.print(error_speed*P_S); Serial.print("\t");
  Serial.print(integrator_speed * I_S); Serial.print("\t");
  Serial.print(speed * D_S); Serial.print("\t");
  Serial.print(control_speed); Serial.print("\t");


  Serial.print(duty); Serial.print("\t");
  Serial.print(speed); Serial.print("\t");
  Serial.print(raw_angle); Serial.print("\t");
  Serial.println(angle);
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
/*
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
}*/
