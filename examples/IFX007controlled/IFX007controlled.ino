#include <tlx5012-arduino.hpp>

using namespace tle5012;

// For the TLE5012

// Pin selection for SPI2 on X2
#define PIN_SPI2_SS0 94   //! P0.12
#define PIN_SPI2_SS1 93   //! P0.15
#define PIN_SPI2_SS2 71   //! P3.14
#define PIN_SPI2_SS3 70   //! P0.14
#define PIN_SPI2_MOSI 69  //! P3.11
#define PIN_SPI2_MISO 95  //! P3.12
#define PIN_SPI2_SCK 68   //! P3.13

tle5012::SPIClass3W tle5012::SPI3W2(2);  //!< SPI port 2 on XMC4700 X2 according HW SPI setup
Tle5012Ino Tle5012SensorSPI2 = Tle5012Ino(&SPI3W2, PIN_SPI2_SS0, PIN_SPI2_MISO, PIN_SPI2_MOSI, PIN_SPI2_SCK, Tle5012Ino::TLE5012B_S0);
errorTypes checkError = NO_ERROR;

#define POLEPAIRS 4

//#define PI                (double)  3.14159265359
#define PHASE_DELAY_1 (double)2.094395102  //120°
#define PHASE_DELAY_2 (double)4.188790205  //240°

const int U = 11;
const int V = 10;
const int W = 9;
const int EN_U = 6;
const int EN_V = 5;
const int EN_W = 3;

int duty = 40;

double pwmOne = 0;
double pwmTwo = 0;
double pwmThree = 0;

double angle = 0.0;
float angle_rad = 0.0;

double offset = -2.05;
//double PhaseShift = PI * 1 / 0.12466;
double PhaseShift = 10.8;

//double PhaseShift = 10.80;
//double PhaseShift = 6.70;

volatile double mina[2] = {100,100};


extern "C" {
  void CCU40_0_IRQHandler(void) {
    Tle5012SensorSPI2.getAngleValue(angle);
    angle_rad = (angle) * ((double)2.0 * (double)PI / (double)360.0);  // Mechanical angle in RAD
    
    pwmOne   = 127 + duty * sin(angle_rad * POLEPAIRS + PhaseShift + offset);                   analogWrite(U, pwmOne);
    pwmTwo   = 127 + duty * sin(angle_rad * POLEPAIRS + PhaseShift + PHASE_DELAY_1 + offset);   analogWrite(V, pwmTwo);
    pwmThree = 127 + duty * sin(angle_rad * POLEPAIRS + PhaseShift + PHASE_DELAY_2 + offset);   analogWrite(W, pwmThree);

    // Serial.print(pwmOne);Serial.print(",");
    // Serial.print(pwmTwo);Serial.print(",");
    // Serial.print(pwmThree);Serial.print(",");
    // Serial.println(angle);
  }
}


void setup() {
  Serial.begin(115200);
  Tle5012SensorSPI2.begin();

  pinMode(U, OUTPUT); setAnalogWriteFrequency(U, 20000);
  pinMode(V, OUTPUT); setAnalogWriteFrequency(V, 20000);
  pinMode(W, OUTPUT); setAnalogWriteFrequency(W, 20000);

  pinMode(EN_U, OUTPUT); digitalWrite(EN_U, HIGH);
  pinMode(EN_V, OUTPUT); digitalWrite(EN_V, HIGH);
  pinMode(EN_W, OUTPUT); digitalWrite(EN_W, HIGH);

  //Setup Interrupt settings
  XMC_CCU4_SLICE_COMPARE_CONFIG_t pwm_config = { 0 };
  pwm_config.passive_level = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_HIGH;
  pwm_config.prescaler_initval = XMC_CCU4_SLICE_PRESCALER_128;
  //Setup interrupt1
  XMC_CCU4_Init(CCU40, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  XMC_CCU4_SLICE_CompareInit(CCU40_CC43, &pwm_config);
  XMC_CCU4_EnableClock(CCU40, 3);
  XMC_CCU4_SLICE_SetTimerPeriodMatch(CCU40_CC43, 5000);  // Adjust last Value or Prescaler
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

  delay(5000);
}

void loop() {
  // PhaseShift -= 0.1;
  // delay(10);
  // double analog = analogRead(A5);

  // Serial.print(analog);
  // Serial.print("\t");
  // Serial.print(PhaseShift);
  // if (analog < mina[0] && analog > 10) {
  //    mina[1] = PhaseShift;
  //    mina[0] = analog;
  // }
  // Serial.print("\t");
  // Serial.print(mina[1]);
  // Serial.print("\t");
  // Serial.println(mina[0]);
  // delay(50);


  // put your main code here, to run repeatedly:
  while (Serial.available() != 0) {
    int input = 0;
    input = Serial.read();
    if (input == 43) {
      PhaseShift += 0.1;
      Serial.print("\t\t");
      Serial.println(PhaseShift);
    }
    if (input == 45) {
      PhaseShift -= 0.1;
      Serial.print("\t\t");
      Serial.println(PhaseShift);
    }
  }
}