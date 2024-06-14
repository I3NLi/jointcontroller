// #define PI              (double)  3.14159265359
#define PHASE_DELAY_1 (double)2.094395102 // 120° phase shift
#define PHASE_DELAY_2 (double)4.188790205 // 240° phase shift

#define PIN_PWM_U2          3                           //! P1.1
#define PIN_PWM_V2          5                           //! P2.12
#define PIN_PWM_W2          6                           //! P2.11
#define PIN_PWM_EN2         72                          //! P0.7


// const int U = 11;
// const int V = 10;
// const int W = 9;
// const int EN_U = 6;
// const int EN_V = 5;
// const int EN_W = 3;

const int U = 11;
const int V = 10;
const int W = 9;
const int EN_U = 8;
const int EN_V = 8;
const int EN_W = 8;


double i = 0;
int dutycyle = 40; // 0-127

double pwmOne = 0;
double pwmTwo = 0;
double pwmThree = 0;

void setup()
{
    pinMode(U, OUTPUT); setAnalogWriteFrequency(U, 20000);
    pinMode(V, OUTPUT); setAnalogWriteFrequency(V, 20000);
    pinMode(W, OUTPUT); setAnalogWriteFrequency(W, 20000);

    pinMode(EN_U, OUTPUT); digitalWrite(EN_U, HIGH);
    pinMode(EN_V, OUTPUT); digitalWrite(EN_V, HIGH);
    pinMode(EN_W, OUTPUT); digitalWrite(EN_W, HIGH);
}

void loop()
{
    pwmOne = 127 + dutycyle * sin(i);
    pwmTwo = 127 + dutycyle * sin(i + PHASE_DELAY_1);
    pwmThree = 127 + dutycyle * sin(i + PHASE_DELAY_2);

    analogWrite(U, pwmOne);
    analogWrite(V, pwmTwo);
    analogWrite(W, pwmThree);
    i += 0.1;
    // Serial.print(pwmOne);Serial.print(",");
    // Serial.print(pwmTwo);Serial.print(",");
    // Serial.println(pwmThree);
    
    delay(1);
}