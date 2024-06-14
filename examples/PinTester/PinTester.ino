/*!
 */

#include "Arduino.h"

int8_t pin = 0;
int8_t lowPin = 0;
int8_t highPin = 95;


boolean isGPIO = true;
boolean isPWM = false;
boolean isON = false;




/**
 * @brief checks status of XMC4700 Button 1
 *
 */
void checkButton1(void)
{
    if (!isGPIO)
    {
        Serial.println("Check GPIO output");

        isGPIO = true;
        isPWM = false;
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
        pinMode(pin,OUTPUT);
        analogWrite(pin,0);
        digitalWrite(pin,LOW);
    }
}

/**
 * @brief checks status of XMC4700 Button 1
 *
 */
void checkButton2(void)
{
    if (!isPWM)
    {
        Serial.println("Check PWM output");
        isGPIO = false;
        isPWM = true;
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);
        pinMode(pin,OUTPUT);
        analogWrite(pin,0);
        digitalWrite(pin,LOW);
    }
}

/**
 * @brief Arduino setup function
 * 
 */
void setup()
{
    // Serial port communication
    Serial.begin(115200);
    while (!Serial) {}
    delay(10000);

    // set LED pin to output, used to blink when writing
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(BUTTON1, INPUT);
    pinMode(BUTTON2, INPUT);
    analogWriteResolution(12);

    Serial.println("\nPinTester begin setup\n");

}


/**
 * @brief
 *
 */
void loop()
{
    if (digitalRead(BUTTON2) == LOW)
    {
        checkButton2();
    }

    if (digitalRead(BUTTON1) == LOW)
    {
        checkButton1();
    }


    while (Serial.available() != 0)
    {
        int input = 0;
        input = Serial.read();


        if (input == '+')
        {
            digitalWrite(pin, LOW);
            pin += 1;
            if (pin==32 || pin ==33)
                pin = 34;
            pin = constrain(pin,lowPin,highPin);

            pinMode(pin, OUTPUT);
            Serial.println(pin);
        }

        if (input == '-')
        {
            digitalWrite(pin, LOW);
            pin -= 1;
            if (pin==32 || pin ==33)
                pin = 31;
            pin = constrain(pin,lowPin,highPin);
            pinMode(pin, OUTPUT);
            Serial.println(pin);
        }


        // check on GPIO output
        if (isGPIO)
        {
            if (input == '1')
            {
                digitalWrite(pin, HIGH);
                Serial.print(pin);
                Serial.println("  DIGITAL High");

            }
            if (input == '3')
            {
                digitalWrite(pin, LOW);
                Serial.print(pin);
                Serial.println("  DIGITAL Low");
            }
        }

        // check on PWM output
        if (isPWM)
        {
            if (input == '1')
            {
                setAnalogWriteFrequency(pin, 20000);
                analogWrite(pin,1024);
                Serial.print(pin);
                Serial.println("  PWM 1024");
            }
            if (input == '2')
            {
                setAnalogWriteFrequency(pin, 20000);
                analogWrite(pin,2048);
                Serial.print(pin);
                Serial.println("  PWM 2048");
            }
            if (input == '3')
            {
                setAnalogWriteFrequency(pin, 20000);
                analogWrite(pin,4095);
                Serial.print(pin);
                Serial.println("  PWM 4096");
            }

        }

    }

    delay(100);
}

