#include <Arduino.h>


//==============warning led detection pins===============
uint8_t LEDR = 4;
uint8_t LEDG = 16;
uint8_t LEDB = 17;
uint8_t LEDW = 2;


void setupWarningLed()
{
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(LEDW, OUTPUT);

    digitalWrite(LEDW, HIGH);
}

//------------------
// white = 0
// red = 1
// blue = 2
// green = 3
// yellow = 4
// off = 5
//------------------

uint8_t getActualWarningColor()
{
    if (digitalRead(LEDR) == LOW && digitalRead(LEDG) == LOW && digitalRead(LEDB) == LOW)
    {//white
        return 0;
    }
    else if (digitalRead(LEDR) == LOW && digitalRead(LEDG) == HIGH && digitalRead(LEDB) == HIGH)
    {//red
        return 1;
    }
    else if (digitalRead(LEDR) == HIGH && digitalRead(LEDG) == HIGH && digitalRead(LEDB) == LOW)
    {//blue
        return 2;
    }
    else if (digitalRead(LEDR) == HIGH && digitalRead(LEDG) == LOW && digitalRead(LEDB) == HIGH)
    {//green
        return 3;
    }
    else if (digitalRead(LEDR) == LOW && digitalRead(LEDG) == LOW && digitalRead(LEDB) == HIGH)
    {//yellow
        return 4;
    }
    else if (digitalRead(LEDR) == HIGH && digitalRead(LEDG) == HIGH && digitalRead(LEDB) == HIGH)
    {//off
        return 5;
    }
    else
    {
        return 5;
    }
}


void warningLed(uint8_t color)
{
    if (color == 0)
    { // white
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, LOW);
    }
    else if (color == 1)
    { // red
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
    }
    else if (color == 2)
    { // blue
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, LOW);
    }
    else if (color == 3)
    { // green
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
    }
    else if (color == 4)
    { // yellow
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
    }
    else if (color == 5)
    { // off
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
    }
}
