// Programa : Display LCD 16x2 e modulo I2C
// Autor : Arduino e Cia

#include <wiringPiI2C.h>
//#include <LiquidCrystal_I2C.h>
#include "LiquidCrystal_I2C.h"
#include <iostream>
using namespace std;

// Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);

int main()
{
    lcd.begin (16,2);
    while()
    {
        //lcd.setBacklight(HIGH);
        lcd.setCursor(0,0);
        lcd.print("Arduino e Cia !!");
        lcd.setCursor(0,1);
        lcd.print("LCD e modulo I2C");
        delay(1000);
        //lcd.setBacklight(LOW);
        delay(1000);
    }
}
