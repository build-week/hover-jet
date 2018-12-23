// Build with gcc -o gpio_test gpio_test.c -lwiringPi -lwiringPiDev -lpthread -lcrypt -lm -lrt
#include "third_party/wiringpi/wiringPi.h"
 
int main(void)
{
    wiringPiSetup();
    pinMode(0, OUTPUT);
 
    while(1) {
        digitalWrite(0, HIGH); 
        delay(1000);
        digitalWrite(0, LOW); 
        delay(1000);
    }
    return 0;
}
