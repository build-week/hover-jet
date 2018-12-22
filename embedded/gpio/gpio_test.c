// Build with gcc -o gpio_test gpio_test.c -lwiringPi -lwiringPiDev -lpthread -lcrypt -lm -lrt
#include <wiringPi.h>
 
int main(void)
{
    wiringPiSetup();
    pinMode(0, OUTPUT);
 
    for (;;)
    {
        digitalWrite(0, HIGH); 
        delay(1000);
        digitalWrite(0, LOW); 
        delay(1000);
    }
    return 0;
}
