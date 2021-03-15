#include "myDebug.h"
#include <VirtualWire.h>



void setup()
{
    Serial.begin(115200);
    pinMode(13,OUTPUT);
    digitalWrite(13,0);
    delay(1000);
    display(0,"robot01.ino setup()",0);

    vw_setup(2000);
    vw_rx_start();
    display(0,"robot01-receiver.ino started",0);

}



void loop()
{
    static uint32_t flasher_time = 0;
    static int flasher = 0;
    if (millis() > flasher_time + 1000)
    {
        flasher = flasher ? 0 : 1;
        digitalWrite(13,flasher);
        flasher_time = millis();
    }

    int transmit;
    byte wireless = sizeof(int);
    if (1)
    {
        warning(0,"waiting for radio",0);
        vw_wait_rx();
    }

    if (vw_get_message((byte *) &transmit, &wireless))
    {
        display(0,"RADIO received %d",transmit);
    }

}
