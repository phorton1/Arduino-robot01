#include "myDebug.h"
#include <VirtualWire.h>

void setup()
{
    pinMode(13,OUTPUT);
    Serial.begin(115200);
    vw_setup(2000);
    display(0,"433Mhz radio xmit 2 setup completed",0);
}

void loop()
{
    #define SEND_CHARS   1

    #if SEND_CHARS
        if (Serial.available())
        {
            digitalWrite(13,1);
            char transmit = Serial.read();
            display(0,"sending '%c' chr(%d)",transmit>=32?transmit:' ',transmit);
            vw_send((byte *) &transmit, sizeof(transmit));
            vw_wait_tx();
            digitalWrite(13,0);
        }
    #endif

    static uint32_t flash_time = 0;
    static int flasher = 0;
    if (millis() > flash_time + 2000)
    {
        flasher = !flasher;
        digitalWrite(13,flasher);
        flash_time = millis();

        #if !SEND_CHARS
            static int transmit = 0;
            transmit++;
            display(0,"sending %d",transmit);
            vw_send((byte *) &transmit, sizeof(transmit));
            vw_wait_tx();
        #endif
    }
}
