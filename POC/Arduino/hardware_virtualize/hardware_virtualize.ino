#include <Arduino.h>

void setup()
{
    Serial.begin(57600);
    Serial1.begin(57600);
    Serial2.begin(9600);
}

/*
Before sending a new command it must be assured, that the echo and the handshakes of all
addressed devices have been received (especially when assuming that more than one ECU is
connected) !
*/
void EngineMimic()
{
    if(Serial2.available())
    {
        String data = Serial2.readString();
        //Serial.print("Data available: ");
        //Serial.println(data);
        if(data == "1,RAC,1\r")
        {
            Serial2.write("1,RAC,1\r1,HS,OK,35000,568,1.32,11,30.1,10\r");
        }
        else if(data == "1,DHC,1\r")
        {
            Serial2.write("1,DHC,1\r1,HS,OK,1\r");
        }
        else if(data == "1,RHC,1\r")
        {
            Serial2.write("1,RHC,1\r1,HS,OK,1,1,1,1,8,1,1\r");
        }
        else if(data == "1,RTY,1\r")
        {
            Serial2.write("1,RTY,1\r1,HS,OK,0.002,101.12,133,101,1452,909\r");
        }
        else if(data == "1,RFI,1\r")
        {
            Serial2.write("1,RFI,1\r1,HS,OK,100,1000,7888,9,6567,25342\r");
        }
        else if(data == "1,TCO,1\r")
        {
            Serial2.write("1,TCO,1\r1,HS,OK\r");
        }
        else if(data == "1,WPE,32.22\r")
        {
            Serial2.write("1,WPE,32.22\r1,HS,OK\r");
        }
        else if(data == "1,WRP,50000\r")
        {
            Serial2.write("1,WRP,50000\r1,HS,OK\r");
        }
    }
}

void loop()
{
    EngineMimic();
     if(Serial.available())
     {
         Serial1.write(Serial.read());
     }
     if(Serial1.available())
     {
         Serial.write(Serial1.read());
     }
}
