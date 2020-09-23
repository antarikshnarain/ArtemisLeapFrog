#include "../include/SEN0259/SEN0259.h"

/*
* Name: begin()
* Function Prototype:   void SEN0259::begin(serialib &s_, string port)
* Description:  Opens device at specified port, with a BAUD rate of 115200
* Parameters:   string port - the port in which the device is connected
* Side Effects: Prints error if device fails to open or if an exception occurs
* Error Conditions: None
* Return Value: None
*/
void SEN0259::begin(char * port)
{
    char openStatus;
    
    try { openStatus = TFSerial.openDevice(port, 115200); }
    catch(const char * e) { fprintf(stderr, "\c\n", e); }

    if(openStatus != 1)
        fprintf(stderr, "DEVICE FAILED TO OPEN!\n");
}

/*
* Name: read()
* Function Prototype:   char DFRobot_TFmini::read(void)
* Description:  reads one byte from serial port
* Parameters:   None
* Side Effects: None
* Error Conditions: None
* Return Value: returns the byte that was read from serial port
*/
char SEN0259::read(void)
{
    char c;
    TFSerial.readChar(&c, 0);
    return c;
}

/*
* Name: measure()
* Function Prototype:   bool SEN0259::measure(void)
* Description:  measures and calculates the distance and strength values coming
                from the sensor connected through serial.
* Parameters:   None
* Side Effects: None
* Error Conditions: None
* Return Value: returns true if a character is able to be read
                returns false when a character is not available
*/
bool SEN0259::measure(void)
{
    uint8_t TFbuff[9] = {0};
    long checksum = 0;
    //loops runs while sensor data is available from serial port
    while (TFSerial.available())
    {
        //read in one byte
        TFbuff[0] = read();
        //initialize checksum
        checksum += TFbuff[0];
        //if first character is not 'Y' reset checksum and loop
        if (TFbuff[0] == 'Y')
        {
            //if 'Y', then read next byte
            TFbuff[1] = read();
            checksum += TFbuff[1];
            //if next character is not 'Y', reset checksum and loop
            if (TFbuff[1] == 'Y')
            {
                //if both 'Y', then read the next 6 bytes and add checksum
                for (int i = 2; i < 8; i++)
                {
                    TFbuff[i] = read();
                    checksum += TFbuff[i];
                }
                //read last byte
                TFbuff[8] = read();
                //bitwise and operation to calculate checksum
                checksum &= 0xff;
                //if checksum matches the last byte, then data is not corrupter
                if (checksum == TFbuff[8])
                {
                    //calculate sensor data
                    distance = TFbuff[2] + TFbuff[3] * 256;
                    strength = TFbuff[4] + TFbuff[5] * 256;
                    return true;
                }
                else
                {
                    checksum = 0;
                }
            }
            else
            {
                checksum = 0;
            }
        }
        else
        {
            checksum = 0;
        }
    }

    //return false if no data available from serial port
    return false;
}

/*
* Name: close()
* Function Prototype:   void SEN0259::close(void)
* Description:  closes the current device.
* Parameters:   None
* Side Effects: None
* Error Conditions: None
* Return Value: None
*/
void SEN0259::close(void)
{
    TFSerial.close();
}

/*
* Name: getDistance()
* Function Prototype:   uint16_t SEN0259::getDistance(void)
* Description:  returns the distance retrieved from the sensor
* Parameters:   None
* Side Effects: None
* Error Conditions: None
* Return Value: distance
*/
uint16_t SEN0259::getDistance(void)
{
    return distance;
}

/*
* Name: getStrength()
* Function Prototype:   uint16_t SEN0259::getStrength(void)
* Description:  returns the strength value retrieved from the sensor
* Parameters:   None
* Side Effects: None
* Error Conditions: None
* Return Value: strength
*/
uint16_t SEN0259::getStrength(void)
{
    return strength;
}