/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG 
 * Author     : Antariksh Narain
----------------------------------------------------------------- */

#include "sensors/ADS1015.hpp"

ADS1015::~ADS1015()
{
    printf("Closing ADS1015.\n");
    this->exit_signal.set_value();
    sleep(1);
}

ADS1015::ADS1015(int device_address) : I2C(device_address)
{
    // Initialize Manager thread
    this->exit_future = this->exit_signal.get_future();
    thread(&ADS1015::adsManager, this, std::move(this->exit_future)).detach();
}

void ADS1015::readValues()
{
    for (int i = 0; i < NUM_ANALOG; i++)
    {
        switch (i)
        {
        case 0:
            i2cWriteWord(CONFIG_REG, 0xc183);
            break;
        case 1:
            i2cWriteWord(CONFIG_REG, 0xd183);
            break;
        case 2:
            i2cWriteWord(CONFIG_REG, 0xe183);
            break;
        case 3:
            i2cWriteWord(CONFIG_REG, 0xf183);
            break;
        }
        i2cSend(CONV_REG);
        std::this_thread::sleep_for(std::chrono::nanoseconds(DELAY_READ));
        int16_t value = i2cReadWord(CONV_REG);
        this->analog_value[i] = (value >> 8) | (value << 8);
        
    }
}

void ADS1015::adsManager(std::future<void> _future)
{
    printf("Starting ADS1015 manager!\n");
    while (_future.wait_for(chrono::milliseconds(RATE)) == std::future_status::timeout)
    {
        this->readValues();
    }
    printf("Closing ADS1015 Manager.\n");
}

#ifdef TEST_ADS1015
#include <bits/stdc++.h>
#include <unistd.h>
int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        printf("Pass port as parameters\n");
        return -1;
    }
    ADS1015 ads(atoi(argv[1]));
    int values = 100;
    while (values--)
    {
        for(int i=0;i<NUM_ANALOG;i++)
        {
            printf("%d, ", ads.analog_value[i]);
        }
        printf("\n");
        usleep(100000);
    }
    return 0;
}
#endif
