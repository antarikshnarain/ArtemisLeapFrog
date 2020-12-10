/*---------------------------------------------------------------
 * Copyright (c) 2020 Space Engineering Research Center (SERC)
 * Project    : LEAPFROG
 * Author     : Antariksh Narain
 * Description: Library to control JetCat P300 engine
----------------------------------------------------------------- */

#include "JetCatP300.hpp"

#define LOW 0
#define HIGH 1
namespace Actuators
{
    void digitalWrite(int pin, int value){};
    //#ifdef __RASPBERRYPI__
    JetCatP300::JetCatP300(char *interface_port, int baud_rate)
    {
        // Configure turbine
        RS232 data_wbd = this->SendCommand(1, "WBD", 0, NULL);
    }
    void JetCatP300::Start()
    {
        // Enable Safe mode in JetCat
        digitalWrite(JETCAT_SAFE, LOW);
        // Enable ECU
        digitalWrite(JETCAT_POWER, HIGH);
        // Start Turbine
        RS232 data_tco = this->SendCommand(1, "TCO", 1, new double{0});
    }
    void JetCatP300::Stop()
    {
        printf("Stopping Engine\n");
        // Stop Turbine
        RS232 data_tco = this->SendCommand(1, "TCO", 1, new double{1});

        // Power Off ECU
        digitalWrite(JETCAT_POWER, LOW);
    }
    // bool JetCatP300::Emergency()
    // {
    //     // Disable Engine save mode and kill power
    //     digitalWrite(JETCAT_SAFE, HIGH);
    // }

    RS232 JetCatP300::SendCommand(int adr, string cmdcode, int l, double argv[])
    {
        // Serial Send
        this->string_request(RS232(adr, cmdcode, l, argv));
        char resp[] = {"1,HS,OK,1"};
        return this->parse_response(resp);
    }

    string JetCatP300::GetTelemetry(bool verbose)
    {
        // RAC
        RS232 data_rac = this->SendCommand(1, "RAC", 0);
        printf("Turbine RPM: %f | EGT `C: %f | Pump: %fV | Turbine state: %f | Throttle position: %f | Engine: %fA\n",
               data_rac.params[0], data_rac.params[1], data_rac.params[2], data_rac.params[3], data_rac.params[4], data_rac.params[5]);

        // RFI
        RS232 data_rfi = this->SendCommand(1, "RFI", 0);
        printf("Fuel flow: %f | Rest Volume: %f | Set RPM: %f | Battery: %fV | LastRun: %fs | Fuel Consumed: %fml\n",
               data_rfi.params[0], data_rfi.params[1], data_rfi.params[2], data_rfi.params[3], data_rfi.params[4], data_rfi.params[5]);

        // RSS
        RS232 data_rss = this->SendCommand(1, "RSS", 0);
        printf("Off-condition: %f | Flight Speed: %f\n", data_rss.params[1], data_rss.params[3]);

        // RTY
        RS232 data_rty = this->SendCommand(1, "RTY", 0);
        printf("Firmware ver: %f | Version: %f | LastRun: %f | Total Operation: %f | Serial No: %f | Turbine Type: %f\n",
               data_rty.params[0], data_rty.params[1], data_rty.params[2], data_rty.params[3], data_rty.params[4], data_rty.params[5]);

        return "";
    }

    void JetCatP300::CheckHealth()
    {
        // Do Health Check
        RS232 data_dhc = this->SendCommand(1, "DHC", 0);
        // Wait for 8 seconds
        int delay_ctr = 8;
        while (delay_ctr)
        {
            printf("Wait! %d\n", delay_ctr);
            usleep(1000000);
            delay_ctr--;
        }
        // Read Health check
        RS232 data_rhc = this->SendCommand(1, "RHC", 0);
        // Check flags
        // Starter Health Flag
        char error_list[][20] = {"Starter", "Main Valve", "Starter Valve", "RPM Sensor", "Pump", "Glow Plug", "EGT Sensor"};
        for (int i = 0; i < data_rhc.len; i++)
        {
            if (!((int)data_rhc.params[i] & 0x01))
            {
                printf("Error in %s\n", error_list[i]);
            }
        }
    }
    void JetCatP300::Thrust(double per_value)
    {
        // Set turbine thrust %age
        RS232 data_wpe = this->SendCommand(1, "WPE", 1, new double{per_value});
    }
    //#endif
} // namespace Actuators