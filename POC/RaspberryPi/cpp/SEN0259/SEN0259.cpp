#include "Serial.hpp"

using namespace std;

int main()
{
    Serial serial("/dev/ttyAMA1", 115200, 0, 1, 9);
    int count = 10;
    while(count)
    {
        if(serial.IsAvailable())
        {
            vector<uint8_t> data = serial.Recv();
            int sum = 0;
            uint8_t header1 = data[0];
            uint8_t header2 = data[1];
            uint16_t dist = (data[3] << 8) | data[2];
            uint16_t sig = (data[5] << 8) | data[4];
            uint8_t mode = (int8_t)data[6];
            uint8_t checksum = (int8_t)data[8];
            cout << data.size() << endl;
            
            for(int i=0;i<9;i++)
            {
                printf("%d,",data[i]);
            }
            printf("\n");
            for(int i=0;i<8;i++)
            {
                sum += (int8_t)data[i];
            }
            sum &= 0x00FF;
            if(sum == checksum)
            {
                printf("checksum passed!\n");
            }
            else
            {
                printf("checksum failed %d, %d\n", sum, checksum);
            }
            printf("0x%x 0x%x, %d, %d, %d\n", header1, header2, dist, sig, mode);
            count --;
        }
        else
        {
            //sleep(1);
            usleep(10000);
            printf("Sleeping!\n");
        }
    }
    return 0;
}