#ifndef __DFRobot_TFmini_H__
#define __DFRobot_TFmini_H__


#include <stdint.h>
#include <cstdio>
#include "../serialib/serialib.h"

class SEN0259
{
public:
    void        begin(char * port);
    void        close(void);
    bool        measure(void);
    uint16_t    getDistance(void);
    uint16_t    getStrength(void);
private:
    char        read(void);
    serialib    TFSerial;
    uint16_t    distance = 0;
    uint16_t    strength = 0;
};

#endif