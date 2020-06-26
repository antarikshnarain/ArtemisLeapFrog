#include "controls.h"

Directions::Directions()
{
    // Start in the "middle" of the arbitrary plane
    x_coord = 0;
    y_coord = 0;
}
void Directions::process(char cmd)
{
    
}
std::pair<int, int> Directions::getState()
{
    return std::make_pair(x_coord, y_coord);
}
void Directions::setState(char cmd)
{
    int temp;
    // increment y_coord by 1
    if (cmd == 'w')
    {
        temp = y_coord;
        temp += 1;
        if (temp > 5)
            y_coord = 5;
        else
            y_coord = temp;
    }
    // decrement x_coord by 1
    else if (cmd == 'a')
    {
        temp = x_coord;
        temp -= 1;
        if (temp < -5)
            x_coord = -5;
        else
            x_coord = temp;
    }
    // decrement y_coord by 1
    else if (cmd == 's')
    {
        temp = y_coord;
        temp -= 1;
        if (temp < -5)
            y_coord = -5;
        else
            y_coord = temp;
    }
    // increment x_coord by 1
    else if (cmd == 'd')
    {
        temp = x_coord;
        temp += 1;
        if (temp > 5)
            x_coord = 5;
        else
            x_coord = temp;
    }
}



Thruster::Thruster()
{
    // initialize the thrust to 0 (obivously)
    thrust_level = 0;
}
void Thruster::process(char cmd)
{

}
int Thruster::getState()
{
    return thrust_level;
}
void Thruster::setState(char cmd)
{
    int temp = thrust_level;

    // increase thrust
    if (cmd == 'z')
    {
        // increase thrust by 5% (is percent the right unit to be using?)
        temp += 5;
        // check if it has eclipsed 100% thrust
        if (temp > 100)
        {
            thrust_level = 100;
            // Should we cout something here? (i.e. pass in an ostream as param)
        }
        else
        {
            thrust_level = temp;
        }
    }
    // decrease thrust
    else if (cmd == 'c')
    {
        temp -= 5;
        // check if it has dipped to zero or below
        if (temp < 0)
        {
            thrust_level = 0;
            // Cout something here?
        }
        else
        {
            thrust_level = temp;
        }
    }

    return;
}



Yaw::Yaw()
{
    // Start w/ Leapfrog rotated 0 degrees
    orientation = 0;
}
void Yaw::process(char cmd)
{
    
}
int Yaw::getState()
{
    return orientation;
}
void Yaw::setState(char cmd)
{
    // temp variable to avoid >360 bugs
    int temp = orientation;

    // if we want to rotate leapfrog clockwise ('e' key)
    if (cmd == 'e')
    {
        // increment by 5 degrees
        temp += 5;
        // checks for "overflow" (degree measures over 360)
        if (temp >= 360)
            orientation = temp - 360;
        else
            orientation = temp;
    }
    // if we want to rotate anticlockwise ('q' key)
    else if (cmd == 'q')
    {
        // decrement by 5 degrees
        temp -= 5;
        // checks for "overflow" (negative degrees)
        if (temp < 0)
            orientation = 360 + temp;
        else
            orientation = temp;
    }
    
    return; 
}

