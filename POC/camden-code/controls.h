#ifndef CONTROLS_H
#define CONTROLS_H
#include <utility>


/*** Think this class is superfluous...no real need to inherit
class Controls
{
public:
    Controls();
    char getState();
    void setState(char input);
    virtual void process(char cmd) = 0;

protected:
    char control_state;
};
****/

/***** Start of Directions Class *****/
class Directions
{
public:
    Directions();
    // Executes
    void process(char cmd);

    // Returns current coords (Should this return a pair of ints?)
    std::pair<int, int> getState();

    // Update coords
    void setState(char cmd);

private:
    int x_coord;
    int y_coord;
};
/***** End of Directions *****/

/***** Start of Thruster Class *****/
class Thruster
{
public:
    Thruster();

    // Executes the "action"
    void process(char cmd);

    // Returns thrust level
    int getState();

    // set thrust level to the new value
    void setState(char cmd);

private:

    // holds Leapfrog's thrust_level (from 0 to some MAX_THRUST value)
    int thrust_level;
};
/***** End of Thruster *****/

/***** Start of Yaw Class *****/
class Yaw
{
public:
    Yaw();
    void process(char cmd);

    // Returns lepafrog's orientation in degrees
    int getState();

    // set orientation to the new value
    void setState(char cmd);

private:

    // holds Leapfrog's orientation(in degrees) relative to some arbitary 0 point
    int orientation;
};
/***** End of Yaw Class *****/
#endif