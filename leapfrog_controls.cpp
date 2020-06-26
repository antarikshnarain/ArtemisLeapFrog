#include <iostream> 
#include <stdio.h>  
#include "controls.h"
using namespace std;  


// Declaring global "state" variable
char CTRL_STATE = '\0';

int main()
{
    // Instatiating objects
    Directions dir;
    Thruster thrust;
    Yaw y;

    // Output prompt 
    cout << "Leapfrog is ready for commands:" << endl; 

    // Set terminal to raw mode
    // Enables fluid/continuous input of keystrokes
    system("stty raw");

    while(true)
    {
        // Wait for single character 
        CTRL_STATE = getchar();
    
        // Directional scenario
        if (CTRL_STATE == 'w' || CTRL_STATE == 'a' || CTRL_STATE == 's' || CTRL_STATE == 'd')
        {
            // Call dir.process()
            dir.setState(CTRL_STATE);
            cout << "Moving" << endl;
        }
        // Thruster scenario
        else if (CTRL_STATE == 'z' || CTRL_STATE == 'c')
        {
            // Call thrust.process()
            thrust.setState(CTRL_STATE);
            cout << "Changing thrust" << endl;
        }
        // Yaw scenario
        else if (CTRL_STATE == 'q' || CTRL_STATE == 'e')
        {
            // Call yaw.process()
            y.setState(CTRL_STATE);
            cout << "Changing yaw position" << endl;
        }
        else if (CTRL_STATE == '\n')
        {
            break;
        }
        else
        {
            continue;
        }

        // 
        CTRL_STATE = '\0';
    }

    cout << "Done!" << endl;
    
    // Reset terminal to normal "cooked" mode
    system("stty cooked"); 
    
    // And we're out of here 
    return 0; 
}