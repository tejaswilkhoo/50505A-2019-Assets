#include "robot-config.h"

int sfind()
{
    while(1)
    {
        Brain.Screen.printAt( 10, 80, "Line Value: %d" , Line.value(vex::analogUnits::range8bit));

    }
}




int main(void) 
{
  vex::task find(sfind);
}
