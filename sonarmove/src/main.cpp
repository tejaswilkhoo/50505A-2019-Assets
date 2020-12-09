#include "robot-config.h"
#include "math.h"
#include "algorithm"

int sfind()
{
    while(1)
    {
        Brain.Screen.printAt( 10, 20, "L value: %f" , Leftdrivefront.rotation(vex::rotationUnits::deg));
        Brain.Screen.printAt( 10, 40, "R value: %f" , Rightdriveback.rotation(vex::rotationUnits::deg));
        Brain.Screen.printAt( 10, 80, "Sonar value: %f" , Sonar.distance(vex::distanceUnits::mm));

    }
}



int sonarmove(int sdistance, int sspeed)
{
  vex::task:: sleep(200);
  if (Sonar.distance(vex::distanceUnits::mm) > sdistance)
  {
    while (Sonar.distance(vex::distanceUnits::mm) > sdistance) 
    {
        Leftdrivefront.spin(vex::directionType::rev,sspeed,vex::velocityUnits::rpm);
        Leftdriveback.spin(vex::directionType::rev,sspeed,vex::velocityUnits::rpm);
        Rightdrivefront.spin(vex::directionType::rev,sspeed,vex::velocityUnits::rpm);
        Rightdriveback.spin(vex::directionType::rev,sspeed,vex::velocityUnits::rpm);
    }
    Leftdrivefront.stop(vex::brakeType::brake);
    Leftdriveback.stop(vex::brakeType::brake);
    Rightdrivefront.stop(vex::brakeType::brake);
    Rightdriveback.stop(vex::brakeType::brake);
  }

  else if (Sonar.distance(vex::distanceUnits::mm) < sdistance)
  {
    while (Sonar.distance(vex::distanceUnits::mm) < sdistance) 
    {
        Leftdrivefront.spin(vex::directionType::fwd,sspeed,vex::velocityUnits::rpm);
        Leftdriveback.spin(vex::directionType::fwd,sspeed,vex::velocityUnits::rpm);
        Rightdrivefront.spin(vex::directionType::fwd,sspeed,vex::velocityUnits::rpm);
        Rightdriveback.spin(vex::directionType::fwd,sspeed,vex::velocityUnits::rpm);
    }
    Leftdrivefront.stop(vex::brakeType::brake);
    Leftdriveback.stop(vex::brakeType::brake);
    Rightdrivefront.stop(vex::brakeType::brake);
    Rightdriveback.stop(vex::brakeType::brake);
  }


  return(0);

}

int main(void) 
{
  vex::task find(sfind);

  vex::task:: sleep(200);
  sonarmove(300,20);
}
