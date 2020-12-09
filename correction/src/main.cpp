#include "robot-config.h"
#include "math.h"
#include "algorithm"

using namespace vex;
const double minimum_velocity = 20.0;

double sincreasing_speed (double starting_point, double current_position) 
{
    static const double acceleration_constant = 100.0;
    return acceleration_constant * std::abs(current_position - starting_point) + minimum_velocity;
}

double sdecreasing_speed (double ending_point, double current_position) 
{
    static const double deceleration_constant = 100.0;
    return deceleration_constant * std::abs(ending_point - current_position) + minimum_velocity;
}

int sfind()
{
    while(1)
    {
        Brain.Screen.printAt( 10, 20, "L value: %f" , Leftdrivefront.rotation(vex::rotationUnits::deg));
        Brain.Screen.printAt( 10, 40, "R value: %f" , Rightdriveback.rotation(vex::rotationUnits::deg));
    }
}

void strafe (double distanceIn, double maxVelocity) 
{
    static const double circumference = 360;
    double direction5 = distanceIn > 0 ? -1 : 1;
    double direction6 = distanceIn > 0 ? 1 : -1;
    double wheelRevs5 = (direction5*std::abs(distanceIn)) / circumference;
    double wheelRevs6 = (direction6*std::abs(distanceIn)) / circumference;
    
    Rightdrivefront.spin(directionType::fwd,direction5 * minimum_velocity,velocityUnits::pct);
    Leftdrivefront.spin(directionType::fwd,direction6 * minimum_velocity,velocityUnits::pct);
    Rightdriveback.spin(directionType::fwd,direction6* minimum_velocity,velocityUnits::pct);
    Leftdriveback.spin(directionType::fwd,direction5* minimum_velocity,velocityUnits::pct);
    
    double leftStartPoint = Leftdrivefront.rotation(rotationUnits::rev);
    double leftEndPoint = leftStartPoint + wheelRevs6;
    double rightStartPoint = Rightdrivefront.rotation(rotationUnits::rev);
    double rightEndPoint = rightStartPoint + wheelRevs5;
    
    double leftBStartPoint = Leftdriveback.rotation(rotationUnits::rev);
    double leftBEndPoint = leftBStartPoint + wheelRevs5;
    double rightBStartPoint = Rightdriveback.rotation(rotationUnits::rev);
    double rightBEndPoint = rightBStartPoint + wheelRevs6;
    
    while (
            (direction5* (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction5* wheelRevs5) ||
            (direction6* (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction6 * wheelRevs6)  ||
            (direction5* (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction5 * wheelRevs5)  ||
            (direction6* (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction6 * wheelRevs6)  
          ) 
    {
        if (direction5 * (Rightdrivefront.rotation(rotationUnits::rev) - rightStartPoint) < direction5 * wheelRevs5) 
        {
            Rightdrivefront.setVelocity(
                direction5 * std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(rightStartPoint,Rightdrivefront.rotation(rotationUnits::rev)),
                        sdecreasing_speed(rightEndPoint,Rightdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdrivefront.stop(brakeType::brake);
        }
        
        if (direction6 * (Leftdrivefront.rotation(rotationUnits::rev) - leftStartPoint) < direction6 * wheelRevs6) 
        {
            Leftdrivefront.setVelocity(
                direction6 * std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(leftStartPoint,Leftdrivefront.rotation(rotationUnits::rev)),
                        sdecreasing_speed(leftEndPoint,Leftdrivefront.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdrivefront.stop(brakeType::brake);
        }
        
        if (direction5 * (Leftdriveback.rotation(rotationUnits::rev) - leftBStartPoint) < direction5 * wheelRevs5) {
            Leftdriveback.setVelocity(
                direction5 *0.80* std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(leftBStartPoint,Leftdriveback.rotation(rotationUnits::rev)),
                        sdecreasing_speed(leftBEndPoint,Leftdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Leftdriveback.stop(brakeType::brake);
        }
        
        if (direction6 * (Rightdriveback.rotation(rotationUnits::rev) - rightBStartPoint) < direction6 * wheelRevs6) {
            Rightdriveback.setVelocity(
                direction6 *0.80* std::min(
                    maxVelocity,
                    std::min(
                        sincreasing_speed(rightBStartPoint,Rightdriveback.rotation(rotationUnits::rev)),
                        sdecreasing_speed(rightBEndPoint,Rightdriveback.rotation(rotationUnits::rev))
                    )
                ),
                vex::velocityUnits::pct
            );
        } 
        else 
        {
            Rightdriveback.stop(brakeType::brake);
        }
    }
    Leftdrivefront.stop(brakeType::brake);
    Leftdriveback.stop(brakeType::brake);
    Rightdrivefront.stop(brakeType::brake);
    Rightdriveback.stop(brakeType::brake);
}

void correct()
{
  vex::task::sleep(200);
  if  (Leftdrivefront.rotation(vex::rotationUnits::deg) > Rightdriveback.rotation(vex::rotationUnits::deg))
  {
     while (Leftdrivefront.rotation(vex::rotationUnits::deg) > Rightdriveback.rotation(vex::rotationUnits::deg))
      {
          Leftdrivefront.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
          Leftdriveback.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
          Rightdrivefront.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
          Rightdriveback.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
      }
      Leftdrivefront.stop(vex::brakeType::brake);
      Leftdriveback.stop(vex::brakeType::brake);
      Rightdrivefront.stop(vex::brakeType::brake);
      Rightdriveback.stop(vex::brakeType::brake);
    
  }
    
  else if (Leftdrivefront.rotation(vex::rotationUnits::deg) < Rightdriveback.rotation(vex::rotationUnits::deg))
  {
      while (Leftdrivefront.rotation(vex::rotationUnits::deg) < Rightdriveback.rotation(vex::rotationUnits::deg))
      {
          Leftdrivefront.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
          Leftdriveback.spin(vex::directionType::fwd,200,vex::velocityUnits::rpm);
          Rightdrivefront.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
          Rightdriveback.spin(vex::directionType::fwd,-200,vex::velocityUnits::rpm);
      }
      Leftdrivefront.stop(vex::brakeType::brake);
      Leftdriveback.stop(vex::brakeType::brake);
      Rightdrivefront.stop(vex::brakeType::brake);
      Rightdriveback.stop(vex::brakeType::brake);
  }
}
    
 int main() 
{
    vex::task sfinds(sfind);
    strafe(-500,100);
    correct();
}
