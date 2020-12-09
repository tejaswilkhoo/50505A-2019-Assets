#pragma config(Sensor, in1,    line1,          sensorLineFollower)
#pragma config(Sensor, dgtl1,  flipperencoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  driveencoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl11, liftencoder,    sensorQuadEncoder)
#pragma config(Motor,  port1,           leftdrive1,    tmotorVex393HighSpeed_HBridge, openLoop)
#pragma config(Motor,  port2,           leftdrive2,    tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           flipper,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           shooter,       tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           lift,          tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port6,           ballintake,    tmotorVex393HighSpeed_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           rightdrive2,   tmotorVex393HighSpeed_MC29, openLoop)
#pragma config(Motor,  port10,          rightdrive1,   tmotorVex393HighSpeed_HBridge, openLoop, reversed)

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"

int direction = 1;
int liftval = 1;
int highflag = -120;
int lowflag = -180;

float liftkp = 1.3;
float liftrequestedvalue = 0;
float flipperkp = 1.3;
float flipperrequestedvalue = 0;

void movedistance (int degrees, int power, int type)
{
	if (SensorValue(driveencoder) > degrees)
	{
		repeatUntil(SensorValue(driveencoder) <= degrees)
		{
			motor[leftdrive1] = -power;
			motor[leftdrive2]  = -power;
			motor[rightdrive1] = -power;
			motor[rightdrive2]  = -power;
		}

		if (type == 1)
		{
			motor[leftdrive1] = 10;
			motor[leftdrive2] = 10;
			motor[rightdrive2] = 10;
			motor[rightdrive1] = 10;
		}

		if (type == 0)
		{
			motor[leftdrive1] = 0;
			motor[leftdrive2] = 0;
			motor[rightdrive2] = 0;
			motor[rightdrive1] = 0;
		}
	}

	else
	{
		repeatUntil(SensorValue(driveencoder) >= degrees)
		{
			motor[leftdrive1] = power;
			motor[leftdrive2]  = power;
			motor[rightdrive1] = power;
			motor[rightdrive2]  = power;
		}
		if (type == 1)
		{
			motor[leftdrive1] = -10;
			motor[leftdrive2] = -10;
			motor[rightdrive2] = -10;
			motor[rightdrive1] = -10;
		}

		if (type == 0)
		{
			motor[leftdrive1] = 0;
			motor[leftdrive2] = 0;
			motor[rightdrive2] = 0;
			motor[rightdrive1] = 0;
		}
	}
}

void turndistance (int degrees, int power, int type)
{
	if (SensorValue(driveencoder) > degrees)
	{
		repeatUntil(SensorValue(driveencoder) <= degrees)
		{
			motor[leftdrive1] = -power;
			motor[leftdrive2]  = -power;
			motor[rightdrive1] = power;
			motor[rightdrive2]  = power;
		}

		if (type == 1)
		{
			motor[leftdrive1] = 10;
			motor[leftdrive2] = 10;
			motor[rightdrive2] = -10;
			motor[rightdrive1] = -10;
		}
		if (type == 0)
		{
			motor[leftdrive1] = 0;
			motor[leftdrive2] = 0;
			motor[rightdrive2] = 0;
			motor[rightdrive1] = 0;
		}
	}

	else
	{
		repeatUntil(SensorValue(driveencoder) >= degrees)
		{
			motor[leftdrive1] = power;
			motor[leftdrive2]  = power;
			motor[rightdrive1] = -power;
			motor[rightdrive2]  = -power;
		}

		if (type == 1)
		{
			motor[leftdrive1] = -10;
			motor[leftdrive2] = -10;
			motor[rightdrive2] = 10;
			motor[rightdrive1] = 10;
		}

		if (type == 0)
		{
			motor[leftdrive1] = 0;
			motor[leftdrive2] = 0;
			motor[rightdrive2] = 0;
			motor[rightdrive1] = 0;
		}
	}
}

task flipperpid()
{
	float  flippercurrentvalue;
  float  flippererror;
  float  flipperpower;

  while (true)
  {
  	flippercurrentvalue = SensorValue[flipperencoder];
	  flippererror =  flipperrequestedvalue - flippercurrentvalue;
    flipperpower = (flipperkp * flippererror);

    if (flipperpower > 127)
    {
    	flipperpower = 127;
    }
    else if (flipperpower < -127)
    {
    	flipperpower = -127;
    }

    motor[flipper] = flipperpower;
  }
}

task liftpid()
{
  float  liftcurrentvalue;
  float  lifterror;
  float  liftpower;

  while (true)
  {
		liftcurrentvalue = SensorValue[liftencoder];
		lifterror =  liftrequestedvalue - liftcurrentvalue;
		liftpower = (liftkp * lifterror - 30);

    if (liftpower > 127)
    {
    	liftpower = 127;
    }
    else if (liftpower < -127)
    {
    	liftpower = -127;
    }

		motor[lift] = liftpower;
  }
}

task shoot()
{
	motor[shooter]=127;
	wait1Msec(800);
	motor[shooter]=0;
}

task ballpick()
{
	repeatUntil(SensorValue(line1)<2900)
	{
		motor[ballintake]=127;
	}
	motor[ballintake]=127;
	wait1Msec(500);
	motor[ballintake]=-127;
	wait1Msec(250);
	motor[ballintake]=0;
}

task balldrop()
{
	motor[ballintake]=-127;
	wait1Msec(700);
	motor[ballintake]=0;
}

void pre_auton()
{
}

task autonomous()
{
	SensorValue(liftencoder) = 0;
	SensorValue(flipperencoder) = 0;
	SensorValue(driveencoder) = 0;

  liftrequestedvalue = SensorValue[liftencoder];
  flipperrequestedvalue = SensorValue[flipperencoder];

  startTask(liftpid);
  startTask(flipperpid);

  movedistance(-1000,127,0);
  movedistance(-1650,30,1);
  wait1Msec(500);

  startTask(ballpick);
  wait1Msec(500);
  movedistance(-380,127,1);
  wait1Msec(500);

  turndistance(-330,40,1);
  wait1Msec(500);


  liftrequestedvalue = highflag;
  startTask(shoot);
  wait1Msec(1000);

  startTask(ballpick);
  wait1Msec(1000);

  liftrequestedvalue = lowflag;
  startTask(shoot);
  wait1Msec(1000);

  turndistance(-330,40,1);
  wait1Msec(500);

  movedistance(-2000,127,1);


}

task usercontrol()
{
	SensorValue(liftencoder) = 0;
	SensorValue(flipperencoder) = 0;
	SensorValue(driveencoder) = 0;

  liftrequestedvalue = SensorValue[liftencoder];
  flipperrequestedvalue = SensorValue[flipperencoder];

  startTask(liftpid);
  startTask(flipperpid);

	clearTimer(T1);

	while (true)
	{
		if (vexRT[Btn7L]==1 && time1[T1]>500)
		{
			clearTimer(T1);
			direction = -direction;
			liftval = 3;
		}

		if (direction == 1) //ball pickup
		{
			motor[leftdrive1] = abs(vexRT[Ch3]) > 10 ? -vexRT[Ch3] : 0;
			motor[leftdrive2] = abs(vexRT[Ch3]) > 10 ? -vexRT[Ch3] : 0;
			motor[rightdrive1] = abs(vexRT[Ch2]) > 10 ? -vexRT[Ch2] : 0;
			motor[rightdrive2] = abs(vexRT[Ch2]) > 10 ? -vexRT[Ch2] : 0;

			if (vexRT[Btn5U]==1)
			{
				stopTask(balldrop);
				startTask(ballpick);
 			}

			if (vexRT[Btn5D]==1)
			{
				stopTask(ballpick);
				startTask(balldrop);
			}

			if (vexRT[Btn6U]==1) // aim to shoot top flag
			{
    		liftrequestedvalue = highflag;
 			}

			if (vexRT[Btn6D]==1) // aim to shoot lower flag
			{
    		liftrequestedvalue = lowflag;
      }

			if (vexRT[Btn8U]==1)
			{
				startTask(shoot);
			}
		}

	if (direction == -1) //cap pickup
		{
			motor[leftdrive1] = abs(vexRT[Ch2]) > 10 ? vexRT[Ch2] : 0;
			motor[leftdrive2] = abs(vexRT[Ch2]) > 10 ? vexRT[Ch2] : 0;
			motor[rightdrive1] = abs(vexRT[Ch3]) > 10 ? vexRT[Ch3] : 0;
			motor[rightdrive2] = abs(vexRT[Ch3]) > 10 ? vexRT[Ch3] : 0;

 			if (vexRT[Btn6U]==1) //lift cap all the way, keep same colour
			{
    		liftrequestedvalue = -680;
      	flipperrequestedvalue = 100;
      	liftval = 1;
			}

			if (vexRT[Btn5U]==1) // lift cap all the way, flip cap colour
			{
    		liftrequestedvalue = -680;
     		flipperrequestedvalue = -170;
     		liftval = 2;
 			}

			if (vexRT[Btn6D]==1 && liftval == 1 && time1[T1]>500) // dunk cap with same colour
			{
    		liftrequestedvalue = -450;
      	flipperrequestedvalue = 130;
      	clearTimer(T1);
      	liftval = 3;
      }

			if (vexRT[Btn6D]==1 && liftval == 2 && time1[T1]>500) // dunk cap with different colour
			{
    		liftrequestedvalue = -450;
     		flipperrequestedvalue = -180;
     		clearTimer(T1);
     		liftval = 3;
     	}

			if (vexRT[Btn6D]==1 && liftval == 3 && time1[T1]>500) //put lift down all the way
			{
    		liftrequestedvalue = 50;
     		flipperrequestedvalue = 0;
     		clearTimer(T1);
 			}

			if (vexRT[Btn5D]==1 && time1[T1]>500) // lift the lift to flip postition
			{
    		liftrequestedvalue = -430;
     		flipperrequestedvalue = -160;
     		clearTimer(T1);
     		liftval = 4;
     	}

			if (vexRT[Btn6D]==1 && liftval == 4 && time1[T1]>500) //flip the cap
			{
    		liftrequestedvalue = -680;
    		wait1Msec(500);
     		flipperrequestedvalue = 100;
    		wait1Msec(500);
     		liftrequestedvalue = -450;
     		clearTimer(T1);
     		liftval = 3;
 			}
 		}
	}
}
