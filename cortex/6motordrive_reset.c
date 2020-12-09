#pragma config(Sensor, in1,    line1,          sensorLineFollower)
#pragma config(Sensor, dgtl1,  flipperencoder, sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  rightencoder,   sensorQuadEncoder)
#pragma config(Sensor, dgtl9,  leftencoder,    sensorQuadEncoder)
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

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--TASKS--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

int direction = 1;
int liftval = 1;
int highflag = -100;
int lowflag = -160;
int auton = 0;

float liftkp = 1.3;
float liftrequestedvalue = 0;
float flipperkp = 1.3;
float flipperrequestedvalue = 0;

void displayAuton()  //Different autonomous choices
{
	switch(auton)
	{
		case 0:

			clearLCDLine(0);
			clearLCDLine(1);
			string mainBattery, backupBattery;
			displayLCDString(0, 0, "Primary: ");
			sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V');
			displayNextLCDString(mainBattery);
			displayLCDString(1, 0, "Backup: ");
			sprintf(backupBattery, "%1.2f%c", BackupBatteryLevel/1000.0, 'V');
			displayNextLCDString(backupBattery);
			wait1Msec(100);
			break;

		case 1:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDCenteredString(0, "Blue Right");
			break;

		case 2:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDCenteredString(0, "Red Left");
			break;

		case 3:
			clearLCDLine(0);
			clearLCDLine(1);
			displayLCDCenteredString(0, "Auto Skills");
			break;
	}
}

task LCDControl() //program selector through lcd display
{
	clearLCDLine(0);
	clearLCDLine(1);
	bLCDBacklight = true;
	bool noButtonsPressed = true;
	displayAuton();

	while(true)
	{
		if(noButtonsPressed)
		{
			switch(nLCDButtons)
			{
			case kButtonLeft:
				auton--;
				displayAuton();
				break;
			case kButtonCenter:
				stopTask(LCDControl);
				break;
			case kButtonRight:
				auton++;
				displayAuton();
				break;
			}
		}
		noButtonsPressed = !nLCDButtons;
		wait1Msec(20);
	}
}

void movedistance (int degrees, int power, int type) //command for moving straight
{
	SensorValue(leftencoder) = 0;
	SensorValue(rightencoder) = 0;
	repeatUntil(abs(SensorValue(leftencoder)) >= degrees && abs(SensorValue(rightencoder)) >= degrees)
	{
		motor[leftdrive1] = -power;
		motor[leftdrive2]  = -power;
		motor[rightdrive1] = -0.9*power;
		motor[rightdrive2]  = -0.9*power;
	}
	SensorValue(leftencoder) = 0;
	SensorValue(rightencoder) = 0;

	if (type == 1)
	{
		if (power > 0)
		{
			motor[leftdrive1] = 10;
			motor[leftdrive2] = 10;
			motor[rightdrive2] = 10;
			motor[rightdrive1] = 10;
		}
		else
		{
			motor[leftdrive1] = -10;
			motor[leftdrive2] = -10;
			motor[rightdrive2] = -10;
			motor[rightdrive1] = -10;
		}
	}
	if (type == 0)
	{
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;
	}
}

void turndistance (int degrees, int power, int type) //command for turning
{
	SensorValue(leftencoder) = 0;
	SensorValue(rightencoder) = 0;

	repeatUntil(abs(SensorValue(leftencoder)) >= degrees && abs(SensorValue(rightencoder)) >= degrees)
	{
		motor[leftdrive1] = -power;
		motor[leftdrive2]  = -power;
		motor[rightdrive1] = 0.9*power;
		motor[rightdrive2]  = 0.9*power;
	}

	SensorValue(leftencoder) = 0;
	SensorValue(rightencoder) = 0;

	if (type == 1)
	{
		if (power > 0)
		{
			motor[leftdrive1] = 10;
			motor[leftdrive2] = 10;
			motor[rightdrive2] = -10;
			motor[rightdrive1] = -10;
		}
		else
		{
			motor[leftdrive1] = -10;
			motor[leftdrive2] = -10;
			motor[rightdrive2] = 10;
			motor[rightdrive1] = 10;
		}
	}
	if (type == 0)
	{
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;
	}
}

task flipperpid() //PID loop to bring flipper to desired value
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

task liftpid() //PID loop to bring lift to desired value
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

task shoot() //Task to shoot a ball
{
	motor[shooter]=127;
	wait1Msec(800);
	motor[shooter]=0;
}

task ballpick() //Task to pick up a ball
{
	repeatUntil(SensorValue(line1)<2900)
	{
		motor[ballintake]=127;
	}
	motor[ballintake]=127;
	wait1Msec(500);
}

task balldrop() //Task to remove a ball from intake
{
	motor[ballintake]=-127;
	wait1Msec(70);
	motor[ballintake]=0;
}

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--PRE AUTONOMOUS--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

void pre_auton()
{
	SensorValue(liftencoder) = 0;
	SensorValue(flipperencoder) = 0;
	SensorValue(leftencoder) = 0;
	SensorValue(rightencoder) = 0;

	bStopTasksBetweenModes = true;
	bDisplayCompetitionStatusOnLcd = false;

	startTask(LCDControl);
}

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--AUTONOMOUS--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

task autonomous()
{
	stopTask(LCDControl);
	clearTimer(T1);
	clearLCDLine(0);
	clearLCDLine(1);
	SensorValue(liftencoder) = 0;
	SensorValue(flipperencoder) = 0;
	SensorValue(leftencoder) = 0;
	SensorValue(rightencoder) = 0;

	liftrequestedvalue = SensorValue[liftencoder];
	flipperrequestedvalue = SensorValue[flipperencoder];

	startTask(liftpid);
	startTask(flipperpid);

	liftrequestedvalue = 50;

	switch(auton)
	{
		case 10: //No autonomous
		break;

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--BLUE RIGHT--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

		case 1: //Autonomous for Blue Right square

	  movedistance(600,127,0);
	  startTask(ballpick);
	  movedistance(600,30,1); //Move forward to flip cap and collect ball
	  wait1Msec(500);

	  movedistance(700,-127,0); //Come back
		motor[leftdrive1] = 20;
		motor[leftdrive2] = 20;
		motor[rightdrive2] = 20;
		motor[rightdrive1] = 20;
	  wait1Msec(1500);
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;

		SensorValue(leftencoder) = 0;
		SensorValue(rightencoder) = 0;

		startTask(balldrop);
		movedistance(210,40,1);
		wait1Msec(500);

	  turndistance(305,40,1); //Align to shoot flags
	  wait1Msec(500);

	  liftrequestedvalue = -110; //Shoot high flag
	  wait1Msec(1000);
	  startTask(shoot);
	  wait1Msec(1000);

	  startTask(ballpick);
	  liftrequestedvalue = -180; //Shoot low flag
	  wait1Msec(1000);
	  startTask(shoot);
	  wait1Msec(1000);

		stopTask(ballpick);
	  turndistance(20,40,1);
	  wait1Msec(500);

		motor[leftdrive1] = -127; //Flip bottom flag
		motor[leftdrive2] = -127;
		motor[rightdrive2] = -127;
		motor[rightdrive1] = -127;
	  wait1Msec(1500);
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;

		displayNextLCDNumber(time1[T1]);

		break;

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--RED LEFT--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

		case 2:

	  movedistance(600,127,0); //Move forward to collect ball and flip cap
	  startTask(ballpick);
	  movedistance(600,30,1);
	  wait1Msec(500);

	  movedistance(700,-127,0); //Come back
		motor[leftdrive1] = 20;
		motor[leftdrive2] = 20;
		motor[rightdrive2] = 20;
		motor[rightdrive1] = 20;
	  wait1Msec(1500);
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;

		SensorValue(leftencoder) = 0;
		SensorValue(rightencoder) = 0;

		startTask(balldrop);
		movedistance(210,40,1);
		wait1Msec(500);

	  turndistance(305,-40,1); //Turn to align with flags
	  wait1Msec(500);

	  liftrequestedvalue = -100; //Shoot high flag
	  wait1Msec(1000);
	  startTask(shoot);
	  wait1Msec(1000);

	  startTask(ballpick);

	  liftrequestedvalue = -180; //Shoot low flag
	  wait1Msec(1000);
	  startTask(shoot);
	  wait1Msec(1000);

		stopTask(ballpick);
	  turndistance(20,-40,1);
	  wait1Msec(500);

		motor[leftdrive1] = -127;
		motor[leftdrive2] = -127;
		motor[rightdrive2] = -127;
		motor[rightdrive1] = -127;
	  wait1Msec(1500);
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;

		displayNextLCDNumber(time1[T1]);

		break;


/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--AUTONOMOUS SKILLS BLUE RIGHT--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

		case 0: //autonomous skills red left

	  movedistance(600,127,0); //Move forward to collect ball and flip cap
	  startTask(ballpick);
	  movedistance(600,30,1);
	  wait1Msec(500);

	  movedistance(700,-127,0); //Come back
		motor[leftdrive1] = 20;
		motor[leftdrive2] = 20;
		motor[rightdrive2] = 20;
		motor[rightdrive1] = 20;
	  wait1Msec(1500);
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;

		SensorValue(leftencoder) = 0;
		SensorValue(rightencoder) = 0;

		startTask(balldrop);
		movedistance(210,40,1);
		wait1Msec(500);

	  turndistance(305,-40,1); //Turn to align with flags
	  wait1Msec(500);

	  liftrequestedvalue = -100; //Shoot high flag
	  wait1Msec(1000);
	  startTask(shoot);
	  wait1Msec(1000);

	  startTask(ballpick);

	  liftrequestedvalue = -180; //Shoot low flag
	  wait1Msec(1000);
	  startTask(shoot);
	  wait1Msec(1000);

		stopTask(ballpick);

		movedistance(750,-127,1);
		wait1Msec(500);

		liftrequestedvalue = 50;
		turndistance(290,40,1);
	  wait1Msec(500);

		motor[leftdrive1] = 30; //Align to park
		motor[leftdrive2] = 30;
		motor[rightdrive2] = 30;
		motor[rightdrive1] = 30;
	  wait1Msec(1500);
		motor[leftdrive1] = 0;
		motor[leftdrive2] = 0;
		motor[rightdrive2] = 0;
		motor[rightdrive1] = 0;

		liftrequestedvalue = -100;

		motor[leftdrive1] = -127; //Center park
		motor[leftdrive2] = -127;
		motor[rightdrive2] = -127;
		motor[rightdrive1] = -127;
	  wait1Msec(3200);
		motor[leftdrive1] = 10;
		motor[leftdrive2] = 10;
		motor[rightdrive2] = 10;
		motor[rightdrive1] = 10;

		displayNextLCDNumber(time1[T1]);

		break;
	}
}

/*XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX--USERCONTROL--XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX*/

task usercontrol() //Driver control
{
	stopTask(LCDControl);
	liftrequestedvalue = SensorValue[liftencoder];
	flipperrequestedvalue = SensorValue[flipperencoder];

	startTask(liftpid);
	startTask(flipperpid);

	clearTimer(T1);

	switch(auton)
	{
		case 10: //throwaway case
		break;

		default:

			while (true)
			{
				if (vexRT[Btn7L]==1 && time1[T1]>500) //change mode, cap or ball
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

					if (vexRT[Btn5U]==1) //pick up a ball
					{
						stopTask(balldrop);
						startTask(ballpick);
		 			}

					if (vexRT[Btn5D]==1) //remove a ball
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

					if (vexRT[Btn8U]==1) //shoot a ball
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
		    		liftrequestedvalue = -420;
		      	flipperrequestedvalue = 130;
		      	clearTimer(T1);
		      	liftval = 3;
		      }

					if (vexRT[Btn6D]==1 && liftval == 2 && time1[T1]>500) // dunk cap with different colour
					{
		    		liftrequestedvalue = -420;
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
		    		liftrequestedvalue = -420;
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
		     		liftrequestedvalue = -420;
		     		clearTimer(T1);
		     		liftval = 3;
					}
				}
			}
	break;
	}
}
