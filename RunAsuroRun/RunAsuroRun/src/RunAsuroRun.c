#include "lib/inc/asuro.h"
#include "lib/inc/myasuro.h"
#include <asf.h>

int WHITE_SURFACE =550;
int DARK_SURFACE  =200;
#define SPEED 150
#define MY_ODO_LIGHT_VALUE_L     600
#define MY_ODO_DARK_VALUE_L      700
#define MY_ODO_LIGHT_VALUE_R     650
#define MY_ODO_DARK_VALUE_R      750


enum States {
	LINEFOLLOW,
	SEARCHLINE,
	FINDLINE,
	SCANBARCODE,
	BLINK,
	ENDSTATE,
	OBSTACLE
};

enum States currentState =LINEFOLLOW;


float error=0;
float lastError=0;
float PV =0 ;
float kp = 0;
float ki = 0;
float kd =0;
int m1Speed=0;
int m2Speed=0;
int line_position=0;
int motorspeed=0;

int barcodeCounter = 0;

void lineFollowing(int line_position);
void follow(void);
void search(void);
void find(void);
void scan(void);
void blinkNTimes(void);
void avoidObstacle(void);
void Calibration(int n, int pause);

int main (void)
{
	Init();
	EncoderInit();
	//Calibration(5,200);
	StartSwitch();
	while(currentState != ENDSTATE) 
	{
		switch(currentState) {
			case LINEFOLLOW :
				follow();
				break;
			case SEARCHLINE :
				search();
				break;
			case FINDLINE :
				find();
				break;
			case SCANBARCODE :
				scan();
				break;
			case BLINK :
				blinkNTimes();
				break;
			case OBSTACLE :
				avoidObstacle();
				break;
			default:
				break;
		}
	}
	return 0;
}

void Calibration(int n, int pause) {
	int left_wheel=0,right_wheel = 0;
	unsigned int line_data[2];
	LineData(line_data);
	FrontLED(ON);
	for(int i = 0; i < n; i++)
	{
		LineData(line_data);
		left_wheel  += line_data[0];
		right_wheel += line_data[1];
		Msleep(pause);
	}
	FrontLED(OFF);
	WHITE_SURFACE = right_wheel/n;
	DARK_SURFACE = left_wheel/n;
	PrintInt(WHITE_SURFACE);SerPrint(" white");
	PrintInt(DARK_SURFACE);SerPrint(" dark\n");
}

void follow()
{
	FrontLED(ON);
	PrintInt(101);
	unsigned int line_data[2];
	LineData(line_data);
	PrintInt(line_data[0]);SerPrint(" left");
	PrintInt(line_data[1]);SerPrint(" right\n");	
	if(PollSwitch() > 0) 
	{
		//transition to PollSwitch
	//	currentState = OBSTACLE;
		SetMotorPower(0,0);
	}
	else
		if (line_data[0] > WHITE_SURFACE && line_data[1] > WHITE_SURFACE)
		{
			PrintInt(-666);
			//transition to SearchLine
			currentState = SEARCHLINE;
			SetMotorPower(0,0);
		}
		else {
			line_position = 0;
			if (line_data[0] < DARK_SURFACE && line_data[1] > WHITE_SURFACE)
				line_position = 0;
			else
			{
				if(line_data[1] < DARK_SURFACE && line_data[0] > WHITE_SURFACE)
					line_position = 1024;
				else
					line_position = (line_data[0]+line_data[1])/2;
			}
			lineFollowing(line_position);
		}
}

void search(){
	PrintInt(-555);
	unsigned int line_data[2];
	LineData(line_data);
	GoTurn(0,-90,SPEED);
							Msleep(1000);

	LineData(line_data);
	if(line_data[0] > WHITE_SURFACE || line_data[1] > WHITE_SURFACE)
	{
		currentState = LINEFOLLOW;
		error=lastError=0;
			BackLED(ON, OFF);
			Msleep(1000);
	}
	else {
		GoTurn(0,180,SPEED);
								Msleep(1000);
		LineData(line_data);
		if(line_data[0] > WHITE_SURFACE || line_data[1] > WHITE_SURFACE)
		{
			currentState = LINEFOLLOW;
			error=lastError=0;
						BackLED(OFF, ON);
						Msleep(1000);

		}
		else
		{
			GoTurn(0,-90,SPEED);
									Msleep(1000);

			currentState = SCANBARCODE;
			BackLED(ON, ON);
			Msleep(1000);
		}
	}
}
void find(){}
void scan(){}
void blinkNTimes(){
	for(int i = 0; i < barcodeCounter; ++i)
	{
		BackLED(ON,ON);
		PrintInt(i);
		Msleep(1000);
		BackLED(OFF,OFF);
		Msleep(1000);
	}
	if(barcodeCounter == 1)
		currentState = ENDSTATE;
	else
		currentState = FINDLINE;
	barcodeCounter = 0;
}
void avoidObstacle(){
	BackLED(OFF, ON);
	SetMotorPower(-100,-100);
	Msleep(500);
	currentState = LINEFOLLOW;
	StopSwitch();
	switched = 0;
	lastError=error=0;
}


void lineFollowing(int line_position) {
	switch(line_position) {
		// rotate left
		case 0 :
			PrintInt(-1);
			MotorDir(RWD,FWD);
			MotorSpeed(100,150);
			break;	
		//rotate right
		case 1024 :
			PrintInt(-2);
			MotorDir(FWD,RWD);
			MotorSpeed(150,100);
			break;
		default :
			PrintInt(-3);
			error = (float)line_position - DARK_SURFACE;
			kp = 0.5;
			kd = 1;

			PV = kp*error + kd*(error-lastError);
			lastError = error;

			if(PV > 55)
				PV=55;
			else
				if (PV < -55)
					PV = -55;
			m1Speed = 150 + PV;
			m2Speed = 150 - PV;
			MotorDir(FWD,FWD);
			MotorSpeed(m1Speed,m2Speed);
			break;				
	}
}

