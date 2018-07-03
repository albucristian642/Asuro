#include "lib/inc/asuro.h"
#include <asf.h>
#define  DISTANCE 50
enum States {
	LINEFOLLOW,
	SEARCHLINE,
	FINDLINE,
	SCANBARCODE,
	BLINK,
	ENDSTATE,
	OBSTACLE
};
enum BarcodeState {
	BRIGHT,
	DARK,
	END
};
int barcodeCounter = 0;

void lineFollowing(int line_position);
void follow(void);
void search(void);
void find(void);
void scan(void);
void blinkNTimes(void);
void avoidObstacle(void);
int lostLine(void);
void MyTurn(int);
enum States currentState =LINEFOLLOW;

int main (void)
{
	Init();
	Msleep(2000);
	PrintInt(Battery());PrintInt(-404);
	SwitchDetection(ON);
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

void follow() {
      while (!switchesActive && !lostLine()) {

	      if (lineFollowDelta < 0) {
		      if (lineFollowDelta < -90) {
			      MotorSpeed(100, -150);
			      } else {
			      MotorSpeed(120, 100);
		      }
		      } else {
		      if (lineFollowDelta > 90) {
			      MotorSpeed(-150, 100);
			      } else {
			      MotorSpeed(100, 140);
		      }
	      }

      }
	if(switchesActive) currentState = OBSTACLE;
	if(lostLine()) currentState = SEARCHLINE;

}
void scan(){
/*	enum BarcodeState barcode_state = DARK;
 	while(lineFollowLeft < DARK_SURFACE && lineFollowRight < DARK_SURFACE){Drive(10,120);}
	Msleep(100);
	barcode_state = BRIGHT;
	barcodeCounter ++;
	int ticks = 0;
	while(ticks < DISTANCE)
	{	
		PrintInt(lineFollowLeft); PrintInt(-1);
		PrintInt(lineFollowRight); PrintInt(-2);
		PrintInt(barcode_state); PrintInt(-3);
		if(lineFollowLeft < DARK_SURFACE && lineFollowRight < DARK_SURFACE && barcode_state == BRIGHT)
		{
			barcode_state = DARK;
			ticks = 0;
		}
		if(lineFollowLeft > WHITE_SURFACE && lineFollowRight > WHITE_SURFACE && barcode_state == DARK)
		{
			barcode_state = BRIGHT;
			barcodeCounter++;
			ticks = 0;
		}
		Drive(10,120);
		Msleep(200);
		ticks += encoderLeft+encoderRight;
	}
	barcode_state = END;
	currentState = BLINK;*/
	MotorSpeed(100,125);
	unsigned long time_out, Wstarttime;
	time_out=Wstarttime=SystemTime();
	enum BarcodeState barcode_state = DARK;
	while(SystemTime() - Wstarttime <= 600) {
		while(time_out > SystemTime())
			continue;
		time_out+=50;
		if(lineFollowLeft > WHITE_SURFACE && barcode_state==DARK)
		{
			barcodeCounter ++;
			barcode_state = BRIGHT;
		}
		else if(barcode_state == BRIGHT && lineFollowLeft < DARK_SURFACE) {
			Wstarttime = SystemTime();
			barcode_state = DARK;
		}
	};
	MotorSpeed(0,0);
	barcode_state = END;
	currentState = BLINK;
}
void search(){
	MotorSpeed(0,0);
	Turn(-90,100);
	if(lineFollowRight > WHITE_SURFACE || lineFollowLeft > WHITE_SURFACE)
		currentState = LINEFOLLOW;
	else {
		Turn(180,100);
		if(lineFollowRight > WHITE_SURFACE || lineFollowLeft > WHITE_SURFACE)
			currentState = LINEFOLLOW;
		else {
			currentState = SCANBARCODE;
			Turn(-90,100);
		}
	}
}
void find(){
	while(lineFollowLeft < DARK_SURFACE && lineFollowRight < DARK_SURFACE) {
		Drive(40,150);
		Msleep(100);
	}
	currentState = LINEFOLLOW;
}
void blinkNTimes(){
	for(int i = 0; i < barcodeCounter; ++i)
	{
		BackLEDs(ON,ON);
		PrintInt(i);
		Msleep(1000);
		BackLEDs(OFF,OFF);
		Msleep(1000);
	}
	if(barcodeCounter == 1)
	currentState = ENDSTATE;
	else
	currentState = FINDLINE;
	barcodeCounter = 0;
}
void avoidObstacle(){
	Drive(-20,150);
	Turn(80,120);
	Drive(300,150);
	Turn(-80,120);
	Drive(300,150);
	Turn(-80,120);
	Drive(300,150);
	Turn(80,120);
	currentState = LINEFOLLOW;
}

int lostLine() {
	if(!lineFollowLeft && !lineFollowRight)
		return 0;
	if((lineFollowLeft+lineFollowRight) < 2 * DARK_SURFACE)
		return 1;
	return 0;
}

