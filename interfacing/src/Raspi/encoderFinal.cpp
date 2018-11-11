
/******************
* Name: Encoder.cpp
* Description: Calculate velocity and ticks
********************/
#include<iostream>
#include"wiringPi.h"
#include<stdio.h>
#include<stdlib.h>
#include"softPwm.h"
#include<math.h>
#include<signal.h>
#include<ros.h>
#include<interfacing/Velocity.h>
#include <unistd.h>

using namespace std;

/*GOBAL VARIABLE*/

ros::NodeHandle  nh;

int A_Right = 28;
int B_Right = 29;
int pwmPinR=23; //24
int dirPin_r=8;
bool AState_Right,BState_Right,flag_Right=0,dir_r=0;
int pos_r=0,newTick_Right=0,lastTick_Right=0,delTick_Right=0,cpr=2400;
double ITerm_R,lastInput_R,rpm_r=10,setVel_Right=6,kp_r=1,ki_r=0.5,kd_r=0.25;
int pwmVal_Right,outMin=-75,outMax=75;

int A_Left = 15;
int B_Left = 16;
int pwmPinL=24; //24
int dirPin_l=9;
bool AState_Left,BState_Left,flag_Left=0,dir_l=0;
int pos_l=0,newTick_Left=0,lastTick_Left=0,delTick_Left=0;
double ITerm_L,lastInput_L,rpm_l=10,setVel_Left=0,kp_l=1,ki_l=0.5,kd_l=0.25;
int pwmVal_Left;

unsigned long lastTime=0,now=0,dt=10000;
int flags=0;

char *rosSrvrIp = "192.168.31.124";

interfacing::Velocity data;

ros::Publisher Feedback("Feedback", &data);



void callback(const interfacing::Velocity &msg)
  {
  	setVel_Left = msg.leftwheel;
		setVel_Right =  msg.rightwheel;
  }

ros::Subscriber<interfacing::Velocity> sub("Velocity", callback );

int PID(double Input,double Setpoint, double *ITerm, double *perviousError, double *DTerm, double kp,double ki,double kd){
	  int Output;
	  double error = Setpoint - Input;
	  double dInput = (Input - *lastInput);
      *ITerm+= (ki * error);

      if(*ITerm > outMax) *ITerm= outMax;
      else if(*ITerm < outMin) *ITerm= outMin;

      /*Compute PID Output*/
      Output = (kp * error + *ITerm- kd * dInput);

      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;

      /*Remember some variables for next time*/
      *lastInput = Input;
      cout<<"+++++"<<Output<<"+++++"<<endl;
			return Output;
}

void Enc_A_Right(){
	AState_Right = digitalRead(A_Right);
	BState_Right = digitalRead(B_Right);


		if(pos_r>=cpr){pos_r=1;}
		else if(pos_r<=-cpr){pos_r=-1;}

		if(AState_Right==true)
		 	pos_r+=(BState_Right==LOW)?(1):(-1);
	   	else
			pos_r+=(BState_Right==HIGH)?(1):(-1);

}

void Enc_B_Right(){
	AState_Right = digitalRead(A_Right);
	BState_Right = digitalRead(B_Right);

		if(pos_r>=cpr){pos_r=1;}
		else if(pos_r<=-cpr){pos_r=-1;}

		if(BState_Right==true)
		 	pos_r+=(AState_Right==HIGH)?(1):(-1);
	   	else
			pos_r+=(AState_Right==LOW)?(1):(-1);

			//cout<<pos_r<<endl;

}

void Enc_A_Left(){
	AState_Left = digitalRead(A_Left);
	BState_Left = digitalRead(B_Left);


		if(pos_l>=cpr){pos_l=1;}
		else if(pos_l<=-cpr){pos_l=-1;}

		if(AState_Left==true)
		 	pos_l+=(BState_Left==LOW)?(1):(-1);
	   	else
			pos_l+=(BState_Left==HIGH)?(1):(-1);

}

void Enc_B_Left(){
	AState_Left = digitalRead(A_Left);
	BState_Left = digitalRead(B_Left);

		if(pos_l>=cpr){pos_l=1;}
		else if(pos_l<=-cpr){pos_l=-1;}

		if(BState_Left==true)
		 	pos_l+=(AState_Left==HIGH)?(1):(-1);
	   	else
			pos_l+=(AState_Left==LOW)?(1):(-1);

			//cout<<pos_l<<endl;

}

void safeStop(int a){
  softPwmWrite(pwmPinR,0);
  softPwmWrite(pwmPinL,0);
  softPwmWrite(pwmPinR,0);
  softPwmWrite(pwmPinL,0);
  flags=1;
  cout<<"+++++++++++++++++";
}

int main(){

	wiringPiSetup();
	pinMode(A_Left, INPUT);
	pinMode(B_Left, INPUT);
	pinMode(A_Right, INPUT);
	pinMode(B_Right, INPUT);
	pinMode(dirPin_r, OUTPUT);
	pinMode(dirPin_l, OUTPUT);
	pullUpDnControl(A_Right, PUD_UP);
	pullUpDnControl(B_Right, PUD_UP);
	pullUpDnControl(A_Left, PUD_UP);
	pullUpDnControl(B_Left, PUD_UP);
	pullUpDnControl(23, PUD_DOWN);
	pullUpDnControl(24, PUD_DOWN);
	wiringPiISR(A_Left,INT_EDGE_BOTH,Enc_A_Left);
	wiringPiISR(B_Left,INT_EDGE_BOTH,Enc_B_Left);
	wiringPiISR(A_Right,INT_EDGE_BOTH,Enc_A_Right);
	wiringPiISR(B_Right,INT_EDGE_BOTH,Enc_B_Right);
	softPwmCreate(pwmPinR, 0, 100);
	softPwmCreate(pwmPinL, 0, 100);
	signal(SIGINT,safeStop);

	nh.initNode(rosSrvrIp);
	nh.subscribe(sub);
	nh.advertise(Feedback);

	while(flags==0)
		{

			  unsigned long now = micros();
			  if(now-lastTime >= dt)
			  {
					lastTime = now;
					newTick_Left  = pos_l;
					newTick_Right  = pos_r;

					delTick_Left  = newTick_Left-lastTick_Left;
					delTick_Right  = newTick_Right-lastTick_Right;

				if(delTick_Left>0 && newTick_Left<0 && delTick_Left>=cpr/2)
				{
				  delTick_Left = (cpr-delTick_Left)*(-1);
				}
				else if(delTick_Left<0 && newTick_Left>0 && delTick_Left<cpr*(-1/2))
			  {
				  delTick_Left = (cpr+delTick_Left);
				}

				if(delTick_Right>0 && newTick_Right<0 && delTick_Right>=cpr/2)
				{
				  delTick_Right = (cpr-delTick_Right)*(-1);
				}
				else if(delTick_Right<0 && newTick_Right>0 && delTick_Right<cpr*(-1/2))
			  {
				  delTick_Right = (cpr+delTick_Right);
				}

				rpm_l = delTick_Left*0.135;
				rpm_r = delTick_Right*0.135;

				pwmVal_Left=PID(rpm_l,setVel_Left,&ITerm_L,&lastInput_L,kp_l,ki_l,kd_l);
				pwmVal_Right=PID(rpm_r,setVel_Right,&ITerm_R,&lastInput_R,kp_r,ki_r,kd_r);
				//pwmVal=-20;
				dir_l=(pwmVal_Left>=0)?1:0;
				dir_r=(pwmVal_Right>=0)?0:1;

				digitalWrite(dirPin_l,dir_l);
				digitalWrite(dirPin_r,dir_r);

				softPwmWrite(pwmPinL,abs(pwmVal_Left));
				softPwmWrite(pwmPinR,abs(pwmVal_Right));

				cout<<pos_l<<"\t"<<rpm_l<<"\t"<<delTick_Left<<"\t"<<pwmVal_Left<<"------Left-----"<<endl;
				cout<<pos_r<<"\t"<<rpm_r<<"\t"<<delTick_Right<<"\t"<<pwmVal_Right<<"------Right-----"<<endl;
				lastTick_Left = newTick_Left;
				lastTick_Right = newTick_Right;

				lastTime = micros();
				data.leftwheel = rpm_l;
				data.rightwheel = rpm_r;
				Feedback.publish(&data);
				nh.spinOnce();
        usleep(99999);

			 }
       pwmVal_Left=PID(rpm_l,setVel_Left,&ITerm_L,&lastInput_L,kp_l,ki_l,kd_l);
       pwmVal_Right=PID(rpm_r,setVel_Right,&ITerm_R,&lastInput_R,kp_r,ki_r,kd_r);
       //pwmVal=-20;
       dir_l=(pwmVal_Left>=0)?1:0;
       dir_r=(pwmVal_Right>=0)?0:1;

       digitalWrite(dirPin_l,dir_l);
       digitalWrite(dirPin_r,dir_r);

       softPwmWrite(pwmPinL,abs(pwmVal_Left));
       softPwmWrite(pwmPinR,abs(pwmVal_Right));


		 }
return 0;
}
