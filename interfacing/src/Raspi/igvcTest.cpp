/******************
* Name: igvcTest.cpp
* Description: Calculate velocity
********************/

/*Header Files*/

#include<iostream>
#include"wiringPi.h"
#include<stdio.h>
#include<stdlib.h>
#include"softPwm.h"
#include<math.h>
#include<signal.h>
#include<ros.h>
#include<interfacing/Velocity.h>
#include<unistd.h>

using namespace std;

/*Define*/

#define A_Right  28
#define B_Right  29
#define pwmPinR  23 
#define dirPin_r  8

#define A_Left   15
#define B_Left   16
#define pwmPinL  24 
#define dirPin_l  9

/*GOBAL ROS VARIABLE*/

ros::NodeHandle  nh;
interfacing::Velocity data;
char *rosSrvrIp = "192.168.31.124";

/*GOBAL VARIABLE*/

bool AState_Right, BState_Right, dir_r = 0;
long long int pos_r = 0, newTick_Right, lastTick_Right = 0, delTick_Right; 
double prvErr_R = 0, intgtr_R = 0, rpm_r, setVel_Right = 0, kp_r = .35, ki_r = 1.2, kd_r = 0;
int pwmVal_Right;

bool AState_Left, BState_Left, dir_l=0;
long long int pos_l = 0, newTick_Left, lastTick_Left = 0, delTick_Left;
double prvErr_L = 0, intgtr_L = 0, rpm_l, setVel_Left = 0, kp_l = .35, ki_l = 1.2, kd_l = 0;
int pwmVal_Left;

unsigned long long int  lastTime = 0, now = 0;

/*CONSTANTS*/

float cpr = 48000.0;
float circumference = 0.38 * 22 / 7.0;
unsigned long long int dt = 100000;

void callback(const interfacing::Velocity &msg)
{
	setVel_Left  =  msg.leftwheel;
	setVel_Right =  msg.rightwheel;
}

int PID(double currentRpm, double setRpm, double *previousError,double *integrator, double kp,double ki,double kd)
{
	double h = 0;
	double i = 0;
	double error = (setRpm) - (currentRpm);
	
	if (abs(error) < 0.5)
		error = 0;

	double proportional = error;
	h = error - (*previousError);
	double differentiator = h / 0.1;
	i = error * 0.1;
	*integrator = *integrator+i;
	double pid=(kp * proportional)+(ki * (*integrator))+(kd * differentiator);
	*previousError = error;
	return pid;
}

void Enc_A_Right()
{
	AState_Right = digitalRead(A_Right);
	BState_Right = digitalRead(B_Right);

	if(AState_Right==true)
	 	pos_r+=(BState_Right==LOW)?(1):(-1);
	else
		pos_r+=(BState_Right==HIGH)?(1):(-1);
}

void Enc_B_Right()
{
	AState_Right = digitalRead(A_Right);
	BState_Right = digitalRead(B_Right);

	if(BState_Right==true)
	 	pos_r+=(AState_Right==HIGH)?(1):(-1);
   	else
		pos_r+=(AState_Right==LOW)?(1):(-1);
}

void Enc_A_Left()
{
	AState_Left = digitalRead(A_Left);
	BState_Left = digitalRead(B_Left);

	if(AState_Left==true)
	 	pos_l+=(BState_Left==LOW)?(1):(-1);
  	else
		pos_l+=(BState_Left==HIGH)?(1):(-1);
}

void Enc_B_Left()
{
	AState_Left = digitalRead(A_Left);
	BState_Left = digitalRead(B_Left);

	if(BState_Left==true)
	 	pos_l+=(AState_Left==HIGH)?(1):(-1);
   	else
		pos_l+=(AState_Left==LOW)?(1):(-1);
}

void safeStop(int a)
{
	softPwmWrite(pwmPinR,0);
	softPwmWrite(pwmPinL,0);
	usleep(100000);
	cout<<"\n+++++++++++++++++";
	softPwmWrite(pwmPinR,0);
	softPwmWrite(pwmPinL,0);
	usleep(100000);
	cout<<"\n+++++++++++++++++";
}

int main()
{
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
	softPwmCreate(pwmPinR, 0, 255);
	softPwmCreate(pwmPinL, 0, 255);
	signal(SIGINT,safeStop);

	nh.initNode(rosSrvrIp);
  	ros::Publisher Feedback("Feedback", &data);
  	ros::Subscriber<interfacing::Velocity> sub("Velocity", callback) ;
	nh.subscribe(sub);
	nh.advertise(Feedback);

	now = micros();
	
	if(now - lastTime >= dt)
	{
		lastTime = micros();

		lastTick_Left  = newTick_Left;
		lastTick_Right = newTick_Right;

		newTick_Left  = pos_l;
		newTick_Right = pos_r;

		delTick_Left  = newTick_Left  - lastTick_Left;
		delTick_Right = newTick_Right - lastTick_Right;

		rpm_l = (delTick_Left  / cpr) * circumference / 0.1;
		rpm_r = (delTick_Right / cpr) * circumference / 0.1;

		pwmVal_Left  = PID(rpm_l, setVel_Left, &prvErr_L, &intgtr_L, kp_l, ki_l, kd_l);
		pwmVal_Right = PID(rpm_r, setVel_Right, &prvErr_R, &intgtr_R, kp_r, ki_r, kd_r);

		dir_l = (pwmVal_Left  >= 0) ? 1 : 0;
		dir_r = (pwmVal_Right >= 0) ? 0 : 1;

		digitalWrite(dirPin_l, dir_l);
		digitalWrite(dirPin_r, dir_r);

		softPwmWrite(pwmPinL, abs(pwmVal_Left));
		softPwmWrite(pwmPinR, abs(pwmVal_Right));

		cout<<rpm_l<<"\t"<<setVel_Left<<"\t"<<pwmVal_Left<<"-----Left----"<<endl;
		cout<<rpm_r<<"\t"<<setVel_Right<<"\t"<<pwmVal_Right<<"-----Right----"<<endl;

		data.leftwheel = rpm_l;
		data.rightwheel = rpm_r;
		Feedback.publish(&data);
		nh.spinOnce();
	}
	return 0;
}
