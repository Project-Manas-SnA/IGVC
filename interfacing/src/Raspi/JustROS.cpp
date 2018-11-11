
#include<ros.h>
#include <std_msgs/Int8.h>
#include<stdio.h>
#include <unistd.h>
#include"wiringPi.h"

ros::NodeHandle  nh;


char *rosSrvrIp = "192.168.31.124";

std_msgs::Int8 data1;
//std_msgs::Int8 data2;

ros::Publisher Feedback1("Feedback1", &data1);
//ros::Publisher Feedback2("Feedback2", &data2);



int main()
{
  nh.initNode(rosSrvrIp);
  nh.advertise(Feedback1);
  //nh.advertise(Feedback2);
  while (1)
  {
    sleep(1);
    sleep(1);
    //data.leftwheel
    Feedback1.publish(&data1);
    //Feedback2.publish(&data2);
    nh.spinOnce();
  }

}
