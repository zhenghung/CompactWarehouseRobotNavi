
#include "ros.h"

#include "geometry_msgs/Twist.h"

float x; 

ros::NodeHandle nh;

void velCallback(  const geometry_msgs::Twist& vel)
{
     x = vel.linear.x - 1.0; // I CAN USE VEL AS I WANT
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , velCallback);

void setup() {
     nh.initNode();
     nh.subscribe(sub);
}

void loop() {
     nh.spinOnce();
     delay(10);
}
