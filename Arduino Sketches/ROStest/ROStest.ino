/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Int32.h>

int state = 1;
int brightness = 0;
ros::NodeHandle  nh;

const int whiteled = 11;
const int greenled = 10;
int led = whiteled;

void setup();
void offAll();

void brightnessChange( const std_msgs::Int32& msg){
  offAll();
  brightness = msg.data;
}

void stateChange( const std_msgs::Int32& msg){
  offAll();
  state = msg.data;
}
void ledChange( const std_msgs::Int32& msg){
  offAll();
  if (msg.data == 1){
    led = whiteled;
  }else{
    led = greenled;
  }
}

ros::Subscriber<std_msgs::Int32> sub("brightness", &brightnessChange );
ros::Subscriber<std_msgs::Int32> sub2("state", &stateChange );
ros::Subscriber<std_msgs::Int32> sub3("led", &ledChange );


void setup()
{ 
  pinMode(whiteled, OUTPUT);
  pinMode(greenled, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub2);
  nh.subscribe(sub3);

}
void offAll(){
  analogWrite(whiteled, 0);
  analogWrite(greenled, 0);
}
void loop()
{  
  nh.spinOnce();
  if (state == 1){
    analogWrite(led, brightness);
    delay(300);
    analogWrite(led, 0);
    delay(300);
  }else if(state == 2){
    analogWrite(led, brightness);
  }else{
    analogWrite(led, 0);
  }
  delay(1);
}
