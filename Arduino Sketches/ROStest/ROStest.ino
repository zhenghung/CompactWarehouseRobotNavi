/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

int state = 0;
ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg){
//  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  state = !state;
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  if (state == 1){
    digitalWrite(13, HIGH-digitalRead(13));
    delay(300);
    digitalWrite(13, LOW-digitalRead(13));
    delay(300);
  }else{
    digitalWrite(13, LOW);
  }
  delay(1);
}
