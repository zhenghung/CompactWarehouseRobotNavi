// DEFINE CONSTANTS
// PINS
#define PWM_MOVE 5
#define LEFT_HALL_IN A0
#define RIGHT_HALL_IN A1
#define LEFT_REVERSE 8
#define RIGHT_REVERSE 9

// CONSTANTS
#define LEFT_HALL_THRESH 100
#define RIGHT_HALL_THRESH 100
#define DUTY_BEGIN 100

// PHYSICS CONSTANTS
#define WHEEL_CIRCUMFERENCE 518.36
#define ROBOT_HALF_WIDTH 280
// Change in bearing = (90*L*pi)/(pi*HalfWidth*180) = 0.115267
#define THETA_DELTA 0.115267

// ROS Package and std_msg format
#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


//=======================================================
// Globals
ros::NodeHandle robot;

// Rising Edge
bool left_wasLowLevel = false;
int left_hall_val;
bool right_wasLowLevel=false;
int right_hall_val;

// Odometry
unsigned long leftHallCount = 0;
unsigned long rightHallCount = 0;
volatile float x = 0;
volatile float y = 0;
volatile float theta = 0;

// Functions
void moveForward();
void moveTurn(float angle);
void moveStop();
void velCallback( const geometry_msgs::Twist& vel);
void pollHallPins();
void updateOdom(int turn);

// ROS Subscriber and Publisher
//ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , velCallback);
//ros::Publisher pub_odom("odom", 50);

//=======================================================
// ROBOT MOVEMENT
void moveForward(){
  analogWrite(PWM_MOVE, DUTY_BEGIN);
}

void moveTurn(float angle){
  if (angle>0){
    //Turn Left
    digitalWrite(LEFT_REVERSE, HIGH);
    analogWrite(PWM_MOVE, DUTY_BEGIN);
  }else if(angle<0){
    // Turn Right
    digitalWrite(RIGHT_REVERSE, HIGH);
    analogWrite(PWM_MOVE, DUTY_BEGIN);
  }
}

void moveStop(){
  analogWrite(PWM_MOVE, 0);
  delay(50);
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
}

//=======================================================
// Subscriber Callback Function
void velCallback( const geometry_msgs::Twist& vel){
  moveStop();
  float vel_x = vel.linear.x; // Moving Forward Speed
  float ang_z = vel.angular.z; // Angle to Turn
  if (vel_x!=0){
    moveForward();
  }else if (ang_z!=0){
    moveTurn(ang_z);
  }
}

//=======================================================
// Main Setup
void setup() {
  // Setup Pins
  pinMode(LEFT_HALL_IN, INPUT);
  pinMode(RIGHT_HALL_IN, INPUT);
  pinMode(PWM_MOVE, OUTPUT);
  pinMode(LEFT_REVERSE, OUTPUT);
  pinMode(RIGHT_REVERSE, OUTPUT);
  moveStop();

  // ROS
  //robot.initNode();
  //robot.subscribe(sub_cmd_vel);
  //robot.advertise(pub_odom);

}

void loop() {
  // SpinOnce Check for instructions
  //robot.spinOnce();
  if (Serial.available()>0){
    char cmd = Serial.read();
    switch (cmd) {
      case 'w':
        moveForward();
        break;
      case 'a':
        moveTurn(1);
        break;
      case 'd':
        moveTurn(-1);
        break;
      default:
        moveStop();
        break;
    }

  }

  // Check Hall pins of both wheels for Rising Edge
  pollHallPins();
  
  // Publish Odometry
  publishOdom();
}


void pollHallPins(){
  // Left wheel polling
  left_hall_val = analogRead(LEFT_HALL_IN);
  //Serial.println(left_hall_val);
  if (left_hall_val >= LEFT_HALL_THRESH) {
    if (left_wasLowLevel == true) {
      left_wasLowLevel = false;  // Reset edge detector

      // Odometry Update
      updateOdom(1); 
    }
  }else {
    left_wasLowLevel = true;
  }
  

  // Right Wheel Polling
  right_hall_val = analogRead(RIGHT_HALL_IN);
  //Serial.println(right_hall_val);
  if (right_hall_val >= RIGHT_HALL_THRESH) {
    if (right_wasLowLevel == true) {
      right_wasLowLevel = false;  // Reset edge detector

      // Odometry Update
      updateOdom(-1);
    }
  }else {
    right_wasLowLevel = true;
  }
}

// Function turn param takes +1 or -1 depending on left wheel or right wheel poll
void updateOdom(int turn) {
  if (turn>0){
    leftHallCount++;
    float prev_theta = theta;
    
    if (leftReverse){
      theta = theta - THETA_DELTA;
      y = y - HalfWidth*(sin(theta) - sin(prev_theta));
    }else{
      theta = theta + THETA_DELTA;
      y = y - HalfWidth*(sin(theta) - sin(prev_theta));
    x = x + HalfWidth*(cos(prev_theta) - cos(theta));
    
  }else{
    rightHallCount++;
    float prev_theta = theta;
    
    if (rightReverse){
      theta = theta + THETA_DELTA;
      y = y - ROBOT_HALF_WIDTH*(sin(theta) - sin(prev_theta));
    }else{
      theta = theta - THETA_DELTA;
      y = y + ROBOT_HALF_WIDTH*(sin(theta) - sin(prev_theta));
    }
    x = x - ROBOT_HALF_WIDTH*(cos(prev_theta) - cos(theta));
  }

}


void publishOdom(){
  /*
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom_quat = tf::createQuaternionMsgFromYaw(theta);
  odom.pose.pose.orientation = odom_quat;

  pub_odom.publish(&odom);
  */
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" , y: ");
  Serial.print(y);
  Serial.print(" , theta: ");
  Serial.println(theta);
  
}