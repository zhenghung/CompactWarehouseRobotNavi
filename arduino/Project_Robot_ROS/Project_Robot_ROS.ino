// DEFINE CONSTANTS
// DEBUG
//#define DEBUG

// PINS
#define PWM_MOVE 6
#define LEFT_HALL_IN A8
#define RIGHT_HALL_IN A9
#define LEFT_REVERSE 2
#define RIGHT_REVERSE 3
#define BRAKE_PIN 11

// IMU
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

// CONSTANTS
#define LEFT_HALL_THRESH 100
#define RIGHT_HALL_THRESH 100
#define DUTY_MAX 105
#define DUTY_MIN 105
#define BRAKE_DUTY 255
#define MAG_X_OFFSET -900 
#define MAG_Y_OFFSET -300
#define MAG_Z_OFFSET 1100
#define ANG_THRESH 0.4


// PHYSICS CONSTANTS
#define WHEEL_CIRCUMFERENCE 518.36
#define PULSE_WEIGHT 0.5
#define PULSE_DIST 34.56
#define PULSE_INCREMENT (PULSE_WEIGHT*PULSE_DIST)
#define ROBOT_HALF_WIDTH 283.5

// ROS Package and std_msg format
#include <ros.h>
#include <ros/time.h>
//#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
// #include <tf/tf.h>
// #include <tf/transform_broadcaster.h>
#include <imu_read/imu_read.h>

// IMU Package
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>


//=======================================================
// Globals
ros::NodeHandle robot;
LSM9DS1 imu;

// Rising Edge
bool left_wasLowLevel = false;
int left_hall_val;
bool right_wasLowLevel = false;
int right_hall_val;

// Odometry
volatile float x = 0;
volatile float y = 0;
volatile float theta = 0;
volatile float headingOffset = 0;
bool leftReverse = false;     // State of the left relay
bool rightReverse = false;    // State of the right relay


// Functions
void moveForward();
void moveTurn(float angle);
void moveStop();
void velCallback( const geometry_msgs::Twist& vel);
float getHeading(float offset);
void pollHallPins();
void updateOdom();
void publishMsg();

// Robot Control
boolean turning = false;    // Rotating
boolean fwdOrBack = false;  // Moving Forward or in Reverse

// ROS Messages
// geometry_msgs::TransformStamped t;
// tf::TransformBroadcaster broadcaster;
imu_read::imu_read custom_msg;

// ROS Subscriber and Publisher
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , velCallback);
// nav_msgs::Odometry odomMsg;
ros::Publisher pub_custom("CompressedMsg", &custom_msg);

unsigned long last_stamp;

//=======================================================
// ROBOT MOVEMENT
void moveForward(float vel_x) {
  turning = false;
  fwdOrBack = true;
  last_stamp = millis();
  analogWrite(BRAKE_PIN, 0);
  leftReverse = false;
  rightReverse = false;
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
  analogWrite(PWM_MOVE, DUTY_MIN);
}

void moveReverse(float vel_x) {
  turning = false; 
  fwdOrBack = true;
  last_stamp = millis();
  analogWrite(BRAKE_PIN, 0);
  leftReverse = true;
  rightReverse = true;
  digitalWrite(LEFT_REVERSE, HIGH);
  digitalWrite(RIGHT_REVERSE, HIGH);
  analogWrite(PWM_MOVE, DUTY_MIN);
}

void moveTurn(float angle) {
  turning = true;
  fwdOrBack = false;
  last_stamp = millis();

  analogWrite(BRAKE_PIN, 0);
  if (angle > 0) {
    //Turn Left
    leftReverse = true;
    rightReverse = false;
    digitalWrite(LEFT_REVERSE, HIGH);
    digitalWrite(RIGHT_REVERSE, LOW);
    analogWrite(PWM_MOVE, DUTY_MIN);

  } else if (angle < 0) {
    // Turn Right
    leftReverse = false;
    rightReverse = true;
    digitalWrite(LEFT_REVERSE, LOW);
    digitalWrite(RIGHT_REVERSE, HIGH);
    analogWrite(PWM_MOVE, DUTY_MIN);
  }
}

void moveStop() {
  analogWrite(PWM_MOVE, 0);
  leftReverse = false;
  rightReverse = false;
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
}

void moveBrake(){
  turning = false;
  fwdOrBack = false;
  analogWrite(PWM_MOVE, 0);
  analogWrite(BRAKE_PIN, BRAKE_DUTY);
  leftReverse = false;
  rightReverse = false;
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
}

//=======================================================
// Subscriber Callback Function
void velCallback( const geometry_msgs::Twist& vel) {
  moveStop();
  float vel_x = vel.linear.x; // Moving Forward Speed
  float ang_z = vel.angular.z; // Angle to Turn

//  if (abs(vel_x)/MAX_VEL_X > abs(ang_z)/MAX_ANG_Z) {
//    ang_z = 0;
//  } else if (abs(vel_x) < abs(ang_z)) {
//    vel_x = 0;
//  }
  if (vel_x != 0 && abs(ang_z) <= ANG_THRESH){
    ang_z = 0;
  }
  if (ang_z != 0) {
    if (fwdOrBack && (abs(millis()-last_stamp)<500)){
      moveBrake();
      fwdOrBack = true;
    } else {
      moveTurn(ang_z);
    }
  } else if (vel_x != 0) {
    if (turning && (abs(millis()-last_stamp)<500)) {
      moveBrake();
      turning = true;
    } else {
      if (vel_x > 0){
        moveForward(vel_x);
      }else{
        moveReverse(-vel_x);
      }
    }

  } else if (vel_x == 0 && ang_z == 0){
    moveBrake();
  }
  robot.spinOnce();
}

//=======================================================
// IMU
float getHeading(float offset){
  // imu.readMag();
  
  float heading;
  heading = atan2(imu.my + MAG_Y_OFFSET, imu.mx + MAG_X_OFFSET);  

  heading -= offset;

  if (heading > PI) {
    heading -= (2 * PI);
  } else if (heading < -PI) {
    heading += (2 * PI);
  }
  
  return heading;
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
  pinMode(BRAKE_PIN, OUTPUT);
  moveStop();

  // Setup IMU
  // imu.settings.device.commInterface = IMU_MODE_I2C;
  // imu.settings.device.mAddress = LSM9DS1_M;
  // imu.settings.device.agAddress = LSM9DS1_AG;
  // imu.begin();
  // headingOffset = getHeading(0);


  #ifdef DEBUG
    Serial.begin(9600);
  #else
    // ROS
    robot.initNode();
    robot.subscribe(sub_cmd_vel);     //cmd_vel

    //broadcaster.init(robot);        //tf
    robot.advertise(pub_custom);      //custom
  #endif

}

void loop() {

  #ifdef DEBUG
    if (Serial.available() > 0) {
      char cmd = Serial.read();
      switch (cmd) {
        case 'w':
          moveForward(0.1);
          break;
        case 'a':
          moveTurn(0.1);
          break;
        case 's':
          moveBrake();
          break;
        case 'd':
          moveTurn(-0.1);
          break;
        case 'x':
          moveReverse(-0.1);
          break;
        default:
          moveStop();
          break;
      }
    }
  #else
    // SpinOnce Check for instructions
    robot.spinOnce();
  #endif

  // theta = getHeading(headingOffset);

  
  // Check Hall pins of both wheels for Rising Edge
  pollHallPins();

  // Publish Odometry
  publishMsg();
  robot.spinOnce();

}


void pollHallPins() {
  // Left wheel polling
  left_hall_val = analogRead(LEFT_HALL_IN);
  //Serial.println(left_hall_val);
  if (left_hall_val >= LEFT_HALL_THRESH) {
    if (left_wasLowLevel == true) {
      //Serial.println("left");
      left_wasLowLevel = false;  // Reset edge detector

      // Odometry Update
      updateOdom();
    }
  } else {
    left_wasLowLevel = true;
  }


  // Right Wheel Polling
  right_hall_val = analogRead(RIGHT_HALL_IN);
  //Serial.println(right_hall_val);
  if (right_hall_val >= RIGHT_HALL_THRESH) {
    if (right_wasLowLevel == true) {
      //Serial.println("         right");
      right_wasLowLevel = false;  // Reset edge detector

      // Odometry Update
      updateOdom();
    }
  } else {
    right_wasLowLevel = true;
  }
}

// Function increments distance travelled by half a pulse for each from left wheel and right wheel poll
void updateOdom() {
  // Moving Forward
  if (!leftReverse && !rightReverse) {
    // Moving Forward by one pulse
    x = x + PULSE_INCREMENT*cos(theta);
    y = y + PULSE_INCREMENT*sin(theta);
  } else if (leftReverse && rightReverse) {
    // Moving Backwards by one pulse
    x = x - PULSE_INCREMENT*cos(theta);
    y = y - PULSE_INCREMENT*sin(theta);
  }
}
void publishMsg(){
  #ifdef DEBUG

    Serial.print("x: ");
    Serial.print(x);
    Serial.print("   y: ");
    Serial.print(y);
    Serial.print("   theta: ");
    Serial.println(theta);

  #else

    // Get IMU Values
    // imu.readGyro();
    // imu.readAccel();
    
    ros::Time current_time = robot.now();

    custom_msg.header.stamp = current_time;
    custom_msg.x = x/1000;
    custom_msg.y = y/1000;
    custom_msg.theta = theta;
    // custom_msg.gx = imu.calcGyro(imu.gy);
    // custom_msg.gy = -imu.calcGyro(imu.gx);
    // custom_msg.gz = -imu.calcGyro(imu.gz);
    // custom_msg.mx = imu.calcMag(imu.my + MAG_Y_OFFSET);
    // custom_msg.my = -imu.calcMag(imu.mx + MAG_X_OFFSET);
    // custom_msg.mz = -imu.calcMag(imu.mz + MAG_Z_OFFSET);
    // custom_msg.ax = imu.calcAccel(imu.ay);
    // custom_msg.ay = -imu.calcAccel(imu.ax);
    // custom_msg.az = -imu.calcAccel(imu.az);

    pub_custom.publish(&custom_msg);

  #endif
}

// void publishOdom() {
//   #ifdef DEBUG
//     Serial.print("x: ");
//     Serial.print(x);
//     Serial.print("   y: ");
//     Serial.print(y);
//     Serial.print("   theta: ");
//     Serial.println(theta);

//   #else
//     ros::Time current_time = robot.now();
//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
    
//     // Broadcast to tf
//     t.header.stamp = current_time;
//     t.header.frame_id = "odom";
//     t.child_frame_id = "chassis";

//     t.transform.translation.x = x/1000;
//     t.transform.translation.y = y/1000;

//     t.transform.rotation = odom_quat;

//     broadcaster.sendTransform(t);

//     /*
//     // Publish to odom
//     odomMsg.header.stamp = current_time;
//     odomMsg.header.frame_id = "odom";
//     odomMsg.child_frame_id = "chassis";

//     odomMsg.pose.pose.position.x = x/1000;
//     odomMsg.pose.pose.position.y = y/1000;
//     odomMsg.pose.pose.position.z = 0.0;
//     odomMsg.pose.pose.orientation = odom_quat;

//     odomMsg.twist.twist.linear.x = 0.1;
//     odomMsg.twist.twist.linear.y = -0.1;
//     odomMsg.twist.twist.angular.z = 0.1;

//     pub_odom.publish(&odomMsg);
//     */

//   #endif

// }
