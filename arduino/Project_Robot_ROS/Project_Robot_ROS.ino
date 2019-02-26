// DEFINE CONSTANTS
// DEBUG
//#define DEBUG

// PINS
#define PWM_MOVE 5
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
#define LEFT_HALL_THRESH 1023
#define RIGHT_HALL_THRESH 1023
#define DUTY_MAX 105
#define DUTY_MIN 100
#define BRAKE_DUTY 200

// PHYSICS CONSTANTS
#define WHEEL_CIRCUMFERENCE 518.36
#define PULSE_WEIGHT 0.5
#define PULSE_DIST 34.56
#define ROBOT_HALF_WIDTH 283.5

// ROS Package and std_msg format
#include <ros.h>
#include <ros/time.h>
//#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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
bool leftReverse = false;
bool rightReverse = false;

// Functions
void moveForward();
void moveTurn(float angle);
void moveStop();
void velCallback( const geometry_msgs::Twist& vel);
float getHeading(float offset);
void pollHallPins();
void updateOdom(int turn);
void publishOdom();

// ROS Subscriber and Publisher
ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel" , velCallback);
//nav_msgs::Odometry odomMsg;
//ros::Publisher pub_odom("odom", &odomMsg);
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//=======================================================
// ROBOT MOVEMENT
void moveForward(float vel_x) {
  analogWrite(BRAKE_PIN, 0);
  int duty = (vel_x) * ((DUTY_MAX - DUTY_MIN)/2) + DUTY_MIN;
  analogWrite(PWM_MOVE, DUTY_MIN);
}

void moveTurn(float angle) {
  analogWrite(BRAKE_PIN, 0);
  if (angle > 0) {
    //Turn Left
    leftReverse = true;
    digitalWrite(LEFT_REVERSE, HIGH);
    analogWrite(PWM_MOVE, DUTY_MIN);
  } else if (angle < 0) {
    // Turn Right
    rightReverse = true;
    digitalWrite(RIGHT_REVERSE, HIGH);
    analogWrite(PWM_MOVE, DUTY_MIN);
  }
}

void moveStop() {
  analogWrite(PWM_MOVE, 0);
  delay(50);
  leftReverse = false;
  rightReverse = false;
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
  analogWrite(BRAKE_PIN, BRAKE_DUTY);

}

//=======================================================
// Subscriber Callback Function
void velCallback( const geometry_msgs::Twist& vel) {
  moveStop();
  float vel_x = vel.linear.x; // Moving Forward Speed
  float ang_z = vel.angular.z; // Angle to Turn
  if (vel_x != 0) {
    moveForward(vel_x);
  } else if (ang_z != 0) {
    moveTurn(ang_z);
  }
}

//=======================================================
// IMU
float getHeading(float offset){
  imu.readMag();
  
  float heading;
  float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal;
  Xm_off = imu.mx + 306.203686; //X-axis combined bias (Non calibrated data - bias)
  Ym_off = imu.my + 1232.855613; //Y-axis combined bias (Default: substracting bias)
  Zm_off = imu.mz - 695.984327; //Z-axis combined bias

  Xm_cal =  0.018511*Xm_off + 0.001908*Ym_off + 0.000262*Zm_off - 16; //X-axis correction for combined scale factors (Default: positive factors)
  Ym_cal =  0.001908*Xm_off + 0.018923*Ym_off - 0.000177*Zm_off - 42; //Y-axis correction for combined scale factors
  Zm_cal =  0.000262*Xm_off - 0.000177*Ym_off + 0.018239*Zm_off; //Z-axis correction for combined scale factors

  heading = atan2(Xm_cal, Ym_cal);  
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
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.begin();
  headingOffset = getHeading(0);

  #ifdef DEBUG
    Serial.begin(9600);
  #else
    // ROS
    robot.initNode();
    robot.subscribe(sub_cmd_vel);   //cmd_vel
    broadcaster.init(robot);        //tf
    //robot.advertise(pub_odom);      //odom
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
  #else
    // SpinOnce Check for instructions
    robot.spinOnce();
  #endif

  theta = getHeading(headingOffset);
  
  // Check Hall pins of both wheels for Rising Edge
  pollHallPins();

  // Publish Odometry
  publishOdom();
}


void pollHallPins() {
  // Left wheel polling
  left_hall_val = analogRead(LEFT_HALL_IN);
  //Serial.println(left_hall_val);
  if (left_hall_val >= LEFT_HALL_THRESH) {
    if (left_wasLowLevel == true) {
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
      right_wasLowLevel = false;  // Reset edge detector

      // Odometry Update
      updateOdom();
    }
  } else {
    right_wasLowLevel = true;
  }
}

// Function turn param takes +1 or -1 depending on left wheel or right wheel poll
void updateOdom() {
  // Moving Forward
  if (!leftReverse && !rightReverse) {
    // Moving Forward by one pulse
    x = x + PULSE_WEIGHT*PULSE_DIST*cos(theta);
    y = y + PULSE_WEIGHT*PULSE_DIST*sin(theta);
  }
}


void publishOdom() {
  #ifdef DEBUG
    Serial.print("x: ");
    Serial.print(x);
    Serial.print("   y: ");
    Serial.print(y);
    Serial.print("   theta: ");
    Serial.println(theta);

  #else
    ros::Time current_time = robot.now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
    
    // Broadcast to tf
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "chassis";

    t.transform.translation.x = x/1000;
    t.transform.translation.y = y/1000;

    t.transform.rotation = odom_quat;

    broadcaster.sendTransform(t);

    /*
    // Publish to odom
    odomMsg.header.stamp = current_time;
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "chassis";

    odomMsg.pose.pose.position.x = x/1000;
    odomMsg.pose.pose.position.y = y/1000;
    odomMsg.pose.pose.position.z = 0.0;
    odomMsg.pose.pose.orientation = odom_quat;

    odomMsg.twist.twist.linear.x = 0.1;
    odomMsg.twist.twist.linear.y = -0.1;
    odomMsg.twist.twist.angular.z = 0.1;

    pub_odom.publish(&odomMsg);
    */
  #endif

}
