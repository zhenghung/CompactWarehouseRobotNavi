// DEFINE CONSTANTS
// DEBUG
//#define DEBUG

// PINS
#define PWM_MOVE 5
#define LEFT_HALL_IN A8
#define RIGHT_HALL_IN A9
#define LEFT_REVERSE 8
#define RIGHT_REVERSE 9

// CONSTANTS
#define LEFT_HALL_THRESH 1023
#define RIGHT_HALL_THRESH 1023
#define DUTY_MAX 105
#define DUTY_MIN 95

// PHYSICS CONSTANTS
#define WHEEL_CIRCUMFERENCE 518.36
#define ROBOT_HALF_WIDTH 283.5
// Change in bearing = (90*L*pi)/(pi*HalfWidth*180) = 0.115267
#define THETA_DELTA 0.115267

// ROS Package and std_msg format
#include <ros.h>
#include <ros/time.h>
//#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


//=======================================================
// Globals
ros::NodeHandle robot;

// Rising Edge
bool left_wasLowLevel = false;
int left_hall_val;
bool right_wasLowLevel = false;
int right_hall_val;

// Odometry
unsigned long leftHallCount = 0;
unsigned long rightHallCount = 0;
volatile float x = 0;
volatile float y = 0;
volatile float theta = 0;
bool leftReverse = false;
bool rightReverse = false;

// Functions
void moveForward();
void moveTurn(float angle);
void moveStop();
void velCallback( const geometry_msgs::Twist& vel);
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
  int duty = (vel_x) * ((DUTY_MAX - DUTY_MIN)/2) + DUTY_MIN;
  analogWrite(PWM_MOVE, duty);
}

void moveTurn(float angle) {
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
// Main Setup
void setup() {
  // Setup Pins
  pinMode(LEFT_HALL_IN, INPUT);
  pinMode(RIGHT_HALL_IN, INPUT);
  pinMode(PWM_MOVE, OUTPUT);
  pinMode(LEFT_REVERSE, OUTPUT);
  pinMode(RIGHT_REVERSE, OUTPUT);
  moveStop();

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
  #else
    // SpinOnce Check for instructions
    robot.spinOnce();
  #endif

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
      updateOdom(1);
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
      updateOdom(-1);
    }
  } else {
    right_wasLowLevel = true;
  }
}

// Function turn param takes +1 or -1 depending on left wheel or right wheel poll
void updateOdom(int turn) {
  if (turn > 0) {
    leftHallCount++;
    float prev_theta = theta;
    if (leftReverse) {
      theta = theta + THETA_DELTA;

      if (theta > 3.1416) {
        theta = theta - 6.2832;
      }
      if (prev_theta >= 0 && prev_theta < 1.5708) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta >= 1.5708 && prev_theta < 3.1416) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta >= -3.1416 && prev_theta < -1.5708) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta >= -1.5708 && prev_theta < 0) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
    } else {
      theta = theta - THETA_DELTA;
      if (theta < -3.1416) {
        theta = theta + 6.2832;
      }
      if (prev_theta <= 1.5708 && prev_theta > 0) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta <= 0 && prev_theta > -1.5708) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta <= -1.5708 && prev_theta > -3.1416) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta <= 3.1416 && prev_theta > 1.5708) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
    }
  } else {
    rightHallCount++;
    float prev_theta = theta;

    if (rightReverse) {
      theta = theta - THETA_DELTA;
      if (theta < -3.1416) {
        theta = theta + 6.2832;
      }
      if (prev_theta <= 0 && prev_theta > -1.5708) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta <= -1.5708 && prev_theta > -3.1416) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta <= 3.1416 && prev_theta > 1.5708) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta <= 1.5708 && prev_theta > 0) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
    } else {
      theta = theta + THETA_DELTA;
      if (theta > 3.1416) {
        theta = theta - 6.2832;
      }
      if (prev_theta >= 0 && prev_theta < 1.5708) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta >= 1.5708 && prev_theta < 3.1416) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y + abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta >= -3.1416 && prev_theta < -1.5708) {
        x = x - abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
      else if (prev_theta >= -1.5708 && prev_theta < 0) {
        x = x + abs(ROBOT_HALF_WIDTH * (sin(prev_theta) - sin(theta)));
        y = y - abs(ROBOT_HALF_WIDTH * (cos(theta) - cos(prev_theta)));
      }
    }
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
