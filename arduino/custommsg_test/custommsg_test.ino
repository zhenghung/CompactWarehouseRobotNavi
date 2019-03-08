#include <ros.h>
#include <ros/time.h>
#include <imu_read/imu_read.h>

ros::NodeHandle robot;


imu_read::imu_read custom_msg;

ros::Publisher pub_custom("CompressedMsg", &custom_msg);

void publishMsg(){
  ros::Time current_time = robot.now();

  custom_msg.header.stamp = current_time;
  custom_msg.x = 1/1000;
  custom_msg.y = 1/1000;
  custom_msg.theta = 1.5;

  pub_custom.publish(&custom_msg);

}


void setup(){
    robot.initNode();
    robot.advertise(pub_custom);
}
void loop(){
    publishMsg();
    robot.spinOnce();
}