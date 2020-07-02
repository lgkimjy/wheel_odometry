#include <ros/ros.h>

#include <stdlib.h>
#include <unistd.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <boost/assign/list_of.hpp>

#include "ssoni_mobile_msgs/motor_cmd.h"
#include "ssoni_mobile_msgs/motor_encoder.h"


#define PI 3.14159265
#define TwoPI 6.28318531
#define wheeltrack 0.3876   // units : m  
#define EncoderCountsPerWheel 396
#define wheelradius 0.0575  // units : m


long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time, last_time;
double DistancePerCount = (3.14159265 * 2 * wheelradius) / (EncoderCountsPerWheel * 4); //the wheel diameter is 0.1m

//final odometric datas
double x;
double y;
double th;
double v_left;//left motor speed
double v_right;//right motor speed
double vth;//angular velocity of robot
double deltaLeft;//no of ticks in left encoder since last update
double deltaRight;//no of ticks in right encoder since last update
double dt;
double delta_distance;//distance moved by robot since last update
double delta_th;//corresponging change in heading
double delta_x ;//corresponding change in x direction
double delta_y;//corresponding change in y direction


void WheelCallback(const ssoni_mobile_msgs::motor_encoder::ConstPtr& motor)
{
  current_time = ros::Time::now();

  deltaLeft = motor->left_encoder - _PreviousLeftEncoderCounts;
  deltaRight = motor->right_encoder - _PreviousRightEncoderCounts;

  dt = (current_time - last_time).toSec();

  v_left = deltaLeft * DistancePerCount/dt;    //speed = m/s
  v_right = deltaRight * DistancePerCount/dt;  //speed = m/s

  delta_distance=0.5f * (double)(deltaLeft + deltaRight) * DistancePerCount;
  delta_th = (double)(deltaRight-deltaLeft)*DistancePerCount/wheeltrack; //Distance between the two wheels is 0.36m
  
  delta_x = delta_distance*(double)cos(th);
  delta_y = delta_distance*(double)sin(th);

  x += delta_x;
  y += delta_y;
  th += delta_th;
 
  // if (th > PI)
  //   th -= TwoPI;
  // else if ( th <= -PI)
  //   th += TwoPI;
  
  _PreviousLeftEncoderCounts = motor->left_encoder;
  _PreviousRightEncoderCounts = motor->right_encoder;

  last_time = current_time;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("pos", 100, WheelCallback);
  ros::Subscriber motor_encoder_sub = nh.subscribe("/motor_encoder", 10, WheelCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate r(100);

  while(nh.ok())
  {
    //since all odometry is 6DO F we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.pose.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                   (0) (1e-3)  (0)  (0)  (0)  (0)
                                                   (0)   (0)  (1e6) (0)  (0)  (0)
                                                   (0)   (0)   (0) (1e6) (0)  (0)
                                                   (0)   (0)   (0)  (0) (1e6) (0)
                                                   (0)   (0)   (0)  (0)  (0)  (1e3) ;
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x=delta_x/dt;
    odom.twist.twist.linear.y=delta_y/dt;
    odom.twist.twist.angular.z = delta_th/dt;
    odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                                                    (0) (1e-3)  (0)  (0)  (0)  (0)
                                                    (0)   (0)  (1e6) (0)  (0)  (0)
                                                    (0)   (0)   (0) (1e6) (0)  (0)
                                                    (0)   (0)   (0)  (0) (1e6) (0)
                                                    (0)   (0)   (0)  (0)  (0)  (1e3) ; 

    // ROS_INFO("Position x = %d and Position y = %d ",x,y);

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    ros::spinOnce();
    r.sleep();
    }
}