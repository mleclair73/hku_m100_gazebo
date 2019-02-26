#include "ros/ros.h"
#include <ros/console.h>
#include "tf/transform_datatypes.h"

#include "std_msgs/String.h"

#include <dji_sdk/SetLocalPosRef.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include <cmath>

#include <string>


geometry_msgs::Pose target_pose;
geometry_msgs::Twist target_twist;
geometry_msgs::Pose target_gimbal_pose;
geometry_msgs::Twist target_gimbal_twist;

geometry_msgs::Pose target_roll_pose;
geometry_msgs::Pose target_yaw_pose;
geometry_msgs::Pose target_pitch_pose;

gazebo_msgs::ModelState target_model_state;
gazebo_msgs::LinkState target_gimbal_state;

std::string model_name = "m100";
std::string reference_frame = "world";

ros::Subscriber attitude_quaternion_subscriber;
ros::Subscriber velocity_subscriber;
ros::Subscriber local_position_subscriber;
ros::Subscriber imu_subscriber;
ros::Subscriber gimbal_orientation_subscriber;
ros::Subscriber car_driver_subscriber;

ros::ServiceClient model_state_client;
ros::ServiceClient gimbal_state_client;
ros::ServiceClient get_model_state_client;

gazebo_msgs::SetModelState set_model_state;
gazebo_msgs::SetLinkState set_link_state;

bool velocity_updated = false;
bool position_updated = false;
bool attitude_updated = false;
bool imu_updated = false;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    target_twist.angular.x = imu_msg->angular_velocity.x;
    target_twist.angular.y = imu_msg->angular_velocity.y;
    target_twist.angular.z = imu_msg->angular_velocity.z;
    // ROS_ERROR("Imu Updated");

    imu_updated = true;
}

void attitudeQuaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& attitude_quaternion_msg)
{
  tf::Quaternion quat;
  tf::quaternionMsgToTF(attitude_quaternion_msg->quaternion, quat);
  // the tf::Quaternion has a method to acess roll pitch and yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion orientation= tf::createQuaternionMsgFromRollPitchYaw(roll, -pitch, yaw);

  target_pose.orientation = orientation;
  // ROS_ERROR("Att Updated");

  attitude_updated = true;
}

void velocityCallback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity_msg)
{
  target_twist.linear.x  = velocity_msg->vector.x;
  target_twist.linear.y  = velocity_msg->vector.y;
  target_twist.linear.z  = velocity_msg->vector.z;
  // ROS_ERROR("Vel Updated");

  velocity_updated = true;
}

void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
{
  target_pose.position.x = position_msg->point.x;
  target_pose.position.y = -position_msg->point.y;
  target_pose.position.z = position_msg->point.z;
  ROS_ERROR("Local Pos Updated");

  position_updated = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "hku_m100_pcsim_gazebo_bridge");

  ros::NodeHandle n;

  dji_sdk::SetLocalPosRef srv;
  if (ros::service::call("/dji_sdk/set_local_pos_ref", srv))
  {
    ROS_ERROR("Set local pos ref!");
  }
  else
  {
    ROS_ERROR("Couldn't set local position reference. It's not going to work unless you set this manually");
  }
 
  attitude_quaternion_subscriber = n.subscribe("/uas1/dji_sdk/attitude", 1000, attitudeQuaternionCallback);
  velocity_subscriber = n.subscribe("/uas1/dji_sdk/velocity", 1000, velocityCallback);
  local_position_subscriber = n.subscribe("/uas1/dji_sdk/local_position", 1000, localPositionCallback);
  imu_subscriber = n.subscribe("/uas1/dji_sdk/imu", 1000, imuCallback);

  //model_state_client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);

  ROS_INFO("Bridge between PC sim and gazebo connected");

  ros::Rate spin_rate(50);
  bool first = true;

  while(ros::ok())
  {
    ros::spinOnce();

    target_model_state.model_name = model_name;
    target_model_state.reference_frame = reference_frame;
    target_model_state.pose = target_pose;
    target_model_state.twist = target_twist;
    set_model_state.request.model_state = target_model_state;
    if (first || velocity_updated || position_updated || imu_updated || attitude_updated)
    {
      first = false;
      if (ros::service::call("/gazebo/set_model_state", set_model_state))
      {
        ROS_ERROR_THROTTLE(1, "Set model state succeeded");
      }
      else
      {
        ROS_ERROR_THROTTLE(1, "Set model state failed");
      }
      ROS_ERROR_STREAM_THROTTLE(1, "Name: " << target_model_state.model_name << " Ref frame: " << \
       target_model_state.reference_frame << " Pose: " << target_model_state.pose.position.x << ", " \
       << target_model_state.pose.position.y << ", " << target_model_state.pose.position.z);
      // ROS_ERROR_THROTTLE(1, "Twist: %f, %f, %f", target_model_state.twist.linear.x, target_model_state.twist.linear.y, target_model_state.twist.linear.z);
      // ROS_ERROR_THROTTLE(1, "Pose: %f, %f, %f, %f", target_model_state.pose.orientation.w,
      //                                   target_model_state.pose.orientation.x,
      //                                   target_model_state.pose.orientation.y,
      //                                   target_model_state.pose.orientation.z);

    }
    spin_rate.sleep();
  }

  return 0;
}
