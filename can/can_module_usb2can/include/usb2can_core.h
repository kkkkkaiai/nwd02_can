#pragma once
#ifndef USB2CAN_CORE_H
#define USB2CAN_CORE_H

#include <iostream>
#include <ros/ros.h>
#include "sendMsg.h"
#include "receiveMsg.h"
#include "unistd.h"
#include <cstdlib>
#include <ctime>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <can_msgs/ecu_120.h>
#include <can_msgs/ecu_121.h>
#include <can_msgs/ecu_122.h>
#include <can_msgs/vehicle_status.h>
#include <can_msgs/wheel_speedl.h>
#include <can_msgs/wheel_speedr.h>

namespace USB2CAN
{
struct Param
{
  uint8_t mrun_num;
  std::string name;
  int debug_can_port;
  int main_can_port;
  ros::Publisher wheel_speedl_publisher;
  ros::Publisher wheel_speedr_publisher;
  ros::Publisher vehicle_status_publisher;
};

static DWORD CAN_id = 0; // 默认采用第一个CAN设备, (目前也只有一个CAN设备,即USBCAN2)
static VCI_INIT_CONFIG config;

static pthread_t threadid;
static Param *param;

class CAN_app
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle p_nh;

  ros::Subscriber sub_ecu;
  ros::Publisher pub_vehicle_status;
  ros::Publisher pub_wheel_speedl;
  ros::Publisher pub_wheel_speedr;

  bool param_debug;
  int debug_can_port;
  int main_can_port;
  int send_msg_id;
  bool param_show_sending_msg;

  double pre_steer;
  double pre_steer_f;
  double pre_steer_r;

  void initROS();

  void ecu_cb120(const can_msgs::ecu_120::ConstPtr &msg);
  void ecu_cb121(const can_msgs::ecu_121::ConstPtr &msg);
  void ecu_cb122(const can_msgs::ecu_122::ConstPtr &msg);

public:
  CAN_app();
  ~CAN_app();

  void run();
};

} // namespace USB2CAN
#endif
