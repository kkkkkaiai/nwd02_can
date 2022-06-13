#ifndef CAN_RECEIVE_MSG_H
#define CAN_RECEIVE_MSG_H

#include "controlcan.h"
#include "utils.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sstream>
#include <stdint-gcc.h>
#include <string.h>
#include <string>
#include <iostream>
#include <can_msgs/vehicle_status.h>
#include <can_msgs/wheel_speedl.h>
#include <can_msgs/wheel_speedr.h>

// reveive vehicle status msg protocol
#define CCU_SHIFT_LEVEL_STS_OFFSET_51 0
#define CCU_SHIFT_LEVEL_STS_LENGTH_51 2 // 档位  1：D  2：N  3：R
#define CCU_STEERING_WHEEL_DIRECTION_OFFSET_51 7 // 实际方向盘角度方向 0左边 1右边
#define CCU_STEERING_WHEEL_ANGLE_OFFSET_51 8
#define CCU_STEERING_WHEEL_ANGLE_LEGNTH_51 12 // 实际方向盘角度 单位 0.1度  最大值 1200 代表 120度
#define CCU_VEHICLE_SPEED_OFFSET_51 20
#define CCU_VEHICLE_SPEED_LENGTH_51 9 // 当前车速 单位 0.1m/s 最大值 510 代表 51m/s
#define CCU_DRIVE_MODE_OFFSET_51 29
#define CCU_DRIVE_MODE_LENGTH_51 3 // 当前驾驶模式 0不影响  1自动驾驶模式  2驾驶员PAD模式  3驾驶员方向盘模式
#define CCU_ACC_LEVEL_OFFSET_51 32
#define CCU_ACC_LEVEL_LENGTH_51 2 // 当前加速档位
#define CCU_BRAKE_LEVEL_OFFSET_51 34
#define CCU_BRAKE_LEVEL_LENGTH_51 2 // 当前减速档位
#define CCU_TOTAL_ODOMETER_OFFSET_51 36
#define CCU_TOTAL_ODOMETER_LENGTH_51 20 // 车辆累计里程数 单位：km

#define CCU_SPEED_MEASURED_OFFSET 16
#define CCU_SPEED_MEASURED_LENGTH 16

namespace USB2CAN
{

enum SteerDirection
{
    left = 0,
    right = 1
};

class VehicleStatusMsg
{
private:
    VCI_CAN_OBJ canMsg;
    can_msgs::vehicle_status rosMsg_vehicle_status;

    int driveMode();

    int shiftLevel();

    int accLevel();

    int brakeLevel();

    double speed();

    double wheelAngle();

    SteerDirection wheelDirection();

    double totalOdometer();

    void createMessage();

public:
    VehicleStatusMsg(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    void print();

    std::string toString();

    can_msgs::vehicle_status getMessage();
};


class ReceiveMsg178
{
private:
    VCI_CAN_OBJ canMsg;
    can_msgs::wheel_speedl rosMsg_speed;

    double speed();

    void createMessage();

public:
    ReceiveMsg178(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    can_msgs::wheel_speedl getMessage();
};


class ReceiveMsg188
{
private:
    VCI_CAN_OBJ canMsg;
    can_msgs::wheel_speedr rosMsg_speed;

    double speed();

    void createMessage();

public:
    ReceiveMsg188(VCI_CAN_OBJ in_canMsg)
    {
        this->canMsg = in_canMsg;
    }

    can_msgs::wheel_speedr getMessage();
};

} // namespace USB2CAN
#endif // !CAN_RECEIVE_MSG_H
