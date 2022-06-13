#ifndef CAN_SEND_MSG_H
#define CAN_SEND_MSG_H

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

#include <can_msgs/ecu_120.h>
#include <can_msgs/ecu_121.h>
#include <can_msgs/ecu_122.h>


// send vehicle control msg protocol_120
#define SCU_SHIFT_LEVEL_REQ_OFFSET_120 0
#define SCU_SHIFT_LEVEL_REQ_LENGTH_120 2 // 车辆档位请求： 0 为检测到或初始状态  1：D 2：N 3：R
#define SCU_ACC_MODE_OFFSET_120 2
#define SCU_ACC_MODE_LENGTH_120 2 // 加速模式  0：不影响 1：加速1 2：加速2 3：加速3
#define SCU_BRAKE_MODE_OFFSET_120 4
#define SCU_BRAKE_MODE_LENGTH_120 2 // 制动模式 0：不影响 1：制动1 2：制动2 3：制动3
#define SCU_DRIVE_MODE_REQ_OFFSET_120 6
#define SCU_DRIVE_MODE_REQ_LENGTH_120 2 // 驾驶模式请求： 0：不影响 1：自动驾驶模式请求  2：驾驶员PAD模式请求  3：驾驶员方向盘模式请求
#define SCU_STEERING_WHEEL_ANGLE_OFFSET_120 8
#define SCU_STEERING_WHEEL_ANGLE_LENGTH_120 16 // 请求方向盘转角信号
#define SCU_TARGET_SPEED_OFFSET_120 24
#define SCU_TARGET_SPEED_LENGTH_120 9 // 目标车速请求
#define SCU_BRK_EN_OFFSET_120 33
#define SCU_BRK_EN_LENGTH_120 1 // 紧急制动请求 0 不需要紧急制动  1 需要紧急制动
#define SCU_GEARNTOQUE_EN_OFFSET_120 34
#define SCU_GEARNTOQUE_EN_LENGTH_120 1// 0 空挡滑行  1 扭矩滑行

// send vehicle control msg protocol_121
#define SCU_SHIFT_LEVEL_REQ_OFFSET_121 0
#define SCU_SHIFT_LEVEL_REQ_LENGTH_121 2 
#define SCU_ACC_MODE_OFFSET_121 2
#define SCU_ACC_MODE_LENGTH_121 2 
#define SCU_BRAKE_MODE_OFFSET_121 4
#define SCU_BRAKE_MODE_LENGTH_121 2 
#define SCU_DRIVE_MODE_REQ_OFFSET_121 6
#define SCU_DRIVE_MODE_REQ_LENGTH_121 2 
#define SCU_STEERING_WHEEL_ANGLE_F_OFFSET_121 8
#define SCU_STEERING_WHEEL_ANGLE_F_LENGTH_121 8 // 前轮转向角
#define SCU_STEERING_WHEEL_ANGLE_R_OFFSET_121 16
#define SCU_STEERING_WHEEL_ANGLE_R_LENGTH_121 8 // 后轮转向角
#define SCU_TARGET_SPEED_OFFSET_121 24
#define SCU_TARGET_SPEED_LENGTH_121 9 // 目标车速请求
#define SCU_BRK_EN_OFFSET_121 33
#define SCU_BRK_EN_LENGTH_121 1

// send vehicle control msg protocol_122
#define SCU_SHIFT_LEVEL_REQ_OFFSET_122 0
#define SCU_SHIFT_LEVEL_REQ_LENGTH_122 2 
#define SCU_ACC_MODE_OFFSET_122 2
#define SCU_ACC_MODE_LENGTH_122 2 
#define SCU_BRAKE_MODE_OFFSET_122 4
#define SCU_BRAKE_MODE_LENGTH_122 2 
#define SCU_DRIVE_MODE_REQ_OFFSET_122 6
#define SCU_DRIVE_MODE_REQ_LENGTH_122 2 
#define SCU_LMOTOR_SPD_RPM_OFFSET_122 8
#define SCU_LMOTOR_SPD_RPM_LENGTH_122 16 // 左轮速度
#define SCU_RMOTOR_SPD_RPM_OFFSET_122 24
#define SCU_RMOTOR_SPD_RPM_LENGTH_122 16 // 右轮速度

#define BUFFER_SIZE 8

namespace USB2CAN
{
enum ShiftLevel
{
    NOT_DETECTED = 0,
    D = 1, // 前进
    N = 2, // 停车
    R = 3  // 倒车
};

enum DriveMode
{
    AUTO_MODE = 1,
    PAD_MODE = 2,
    WHEEL_MODE = 3
};

class SendMsg120
{
private:
    can_msgs::ecu_120 ecuMsg;
    VCI_CAN_OBJ canMsg;

    double pre_steer;

    void initCanMsg();

    void setDriveMode(DriveMode driveMode);

    void setShiftLevel(ShiftLevel shiftLevel);

    void setAccMode(int v);

    void setBrakeMode(int v);

    void setSpeed(double v);

    void setWheelAngle(double angle);

    void setEBrake(bool need);

    void setGearNToque(bool need);

    void rosMsg2canMsg();

public:
    SendMsg120()
    {
        initCanMsg();
    }

    SendMsg120(can_msgs::ecu_120 ecuMsg, double pre_steer)
    {
        initCanMsg();
        this->ecuMsg = ecuMsg;
        this->pre_steer = pre_steer;
    }

    void print();

    VCI_CAN_OBJ getMessage();
};

class SendMsg121
{
private:
    can_msgs::ecu_121 ecuMsg;
    VCI_CAN_OBJ canMsg;

    double pre_steer_f;
    double pre_steer_r;

    void initCanMsg();

    void setDriveMode(DriveMode driveMode);

    void setShiftLevel(ShiftLevel shiftLevel);

    void setAccMode(int v);

    void setBrakeMode(int v);

    void setSpeed(double v);

    void setWheelAngleF(double angle);

    void setWheelAngleR(double angle);

    void setEBrake(bool need = false);

    void rosMsg2canMsg();

public:
    SendMsg121()
    {
        initCanMsg();
    }

    SendMsg121(can_msgs::ecu_121 ecuMsg, double pre_steer_f, double pre_steer_r)
    {
        initCanMsg();
        this->ecuMsg = ecuMsg;
        this->pre_steer_f = pre_steer_f;
        this->pre_steer_r = pre_steer_r;
    }

    void print();

    VCI_CAN_OBJ getMessage();
};

class SendMsg122
{
private:
    can_msgs::ecu_122 ecuMsg;
    VCI_CAN_OBJ canMsg;

    void initCanMsg();

    void setDriveMode(DriveMode driveMode);

    void setShiftLevel(ShiftLevel shiftLevel);

    void setAccMode(int v);

    void setBrakeMode(int v);

    void setLSpeed(double v);

    void setRSpeed(double v);

    void rosMsg2canMsg();

public:
    SendMsg122()
    {
        initCanMsg();
    }

    SendMsg122(can_msgs::ecu_122 ecuMsg)
    {
        initCanMsg();
        this->ecuMsg = ecuMsg;
    }

    void print();

    VCI_CAN_OBJ getMessage();
};

} // namespace USB2CAN
#endif // !CANLIB_MSG_H
