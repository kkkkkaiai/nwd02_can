#include "sendMsg.h"

namespace USB2CAN
{

// SendMsg120
void SendMsg120::initCanMsg()
{
    bzero(canMsg.Data, BUFFER_SIZE);
    canMsg.ID = 0x120;
    canMsg.SendType = 0x01;  // 0:正常发送,发送失败则重发  1:只发送一次
    canMsg.RemoteFlag = 0x00; // 0:数据帧  1:远程帧(数据段空)
    canMsg.ExternFlag = 0x00; // 0:标准帧  1:扩展帧
    canMsg.DataLen = 0x08;

    this->setDriveMode(AUTO_MODE); //设置驾驶模式 ： 自动驾驶
    this->setAccMode(1);  //设置加速档位
    this->setBrakeMode(1);  // 设置制动档位
    this->setEBrake(1);
}

void SendMsg120::setDriveMode(DriveMode driveMode)
{
    writeInt(canMsg.Data, SCU_DRIVE_MODE_REQ_OFFSET_120, SCU_DRIVE_MODE_REQ_LENGTH_120, driveMode);
}
void SendMsg120::setShiftLevel(ShiftLevel shift_level)
{
    writeInt(canMsg.Data, SCU_SHIFT_LEVEL_REQ_OFFSET_120, SCU_SHIFT_LEVEL_REQ_LENGTH_120, shift_level);
}
void SendMsg120::setAccMode(int v)
{
    writeInt(canMsg.Data, SCU_ACC_MODE_OFFSET_120, SCU_ACC_MODE_LENGTH_120, v);
}
void SendMsg120::setBrakeMode(int v)
{
    writeInt(canMsg.Data, SCU_BRAKE_MODE_OFFSET_120, SCU_BRAKE_MODE_LENGTH_120, v);
}
void SendMsg120::setSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(canMsg.Data, SCU_TARGET_SPEED_OFFSET_120, SCU_TARGET_SPEED_LENGTH_120, speed);
}
void SendMsg120::setWheelAngle(double angle)
{
    short v = angle*10; 
    writeInt(canMsg.Data, SCU_STEERING_WHEEL_ANGLE_OFFSET_120, SCU_STEERING_WHEEL_ANGLE_LENGTH_120, v);
}
void SendMsg120::setEBrake(bool need)
{
    writeInt(canMsg.Data, SCU_BRK_EN_OFFSET_120, SCU_BRK_EN_LENGTH_120, int(need));
}

void SendMsg120::print()
{
    fprintf(stdout, "send ecu/CCUMsg: "); 
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "%02X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}

void SendMsg120::rosMsg2canMsg()
{
    this->setShiftLevel(ShiftLevel(ecuMsg.shift));
    this->setSpeed(ecuMsg.motor);
    this->setWheelAngle((ecuMsg.steer + this->pre_steer) / 2);
    this->setEBrake(ecuMsg.brake);
}

VCI_CAN_OBJ SendMsg120::getMessage()
{
    this->rosMsg2canMsg();
    VCI_CAN_OBJ return_msg = this->canMsg;
    return return_msg;
}


// SendMsg121
void SendMsg121::initCanMsg()
{
    bzero(canMsg.Data, BUFFER_SIZE);
    canMsg.ID = 0x121;
    canMsg.SendType = 0x01;
    canMsg.RemoteFlag = 0x00;
    canMsg.ExternFlag = 0x00; 
    canMsg.DataLen = 0x08;

    this->setDriveMode(AUTO_MODE); 
    this->setAccMode(1);
    this->setBrakeMode(1);
    this->setEBrake(1);
}

void SendMsg121::setDriveMode(DriveMode driveMode)
{
    writeInt(canMsg.Data, SCU_DRIVE_MODE_REQ_OFFSET_121, SCU_DRIVE_MODE_REQ_LENGTH_121, driveMode);
}
void SendMsg121::setShiftLevel(ShiftLevel shift_level)
{
    writeInt(canMsg.Data, SCU_SHIFT_LEVEL_REQ_OFFSET_121, SCU_SHIFT_LEVEL_REQ_LENGTH_121, shift_level);
}
void SendMsg121::setAccMode(int v)
{
    writeInt(canMsg.Data, SCU_ACC_MODE_OFFSET_121, SCU_ACC_MODE_LENGTH_121, v);
}
void SendMsg121::setBrakeMode(int v)
{
    writeInt(canMsg.Data, SCU_BRAKE_MODE_OFFSET_121, SCU_BRAKE_MODE_LENGTH_121, v);
}
void SendMsg121::setSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(canMsg.Data, SCU_TARGET_SPEED_OFFSET_121, SCU_TARGET_SPEED_LENGTH_121, speed);
}
void SendMsg121::setWheelAngleF(double angle)
{
    short v = angle ; 
    writeInt(canMsg.Data, SCU_STEERING_WHEEL_ANGLE_F_OFFSET_121, SCU_STEERING_WHEEL_ANGLE_F_LENGTH_121, v);
}
void SendMsg121::setWheelAngleR(double angle)
{
    short v = angle ; 
    writeInt(canMsg.Data, SCU_STEERING_WHEEL_ANGLE_R_OFFSET_121, SCU_STEERING_WHEEL_ANGLE_R_LENGTH_121, v);
}
void SendMsg121::setEBrake(bool need)
{
    writeInt(canMsg.Data, SCU_BRK_EN_OFFSET_121, SCU_BRK_EN_LENGTH_121, int(need));
}

void SendMsg121::print()
{
    fprintf(stdout, "send ecu/CCUMsg: "); 
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "%02X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}

void SendMsg121::rosMsg2canMsg()
{
    this->setShiftLevel(ShiftLevel(ecuMsg.shift));
    this->setSpeed(ecuMsg.motor);
    this->setWheelAngleF((ecuMsg.anglef + this->pre_steer_f)/2);
    this->setWheelAngleR((ecuMsg.angler + this->pre_steer_r)/2);
    this->setEBrake(ecuMsg.brake);
}

VCI_CAN_OBJ SendMsg121::getMessage()
{
    this->rosMsg2canMsg();
    VCI_CAN_OBJ return_msg = this->canMsg;
    return return_msg;
}


// SendMsg122
void SendMsg122::initCanMsg()
{
    bzero(canMsg.Data, BUFFER_SIZE);
    canMsg.ID = 0x122;
    canMsg.SendType = 0x01;  
    canMsg.RemoteFlag = 0x00; 
    canMsg.ExternFlag = 0x00;
    canMsg.DataLen = 0x08;

    this->setDriveMode(AUTO_MODE); 
    this->setAccMode(1);
    this->setBrakeMode(1);
}

void SendMsg122::setDriveMode(DriveMode driveMode)
{
    writeInt(canMsg.Data, SCU_DRIVE_MODE_REQ_OFFSET_122, SCU_DRIVE_MODE_REQ_LENGTH_122, driveMode);
}
void SendMsg122::setShiftLevel(ShiftLevel shift_level)
{
    writeInt(canMsg.Data, SCU_SHIFT_LEVEL_REQ_OFFSET_122, SCU_SHIFT_LEVEL_REQ_LENGTH_122, shift_level);
}
void SendMsg122::setAccMode(int v)
{
    writeInt(canMsg.Data, SCU_ACC_MODE_OFFSET_122, SCU_ACC_MODE_LENGTH_122, v);
}
void SendMsg122::setBrakeMode(int v)
{
    writeInt(canMsg.Data, SCU_BRAKE_MODE_OFFSET_122, SCU_BRAKE_MODE_LENGTH_122, v);
}
void SendMsg122::setLSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(canMsg.Data, SCU_LMOTOR_SPD_RPM_OFFSET_122, SCU_LMOTOR_SPD_RPM_LENGTH_122, speed);
}
void SendMsg122::setRSpeed(double v)
{
    int speed = (int)lround(v * 10);
    writeInt(canMsg.Data, SCU_RMOTOR_SPD_RPM_OFFSET_122, SCU_RMOTOR_SPD_RPM_LENGTH_122, speed);
}

void SendMsg122::print()
{
    fprintf(stdout, "send ecu/CCUMsg: "); 
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "%02X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}

void SendMsg122::rosMsg2canMsg()
{
    this->setShiftLevel(ShiftLevel(ecuMsg.shift));
    this->setLSpeed(ecuMsg.lmotor);
    this->setRSpeed(ecuMsg.rmotor);
}

VCI_CAN_OBJ SendMsg122::getMessage()
{
    this->rosMsg2canMsg();
    VCI_CAN_OBJ return_msg = this->canMsg;
    return return_msg;
}

} // namespace USB2CAN
