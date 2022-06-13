#include "receiveMsg.h"

namespace USB2CAN
{

//Receive canMsg51: vehicle_status
int VehicleStatusMsg::driveMode()
{
    return readAsInt(this->canMsg.Data, CCU_DRIVE_MODE_OFFSET_51, CCU_DRIVE_MODE_LENGTH_51);
}
int VehicleStatusMsg::shiftLevel()
{
    return readAsInt(this->canMsg.Data, CCU_SHIFT_LEVEL_STS_OFFSET_51, CCU_SHIFT_LEVEL_STS_LENGTH_51);
}
int VehicleStatusMsg::accLevel()
{
    return readAsInt(this->canMsg.Data, CCU_ACC_LEVEL_OFFSET_51, CCU_ACC_LEVEL_LENGTH_51);
}
int VehicleStatusMsg::brakeLevel()
{
    return readAsInt(this->canMsg.Data, CCU_BRAKE_LEVEL_OFFSET_51, CCU_BRAKE_LEVEL_LENGTH_51);
}
double VehicleStatusMsg::speed()
{
    double speed = readAsInt(this->canMsg.Data, CCU_VEHICLE_SPEED_OFFSET_51, CCU_VEHICLE_SPEED_LENGTH_51) / 10.0; // 单位 m/s
    return speed;
}
double VehicleStatusMsg::wheelAngle()
{
    int wheel_angle_right = readBit(this->canMsg.Data, CCU_STEERING_WHEEL_DIRECTION_OFFSET_51);
    double angle = readAsInt(this->canMsg.Data, CCU_STEERING_WHEEL_ANGLE_OFFSET_51, CCU_STEERING_WHEEL_ANGLE_LEGNTH_51) / 44.44; 
    if (wheel_angle_right == 1)
    {
        return angle;
    }
    else
    {
        return -angle;
    }
}
SteerDirection VehicleStatusMsg::wheelDirection()
{
    int dir = readBit(this->canMsg.Data, CCU_STEERING_WHEEL_DIRECTION_OFFSET_51);
    if (dir == 0)
        return left;
    return right;
}
double VehicleStatusMsg::totalOdometer()
{
    double total_odometer = readAsInt(this->canMsg.Data, CCU_TOTAL_ODOMETER_OFFSET_51, CCU_TOTAL_ODOMETER_LENGTH_51);
    return total_odometer;
}

std::string VehicleStatusMsg::toString()
{
    std::string msg;
    std::stringstream ss;
    ss << "RevMsg speed: " << this->speed() << ", shiftLevel: " << this->shiftLevel() << ", wheelAngle: " << this->wheelAngle()
       << ", driveMode: " << this->driveMode() << ", accLevel: " << this->accLevel() << ", brakeLevel: " << this->brakeLevel()
       << ", totalOdometer: " << this->totalOdometer();
    ss >> msg;
    ss.str();
    return msg;
}
void VehicleStatusMsg::print()
{
    fprintf(stdout, "Recv Feedback: "); 
    fprintf(stdout, "ID: 0x%08X; ", this->canMsg.ID);
    fprintf(stdout, "SendType: %02X; ", this->canMsg.SendType);
    this->canMsg.ExternFlag == 0 ? printf(" Standard ") : printf(" Extend   ");
    this->canMsg.RemoteFlag == 0 ? printf(" Data     ") : printf(" Remote   ");
    fprintf(stdout, "DataLen: %02X; ", this->canMsg.DataLen);
    fprintf(stdout, "Data: ");
    for (int i = 0; i < 8; i++)
    {
        fprintf(stdout, "%X ", this->canMsg.Data[i]);
    }
    fprintf(stdout, "...\n");
}
void VehicleStatusMsg::createMessage()
{
    this->rosMsg_vehicle_status.shift_level = shiftLevel();

    this->rosMsg_vehicle_status.cur_speed = speed();

    this->rosMsg_vehicle_status.cur_steer = wheelAngle();

    this->rosMsg_vehicle_status.wheel_direction = wheelDirection();

    this->rosMsg_vehicle_status.drive_mode = driveMode();

    this->rosMsg_vehicle_status.acc_level = accLevel();

    this->rosMsg_vehicle_status.brake_level = brakeLevel();

    this->rosMsg_vehicle_status.total_odometer = totalOdometer();
}
can_msgs::vehicle_status VehicleStatusMsg::getMessage()
{
    this->createMessage();
    return this->rosMsg_vehicle_status;
}


double ReceiveMsg178::speed()
{
    double speed = readAsInt(this->canMsg.Data, CCU_SPEED_MEASURED_OFFSET, CCU_SPEED_MEASURED_LENGTH) ; 
    return speed;
}
void ReceiveMsg178::createMessage()
{
    this->rosMsg_speed.speedl = speed();
}
can_msgs::wheel_speedl ReceiveMsg178::getMessage()
{
    this->createMessage();
    return this->rosMsg_speed;
}


double ReceiveMsg188::speed()
{
    double speed = readAsInt(this->canMsg.Data, CCU_SPEED_MEASURED_OFFSET, CCU_SPEED_MEASURED_LENGTH) ; 
    return speed;
}
void ReceiveMsg188::createMessage()
{
    this->rosMsg_speed.speedr = speed();
}
can_msgs::wheel_speedr ReceiveMsg188::getMessage()
{
    this->createMessage();
    return this->rosMsg_speed;
}

} // namespace USB2CAN
