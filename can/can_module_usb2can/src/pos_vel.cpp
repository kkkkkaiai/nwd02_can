#include <ros/ros.h>
#include <math.h>
#include <can_msgs/ecu_120.h>
#include "geometry_msgs/PoseStamped.h"
#include <eigen3/Eigen/Dense>

#define PI 3.14159265

double target_point[3] = {10,-2,0};
Eigen::Vector3d axisx(1,0,0);

ros::Publisher pubcarvelocity;

void carposeHandler(const geometry_msgs::PoseStamped::ConstPtr& carposeMsg)
{
    double positionX, positionY, positionZ;
    double orientationX, orientationY, orientationZ, orientationW;

    positionX = carposeMsg->pose.position.x;
    positionY = carposeMsg->pose.position.y;
    positionZ = carposeMsg->pose.position.z;
    orientationX = carposeMsg->pose.orientation.x;
    orientationY = carposeMsg->pose.orientation.y;
    orientationZ = carposeMsg->pose.orientation.z;
    orientationW = carposeMsg->pose.orientation.w;

    Eigen::Quaterniond quaternion(orientationW,orientationX,orientationY,orientationZ);

    Eigen::Matrix3d rotation_matrix = quaternion.matrix();

    Eigen::Vector3d trans_axisx = rotation_matrix*axisx;
    Eigen::Vector3d car_target(target_point[0]-positionX,target_point[1]-positionY,target_point[2]-positionZ);

    float anglex = atan2(trans_axisx(1), trans_axisx(0)) * 180 / PI;
    float anglet = atan2(car_target(1), car_target(0))  * 180 / PI;

    ROS_INFO("is %f , %f", anglex,anglet);

    can_msgs::ecu_120 car_msg;
    ros::Rate loop_rate(100);
    if(fabs(car_target(0)) < 0.5 && fabs(car_target(1))<0.5)
    {
        car_msg.lmotor = 0;
		car_msg.rmotor = 0;
		car_msg.shift = 1;
    }
    else if (anglet - anglex < -1)
    {
        car_msg.lmotor = 25;
		car_msg.rmotor = 20;
		car_msg.shift = 1;
    }
    else if (anglet - anglex >1)
    {
        car_msg.lmotor = 20;
		car_msg.rmotor = 25;
		car_msg.shift = 1;
    }
    else
    {
        car_msg.lmotor = 20;
		car_msg.rmotor = 20;
		car_msg.shift = 1;
    }
    for(int i=0;i<10;i++){
		pubcarvelocity.publish(car_msg);
		loop_rate.sleep();
		}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pos_vel");
    ros::NodeHandle nh;

     ros::Subscriber subcarpose = nh.subscribe("ndt_pose", 10, carposeHandler);

     pubcarvelocity = nh.advertise<can_msgs::ecu_120>("/ecu", 100);


    ros::spin();

    return 0;
}
