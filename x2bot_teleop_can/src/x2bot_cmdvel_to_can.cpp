/********x2bot_joy.cpp***************
*功能：x2bot遥控节点，接收joy/joy_node节点发来的手柄按键信息，转换为控制速度cmd_vel，最大线速度0.85m/s，最大角速度8.20rad/s，
*     由于采用deadman机制，必须按住2号按钮（2/B）才能用方向键控制方向
*作者：张京林
*时间：2016.06.27
***********************/
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include "../../can_module_usb2can/include/sendMsg.h"

#define _CANMSGNAME_  can_msgs::ecu_120

class X2botTeleop_cmdvel_to_can
{
public:
  X2botTeleop_cmdvel_to_can();
private:
  void cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmdvel_msgs);
  void publish();

  ros::NodeHandle ph_,nh_;

  _CANMSGNAME_ ecu_cmd_published; //the conveted ecucmd
  _CANMSGNAME_ ecu_cmd_published_break_; //the conveted ecucmd

  geometry_msgs::Twist last_cmd;   //the cmd_vel cmd
  ros::Publisher ecu_pub_;
  ros::Subscriber cmdvel_sub_;
  
  boost::mutex publish_mutex_;
  //bool deadman_pressed_;
  bool zero_twist_published_;
  ros::Timer timer_;
};

X2botTeleop_cmdvel_to_can::X2botTeleop_cmdvel_to_can():
 ph_("~")
{
  //NodeHandle::param(parameter name,paramer value,default value)
  //ph_.param("axis_linear", linear_, linear_);//线速度下标，axes中代表前后的控制量，对于莱仕达方向键下标是1，上1，下-1
  //ph_.param("axis_angular", angular_, angular_);//角速度下标，axes中代表左右的控制量，对于莱仕达方向键下标是0，左1，右-1
  //ph_.param("axis_deadman", deadman_axis_, deadman_axis_);//按住2号按钮（2/B）然后用方向键控制方向
  //ph_.param("scale_angular", a_scale_, a_scale_);
  //ph_.param("scale_linear", l_scale_, l_scale_);

  //deadman_pressed_ = false;
  zero_twist_published_ = false;

  ecu_pub_ = ph_.advertise<_CANMSGNAME_>("/ecu",1, true);
  cmdvel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &X2botTeleop_cmdvel_to_can::cmdvelCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&X2botTeleop_cmdvel_to_can::publish, this));

  //tbe break cmd
  ecu_cmd_published_break_.brake = 1;
  //ecu_cmd_published_break_.lmotor = 0;
  //ecu_cmd_published_break_.rmotor = 0;
  ecu_cmd_published_break_.motor = 0;
  ecu_cmd_published_break_.steer = 0;
  ecu_cmd_published_break_.shift = 2;
}

void X2botTeleop_cmdvel_to_can::cmdvelCallback(const geometry_msgs::Twist::ConstPtr& cmdvel_msgs)
{
  //float pi=3.1415926;//
  //float R=0.101;//m
  //float xishu = 30.0/(pi*R);// m/s to rpm xishu 
  //float width_robot = 0.32;// kuandu 0.5
  //float vx = cmdvel_msgs->linear.x;
  //float vang = cmdvel_msgs->angular.z; 
  //bool brake = (bool)(cmdvel_msgs->linear.z);

  //if(!brake)
  //{
    //ecu_cmd_published.motor = vx;
   // float sx = vx*xishu; //change to rpm
    //float vth = vang*width_robot /2 *xishu;//change to rpm
    //float angx = 
    //float x;
    //float y;
    //float de = 1;
    //x= sx-vth;
    //y= sx+vth;

  bool brake = (bool)(cmdvel_msgs->linear.z);

  if(!brake){
    float speed = cmdvel_msgs->linear.x;
    float steer = cmdvel_msgs->angular.z * 27;

    if(speed > 0){
      ecu_cmd_published.shift = 1;
    }else if(speed < 0){
      ecu_cmd_published.shift = 3;
      speed = -1 * speed;
    }
    std::cout << ecu_cmd_published.motor << std::endl;
    ecu_cmd_published.motor = speed * 2;
    ecu_cmd_published.steer = steer;
    ecu_pub_.publish(ecu_cmd_published);
  }else{
    ecu_pub_.publish(ecu_cmd_published_break_);
  }

  //float32 motor # 目标速度
  //float32 steer # 转向
  //bool brake # 紧急停车
  //float32 cur_speed # 当前速度

  //uint8 shift # 档位
  //uint8 SHIFT_UNKNOWN = 0
  //uint8 SHIFT_D = 1 #前进档位
  //uint8 SHIFT_N = 2 #停止档位
  //uint8 SHIFT_R = 3 #后退档位
  //# uint8 SHIFT_T = 9 #遥控
}

void X2botTeleop_cmdvel_to_can::publish()
{
  boost::mutex::scoped_lock lock(publish_mutex_);

 // if (deadman_pressed_)
 // {
 //   vel_pub_.publish(last_published_);
 //   zero_twist_published_=false;
 // }
  //else if(!deadman_pressed_ && !zero_twist_published_)
  //{
  //  vel_pub_.publish(*new geometry_msgs::Twist());//没有按下deadman开关则发送的速度均为0
  //  zero_twist_published_=true;
  //}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "X2botTeleop_cmdvel_to_can");
  X2botTeleop_cmdvel_to_can X2botTeleop_cmdvel_to_can;

  ros::spin();
}
