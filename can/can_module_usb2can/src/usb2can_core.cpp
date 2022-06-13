#include "usb2can_core.h"

namespace USB2CAN
{
void *receive_func(void *param_in)
{
    Param *param = (Param *)param_in;
    uint8_t mrun_num = param->mrun_num;
    ros::Publisher pub_wheel_speedl_ = param->wheel_speedl_publisher;
    ros::Publisher pub_wheel_speedr_ = param->wheel_speedr_publisher;
    ros::Publisher pub_vehicle_status_ = param->vehicle_status_publisher;

    VCI_CAN_OBJ rec[3000]; // 接收缓存
    int reclen = 0; // 获取收到的数据的帧数

    while (mrun_num & 0x0f)
    {
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, param->main_can_port, rec, 3000, 100)) > 0) //设备类型,设备索引,can通道索引,接收缓存索引,接收缓存大小,WaitTime(保留参数)
        {
            for (int i = 0; i < reclen; i++)
            {
                if (rec[i].ID == 0x51) // vehicle status
                {
                    VehicleStatusMsg msg_factory(rec[i]);
                    can_msgs::vehicle_status msg_vehicle_status = msg_factory.getMessage();
                    msg_vehicle_status.Header.stamp = ros::Time::now();
                    pub_vehicle_status_.publish(msg_vehicle_status);
                }
                else if (rec[i].ID == 0x178)
                {
                    ReceiveMsg178 msg_factory(rec[i]);
                    can_msgs::wheel_speedl msg_wheel_speedl = msg_factory.getMessage();
                    pub_wheel_speedl_.publish(msg_wheel_speedl);
                }
                else if (rec[i].ID == 0x188)
                {
                    ReceiveMsg188 msg_factory(rec[i]);
                    can_msgs::wheel_speedr msg_wheel_speedr = msg_factory.getMessage();
                    pub_wheel_speedr_.publish(msg_wheel_speedr);
                }
            }
        }
    }
    printf("[can_module] Listener thread exit.\n");
    pthread_exit(0);
}

/*
void *thread_debug(void *param_in)
{
    Param *param = (Param *)param_in;
    uint8_t mrun_num = param->mrun_num;
    VCI_CAN_OBJ recv[3000]; // 接收缓存
    int reclen = 0;  // 获取收到的数据的帧数

    // 初始化CAN设备
    usleep(100000);
    if (VCI_InitCAN(VCI_USBCAN2, CAN_id, param->debug_can_port, &config) != 1) // params: 设备类型, 设备索引, can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Init CAN-" << param->debug_can_port + 1);
        VCI_CloseDevice(VCI_USBCAN2, CAN_id);
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Init CAN-" << param->debug_can_port + 1);
    }
    // 启动CAN设备, CAN_id = 0为默认,main_can_port = 0 对应CAN1
    usleep(100000);
    if (VCI_StartCAN(VCI_USBCAN2, CAN_id, param->debug_can_port) != 1) // params: 设备类型,设备索引,can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Start CAN-" << param->debug_can_port + 1);
        if (VCI_ResetCAN(VCI_USBCAN2, CAN_id, param->debug_can_port) != 1)
        {
            ROS_ERROR_STREAM("[can_module] Try to reset can but failed, CAN-: " << param->debug_can_port + 1);
            VCI_CloseDevice(VCI_USBCAN2, 0);
            exit(1);
        }
        else
        {
            ROS_INFO_STREAM("[can_module] reset can success");
        }
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Start CAN-" << param->debug_can_port + 1);
    }
    while (mrun_num & 0x0f)
    {
        reclen = VCI_Receive(VCI_USBCAN2, 0, param->debug_can_port, recv, 3000, 100);
        if (reclen > 0)
        {
            for (int i = 0; i < reclen; i++)
            {
                if (recv[i].ID == 0x51) // vehicle status
                {
                    VehicleStatusMsg msg_factory(recv[i]);
                    msg_factory.print();
                }
                else if (recv[i].ID == 0x122)
                {
                    fprintf(stdout, "Recv ecu Msg->");
                    SendMsg122 msg_factory(recv[i]);
                    msg_factory.print();
                }
            }
        }
    }
    printf("[can_module] Listener-1 thread exit.\n"); //退出接收线程
    pthread_exit(0);
}
*/

CAN_app::CAN_app() : p_nh("~")
{
    initROS();

    // 打开CAN设备
    usleep(100000);
    int _ecode = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
    if (_ecode == 1) // params: 设备类型,设备索引,预留位(通常为0)  // 设备只需要打开一次
    {
        ROS_INFO_STREAM("[can_module] open device success\n");
    }
    else
    {
        printf("[can module] open USB2CAN device failed once\n");
        printf("[can module] Error code: %d", _ecode); // 返回值=1，表示操作成功； =0表示操作失败； =-1表示USB-CAN设备不存在或USB掉线
        printf("[can module] try to reset CAN-port 1 and 2 to solve this error");
        VCI_ResetCAN(VCI_USBCAN2, CAN_id, 0); //复位CAN通道 //上一次can未正确关闭会导致打不开,需要重新"拔插"一次
        VCI_ResetCAN(VCI_USBCAN2, CAN_id, 1);
        _ecode = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
        if (_ecode == 1)
        {
            ROS_INFO_STREAM("[can_module] open device success\n");
        }
        else
        {
            ROS_WARN_STREAM("[can_module] open device still failed, exit.\n");
            exit(1);
        }
    }

    config.AccCode = 0;  // 帧过滤验收码,详见说明文档及VCI_InitCAN  // 与AccMask共同决定哪些帧可以被接收
    config.AccMask = 0xFFFFFFFF; // 帧过滤屏蔽码,当前表示全部接收 TODO
    config.Filter = 2; // 0/1 接收所有类型;2 只接收标准帧;3 只接收扩展帧
    config.Timing0 = 0x00; // 这两个共同设置波率,当前表示500k,其他请参照说明文档
    config.Timing1 = 0x1C;
    config.Mode = 0; // =0 正常模式; =1 只监听; =2 自发自收(回环模式)

    // 初始化CAN设备,
    usleep(100000);
    if (VCI_InitCAN(VCI_USBCAN2, CAN_id, main_can_port, &config) != 1) // params: 设备类型, 设备索引, can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Init CAN-" << main_can_port + 1);
        VCI_CloseDevice(VCI_USBCAN2, CAN_id);
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Init CAN-" << main_can_port + 1);
    }
    
    // 启动CAN设备, CAN_id = 0为默认,main_can_port = 0 对应CAN1
    usleep(100000);
    if (VCI_StartCAN(VCI_USBCAN2, CAN_id, main_can_port) != 1) // params: 设备类型,设备索引,can通道编号
    {
        ROS_ERROR_STREAM("[can_module] ERROR Start CAN-" << main_can_port + 1);
        if (VCI_ResetCAN(VCI_USBCAN2, CAN_id, main_can_port) != 1)
        {
            ROS_ERROR_STREAM("[caadvertisen_module] Try to reset can but failed, CAN-: " << main_can_port + 1);
            VCI_CloseDevice(VCI_USBCAN2, 0);
            exit(1);
        }
        else
        {
            ROS_INFO_STREAM("[can_module] reset can success");
        }
    }
    else
    {
        ROS_INFO_STREAM("[can_module] SUCCESS Start CAN-" << main_can_port + 1);
    }
}

CAN_app::~CAN_app()
{
    printf("[can module] try to ResetCAN...\n");
    usleep(100000);                                   //延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, CAN_id, main_can_port); //复位CAN通道。
    printf("[can module] ResetCan success\n");
    printf("[can module] try to CloseCAN...\n");
    usleep(100000);                  //延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0); //关闭设备。
    printf("[can module] CloseCAN success! done\n");
}

void CAN_app::initROS()
{
    //p_nh.param<bool>("debug_mode", param_debug, false);
    //p_nh.param<int>("debug_can_id", debug_can_port, 1); // 默认读取can1数据来debug
    //debug_can_port = debug_can_port - 1;
    p_nh.param<int>("send_msg_id", send_msg_id, 120);
    p_nh.param<int>("main_can_id", main_can_port, 2);
    p_nh.param<bool>("show_sending_msg", param_show_sending_msg, false);
    main_can_port = main_can_port - 1;
   
    if(send_msg_id == 120)
    {
        sub_ecu = nh.subscribe<can_msgs::ecu_120>("ecu", 1, &CAN_app::ecu_cb120, this);
        pre_steer = 0.;
    }
//    else if(send_msg_id == 121)
//    {
//        sub_ecu = nh.subscribe<can_msgs::ecu_121>("ecu", 10, &CAN_app::ecu_cb121, this);
//        pre_steer_f = 0.;
//        pre_steer_r = 0.;
//    }
//    else if(send_msg_id == 122)
//    {
//        sub_ecu = nh.subscribe<can_msgs::ecu_122>("ecu", 10, &CAN_app::ecu_cb122, this);
//    }

    pub_vehicle_status = nh.advertise<can_msgs::vehicle_status>("vehicle_status", 100);
    pub_wheel_speedl = nh.advertise<can_msgs::wheel_speedl>("wheel_speedl", 100);
    pub_wheel_speedr = nh.advertise<can_msgs::wheel_speedr>("wheel_speedr", 100);
}

void CAN_app::ecu_cb120(const can_msgs::ecu_120::ConstPtr &msg)
{
    can_msgs::ecu_120 _msg = *msg;
    SendMsg120 sendMsg(_msg, pre_steer);
    pre_steer = _msg.steer;
    VCI_CAN_OBJ sendData = sendMsg.getMessage();

    if (VCI_Transmit(VCI_USBCAN2, CAN_id, main_can_port, &sendData, 1) != 1)
    {
        ROS_WARN_STREAM("[can_module] Transmit failed once.");
        ROS_INFO_STREAM("VCI_USBCAN2 : " << VCI_USBCAN2);
        ROS_INFO_STREAM("CAN_id     : " << CAN_id);
        ROS_INFO_STREAM("CAN_port   : " << main_can_port + 1);
        ROS_INFO_STREAM("sendData : ");
        // sendMsg.print(); // 以十六进制的形式打印ecu->can消息
    }
    if (param_show_sending_msg)
    {
        sendMsg.print();
    }
}
void CAN_app::ecu_cb121(const can_msgs::ecu_121::ConstPtr &msg)
{
    can_msgs::ecu_121 _msg = *msg;
    SendMsg121 sendMsg(_msg, pre_steer_f, pre_steer_r);
    pre_steer_f = _msg.anglef;
    pre_steer_r = _msg.angler;
    VCI_CAN_OBJ sendData = sendMsg.getMessage();

    if (VCI_Transmit(VCI_USBCAN2, CAN_id, main_can_port, &sendData, 1) != 1)
    {
        ROS_WARN_STREAM("[can_module] Transmit failed once.");
        ROS_INFO_STREAM("VCI_USBCAN2 : " << VCI_USBCAN2);
        ROS_INFO_STREAM("CAN_id     : " << CAN_id);
        ROS_INFO_STREAM("CAN_port   : " << main_can_port + 1);
        ROS_INFO_STREAM("sendData : ");
        // sendMsg.print(); // 以十六进制的形式打印ecu->can消息
    }
    if (param_show_sending_msg)
    {
        sendMsg.print();
    }
}
void CAN_app::ecu_cb122(const can_msgs::ecu_122::ConstPtr &msg)
{
    can_msgs::ecu_122 _msg = *msg;
    SendMsg122 sendMsg(_msg);
    VCI_CAN_OBJ sendData = sendMsg.getMessage();

    if (VCI_Transmit(VCI_USBCAN2, CAN_id, main_can_port, &sendData, 1) != 1)
    {
        ROS_WARN_STREAM("[can_module] Transmit failed once.");
        ROS_INFO_STREAM("VCI_USBCAN2 : " << VCI_USBCAN2);
        ROS_INFO_STREAM("CAN_id     : " << CAN_id);
        ROS_INFO_STREAM("CAN_port   : " << main_can_port + 1);
        ROS_INFO_STREAM("sendData : ");
        // sendMsg.print(); // 以十六进制的形式打印ecu->can消息
    }
    if (param_show_sending_msg)
    {
        sendMsg.print();
    }
}

class ThreadGuard
{
private:
    pthread_t &t1_;
    Param *param_;

public:
    explicit ThreadGuard(pthread_t &t1, Param *param) : t1_(t1), param_(param){};
    ~ThreadGuard()
    {
        if (!t1_)
        {
            param->mrun_num = 0;
            pthread_join(t1_, NULL);
        }
    }
    ThreadGuard(const ThreadGuard &) = delete;
    ThreadGuard &operator=(const ThreadGuard &) = delete;
};

void CAN_app::run()
{
    param = new Param();
    param->mrun_num = 1;
    param->name = "Listener Thread";
    param->wheel_speedl_publisher = pub_wheel_speedl;
    param->wheel_speedr_publisher = pub_wheel_speedr;
    param->vehicle_status_publisher = pub_vehicle_status;
    //param->debug_can_port = debug_can_port;
    param->main_can_port = main_can_port;

    int ret = pthread_create(&threadid, NULL, receive_func, param);
    ThreadGuard t1{threadid, param};
    /*
    pthread_t _thread_debug;
    if (param_debug)
    {
        // 目前can通道1闲置,因此用来做监听,进行debug
        int ret1 = pthread_create(&_thread_debug, NULL, thread_debug, param);
        ThreadGuard t2{_thread_debug, param};
    }
    */
    ros::spin();
    ros::shutdown();
}
};  // namespace USB2CAN
    // 写在最后: 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
