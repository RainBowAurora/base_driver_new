#include "base_driver_new.h"

namespace ZROS{
BaseDriver::BaseDriver():ros_nh_(), ros_param_("~"), base_driver_observer_()
{
    Init();
}


BaseDriver::~BaseDriver()
{
    if(serial_.isOpen()){
        ROS_INFO("Close Serial Port");
        serial_.close();
    }
}

/**
 * @brief 串口和ros初始化
 * 
 */
void BaseDriver::Init()
{
    //Initial ROS
    ros_param_.getParam("serial_port",  serial_port_); 
    ros_param_.getParam("serial_baudrate", serial_baudrate_); 
    ros_param_.getParam("serial_timerout", serial_timerout_);

    ROS_INFO("Get Serial Port: %s", serial_port_.c_str());
    ROS_INFO("Get Serial Baudrate: %d", serial_baudrate_);

    //Initial Serial
    try{
        auto time_out = serial::Timeout::simpleTimeout(serial_timerout_);

        serial_.setPort(serial_port_); //指定串口设备
        serial_.setBaudrate(serial_baudrate_); //指定波特率
        serial_.setTimeout(time_out); //设置读串口超时时间
        serial_.open(); //打开串口

        ROS_INFO("Serial Port initialized: %s", serial_.isOpen()? "True": "False");
    }catch(serial::IOException &e){
        ROS_ERROR("Unable to open port: %s: %d", serial_port_.c_str(), serial_baudrate_);
        ROS_ERROR("%s:%d", e.what(), e.getErrorNumber());
        exit(EXIT_FAILURE); //异常导致程序退出
    } //end try

    //单独创建一个轮询任务来接收串口发来数据
    timers_.push_back(std::move(ZROS::COMMON::Timer(50 ,[&]{
        std::vector<uint8_t> temp_read_data{};
        serial_.read(temp_read_data, 8); 
        if(!temp_read_data.empty()){ //读取到下位机传来的数据，创建一个新的消息
            base_driver_observer_.CreateMessage(temp_read_data);
        }
    }, false)));
    timers_.back().Start(); //开始读取任务
}

/**
 * @brief 协议注册
 * 
 * @param observer 
 */
void BaseDriver::Registered(AgreeBasePtr observer, const uint32_t& period)
{
    timers_.push_back(std::move(ZROS::COMMON::Timer(period, [this, observer, period]{ //注册一个写串口任务
            if(observer->IsReady()){
                this->serial_.write(observer->Upload());
            }
    }, false)));
    timers_.back().Start(); //开始写串口任务
    
    base_driver_observer_.Attach(observer); //观察者登记
}

/**
 * @brief 常规任务
 * 
 */
void BaseDriver::Run()
{
    ros::Rate rate(20); // 20[Hz]
    while(ros::ok()){
        rate.sleep(); // loop rate 20Hz
        //Do Something....
        ros::spinOnce(); // run topic callback
    }
}


} //end namespace ZROS