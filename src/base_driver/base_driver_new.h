#ifndef __BASE_DRIVER_NEW_H__
#define __BASE_DRIVER_NEW_H__

/**
 * @file base_driver_new.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2021-07-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <string>

#include <serial/serial.h>
#include <macros/disable_class_cpoy.h>

#include "timer/timer.h"
#include "base_driver_observer.h"

namespace ZROS{
class BaseDriver final{
public:
    BaseDriver();
    ~BaseDriver();

    void Registered(AgreeBasePtr observer, const uint32_t& period);
    
    void Run();

private: 
    void Init();

    ros::NodeHandle ros_nh_;
    ros::NodeHandle ros_param_;
    
    serial::Serial serial_; //ros-serial
    
    std::string serial_port_ = "/dev/controlboard";
    int serial_baudrate_ = 115200;
    int serial_timerout_ = 2000;

    std::vector<ZROS::COMMON::Timer> timers_; //任务管理器
    BaseDriverObserver base_driver_observer_; //观察者

    DISABLE_COPY_AND_ASSIGN(BaseDriver);
};
} //end namespace ZROS
#endif /*__BASE_DRIVER_NEW_H__*/