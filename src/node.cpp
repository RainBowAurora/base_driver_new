#include <ros/ros.h>
#include "base_driver/base_driver_new.h"

#include "agreement/battery.h"
#include "agreement/encode.h"
#include "agreement/light.h"
#include "agreement/lock.h"
#include "agreement/motor.h"
#include "agreement/taillight.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "base_driver_node");

    ZROS::BaseDriver base_driver;
    
    base_driver.Registered(std::make_shared<ZROS::Battery>(), 100); //注册电池电量
    base_driver.Registered(std::make_shared<ZROS::EncodeLeft>(), 50); //注册左编码器
    base_driver.Registered(std::make_shared<ZROS::EncodeRight>(), 50); //注册右编码器
    base_driver.Registered(std::make_shared<ZROS::LeftVel>(), 20); //注册左轮速度
    base_driver.Registered(std::make_shared<ZROS::RightVel>(), 20); //注册右轮速度
    base_driver.Registered(std::make_shared<ZROS::Light>(), 200); //注册前灯
    base_driver.Registered(std::make_shared<ZROS::UnLock>(), 1000); //注册写锁
    base_driver.Registered(std::make_shared<ZROS::LockInfo>(), 1000); //注册读锁
    base_driver.Registered(std::make_shared<ZROS::Taillight>(), 500); //注册尾灯

    base_driver.Run(); 

    return 0;
}