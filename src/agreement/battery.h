#ifndef __BATTERY_H__
#define __BATTERY_H__

#include "agree_base.h"
#include <zr_msgs/battery_info.h>
#include <std_msgs/Bool.h>

namespace ZROS{
class Battery final: public AgreeBase{
public:
    Battery(){
        SetFrameName("Battery"); //设置帧名字
        SetFrameID(0x4a); //关键帧ID
        SetTrigger(Trigger::Cycle);//触发方式 [循环]
        Init();
    }
    
private:
    ros::Publisher battery_pub_; //电池电量
    ros::Publisher low_battery_pub_; //低电量警报

    float low_battery_threshold_ = 20.0;

    /**
     * @brief ros topic 初始化
     * 
     */
    void Init(){
        ros_param_.getParam("low_battery_threshold", low_battery_threshold_);
        
        ROS_INFO("low_battery_threshold: %f", low_battery_threshold_);

        battery_pub_ = ros_nh_.advertise<zr_msgs::battery_info>("battery_info",2);
        low_battery_pub_ = ros_nh_.advertise<std_msgs::Bool>("low_battery", 2, true);
    }
    

    /**
     * @brief 数据封装
     * 
     */
    void Package(std::vector<uint8_t>& data) override{
        data[0] = 0x02;
        data[1] = frame_.id;
        data[2] = data[3] = 0x00;
        data[4] = data[5] = 0x00;
        data[6] = BCC16(frame_.data);
        data[7] = 0xff;
    }

    /**
     * @brief 输入数据解析
     * 
     * @param data 输入数据
     */
    void Analyze(const std::vector<uint8_t>& data) override {
        zr_msgs::battery_info temp_volt;
        float volt = ((data[3] & 0xff) << 8 | (data[2] & 0xff));
        temp_volt.pct = volt;
        battery_pub_.publish(temp_volt); //发送

        if(volt < low_battery_threshold_){
            std_msgs::Bool flag;
            flag.data = true;
            low_battery_pub_.publish(flag);
        }
    }
};



} //end namespace ZROS

#endif /*__BATTERY_H__*/