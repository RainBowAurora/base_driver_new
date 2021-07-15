#ifndef __ENCODE_H__
#define __ENCODE_H__

#include "agree_base.h"
#include <zr_msgs/motor_info.h>

namespace ZROS{

class Encode: public AgreeBase{
public:
    Encode(){
        Init();
    }

private:
    void Init(){
        motor_pub_ =  ros_nh_.advertise<zr_msgs::motor_info>("motor_info",2);
    }

protected:
    ros::Publisher motor_pub_;
    static zr_msgs::motor_info motor_info_;


};

class EncodeLeft final: public Encode{
public:
    EncodeLeft(){
        SetFrameName("EncodeLeft"); //设置帧名字
        SetFrameID(0x42); //关键帧ID
        SetTrigger(Trigger::Cycle);//触发方式 [循环]
    }

private:
    void Package(std::vector<uint8_t>& data) override{
        data[0] = 0x02;
        data[1] = GetFrameId();
        data[2] = data[3] = 0x00;
        data[4] = data[5] = 0x00;
        data[6] = BCC16(data);
        data[7] = 0xff;
    }

    void Analyze(const std::vector<uint8_t>& data) override {
        motor_info_.left_vel = *(float*)(&data[2]);
        motor_pub_.publish(motor_info_);
    }

};

class EncodeRight final: public Encode{
public:
    EncodeRight(){
        SetFrameName("EncodeRight"); //设置帧名字
        SetFrameID(0x43); //关键帧ID
        SetTrigger(Trigger::Cycle);//触发方式 [循环]
    }

private:
    void Package(std::vector<uint8_t>& data) override{
        data[0] = 0x02;
        data[1] = GetFrameId();
        data[2] = data[3] = 0x00;
        data[4] = data[5] = 0x00;
        data[6] = BCC16(data);
        data[7] = 0xff;
    }

    void Analyze(const std::vector<uint8_t>& data) override {
        motor_info_.right_vel = *(float*)(&data[2]);
         motor_pub_.publish(motor_info_);
    }

};

} //end namespace ZROS


#endif /*__ENCODE_H__*/