#ifndef __LOCK_H__
#define __LOCK_H__

#include "agree_base.h"
#include <zr_msgs/locker_info.h>
#include <std_msgs/Int32.h>

namespace ZROS{

class LockInfo final: public AgreeBase{
public:
    LockInfo(){
        SetFrameName("UnLock"); //设置帧名字
        SetFrameID(0x4c); //关键帧ID
        SetTrigger(Trigger::Cycle);//触发方式
        Init();
    }

private:
    ros::Publisher lock_info_pub_;

    void Init(){
        lock_info_pub_ = ros_nh_.advertise<zr_msgs::locker_info>("/locker_info", 2);
    } 

    void Analyze(const std::vector<uint8_t>& data) override {
        std::vector<uint8_t> locker_array(8, 0x00);

        locker_array[0] = data[2];
        locker_array[1] = data[3];
        locker_array[2] = data[4];
        locker_array[3] = data[5];

        uint32_t locker_data = *(uint32_t *)(&locker_array);

        zr_msgs::locker_info local_lock_status;
        local_lock_status.all_closed = false;

        for(int i = 0; i < 32; i++){
            if(locker_data & (1 << i)){
                local_lock_status.all_closed |= true;
                local_lock_status.locker_status.push_back("closed");
            }else{
                local_lock_status.all_closed |= false;
                local_lock_status.locker_status.push_back("opened");
            }
        }

        lock_info_pub_.publish(local_lock_status);
    }

    void Package(std::vector<uint8_t>& data) override{
        data[0] = 0x02;
        data[1] = GetFrameId(); 
        data[2] = data[3] = 0x00;
        data[4] = data[4] = 0x00;
        data[6] = BCC16(data);
        data[7] = 0xff;
    }

};

class UnLock final : public AgreeBase{
public:
    UnLock(){
        SetFrameName("UnLock"); //设置帧名字
        SetFrameID(0x27); //关键帧ID
        SetTrigger(Trigger::Event);//触发方式 [事件]
        Init();
    }

private:
    ros::Subscriber unlock_sub_;
    uint8_t locker_;

    void Init(){
        unlock_sub_ = ros_nh_.subscribe("/unlock", 10, &UnLock::UnlockCallback, this);
    }

    void Package(std::vector<uint8_t>& data) override{
        data[0] = 0x02;
        data[1] = GetFrameId(); 
        data[2] = locker_;
        data[3] = 0x00;
        data[4] = data[5] = 0x00;
        data[6] = BCC16(data);
        data[7] = 0xff;
    }

    void Analyze(const std::vector<uint8_t>& data) override {
        //Null...
    }

    void UnlockCallback(const std_msgs::Int32& msg){
        if( IsReady() ) return;
        locker_ = static_cast<uint8_t>(msg.data);
        Ready();
    }
};

} //end namespace ZROS

#endif /*__LOCK_H__*/