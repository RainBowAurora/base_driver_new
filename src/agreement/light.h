#ifndef __LIGHT_H__
#define __LIGHT_H__

#include "agree_base.h"
#include <std_msgs/Int32.h>

namespace ZROS{

class Light final: public AgreeBase{
public:
    Light(): light_data_(4, 0x00){
        SetFrameName("Light"); //设置帧名字
        SetFrameID(0x2f); //关键帧ID
        SetTrigger(Trigger::Event);//触发方式 [事件]
        Init();
    }

private:
    ros::Subscriber light_sub_;
    std::vector<uint8_t> light_data_;

    void Init(){
        light_sub_ = ros_nh_.subscribe("/led_control", 1, &Light::LightCallback, this);
    }

    /**
     * @brief 数据封装
     * 
     */
    void Package(std::vector<uint8_t>& data) override{
        data[0] = 0x02;
        data[1] = GetFrameId();
        data[2] = light_data_[0];
        data[3] = light_data_[1];
        data[4] = light_data_[2];
        data[5] = light_data_[3];
        data[6] = BCC16(frame_.data);
        data[7] = 0xff;
    }

    /**
     * @brief 输入数据解析
     * 
     * @param data 输入数据
     */
    void Analyze(const std::vector<uint8_t>& data) override {
        // Null..
    }


    void LightCallback(const std_msgs::Int32& msg){
        if( IsReady() ) return;

        switch (msg.data){
            case 1:
                light_data_ = std::vector<uint8_t>{0x01,0x32,0x00,0x00}; //left 常亮
                break;
            case 2:
                light_data_ = std::vector<uint8_t>{0x02,0x00,0x00,0x00}; //right 常亮
                break;
            case 3:
                light_data_ = std::vector<uint8_t>{0x03,0x00,0x00,0x00}; //常亮
                break;
            case 4:
                light_data_ = std::vector<uint8_t>{0x18,0x32,0x00,0x00}; //第二位0x10表示左灯闪烁，0x08表示右灯闪烁
                break;
            case 5:
                light_data_ = std::vector<uint8_t>{0x00,0x00,0x00,0x00}; //常灭
                break;
            case -1:
                light_data_ = std::vector<uint8_t>{0x00,0x00,0x00,0x00};
                break;
        }

        Ready();
    }

};

} //end namespace ZROS

#endif /*__LIGHT_H__*/