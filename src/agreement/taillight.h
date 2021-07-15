#ifndef __TAILLIGHT_H__
#define __TAILLIGHT_H__

#include "agree_base.h"
#include <std_msgs/ColorRGBA.h>
#define UNDEFINED 0

namespace ZROS{
/**
 * @brief http://www.cocoabuilder.com/archive/cocoa/198570-here-is-code-to-convert-rgb-hsb.html
 * 
 */
typedef struct {float r, g, b;} RGBType;
typedef struct {float h, s, v;} HSVType;

RGBType RGBTypeMake(float r, float g, float b);
HSVType HSVTypeMake(float h, float s, float v);

HSVType RGB2HSV(const RGBType& RGB );
RGBType HSV2RGB(const HSVType& HSV );

class Taillight final: public AgreeBase{
public:
    Taillight(){
        SetFrameName("Taillight"); //设置帧名字
        // SetFrameID(0x4a); //关键帧ID
        SetTrigger(Trigger::Event);//触发方式 [事件]
        Init();
    }

private:
    ros::Subscriber taillight_sub_;
    std_msgs::ColorRGBA color_rgb_;
    RGBType rgb_;
    int mode_;

    void Init(){
        taillight_sub_ = ros_nh_.subscribe("led_interaction", 1, &Taillight::TaillightCallback, this);
    }

    void TaillightCallback(const std_msgs::ColorRGBA& msg){
        if(IsReady()) return;
        rgb_ = RGBTypeMake(msg.r, msg.g, msg.b);
        mode_ = msg.a;
        Ready();
    }

    /**
     * @brief 数据封装
     * 
     */
    void Package(std::vector<uint8_t>& data) override{
        auto hsv = RGB2HSV(rgb_);
        uint32_t output = ((static_cast<int>(hsv.v * 100) & 0x7A) | (static_cast<int>(hsv.s * 100) << 7) | \
                            (static_cast<int>(hsv.h * 360) << 14) | (0 << 23) | (mode_ << 30));

        uint8_t *led_array = reinterpret_cast<uint8_t *>(&output);
        switch (mode_)
        {
            case 0:
            case 1:
            case 2:
            case 3:
                data[0] = 0x02;
                data[1] = 0xA0;
                data[2] = led_array[3];
                data[3] = led_array[2];
                data[4] = led_array[1];
                data[5] = led_array[0];
                data[6] = BCC16(data);
                data[7] = 0xff;
                break;
            case 4:
                data[0] = 0x02;
                data[1] = 0xA1;
                data[2] = led_array[3];
                data[3] = led_array[2];
                data[4] = led_array[1];
                data[5] = led_array[0];
                data[6] = BCC16(data);
                data[7] = 0xff;
                break;
            case 5:
                data[0] = 0x02;
                data[1] = 0xA2;
                data[2] = led_array[3];
                data[3] = led_array[2];
                data[4] = led_array[1];
                data[5] = led_array[0];
                data[6] = BCC16(data);
                data[7] = 0xff;
                break;
            default:
                ROS_INFO("No such LED mode");
        }
    }

    /**
     * @brief 输入数据解析
     * 
     * @param data 输入数据
     */
    void Analyze(const std::vector<uint8_t>& data) override {
        //Null ...
    }
};

} //end namespace ZROS

#endif /*__TAILLIGHT_H__*/