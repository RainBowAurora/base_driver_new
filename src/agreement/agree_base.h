#ifndef __AGREE_BASE_H__
#define __AGREE_BASE_H__

/**
 * @file agree_base.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2021-07-12
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <atomic>

#include "timer/timer.h"

namespace ZROS{
/**
 * @brief 触发方式
 * 
 */
enum Trigger{
    Cycle, //循环触发
    Event //事件触发
};

/**
 * @brief 供能帧构造
 * 
 */
struct Frame{
    std::string name; //帧名
    uint8_t id; //关键帧id标识
    std::vector<uint8_t> data; //帧数据
    Trigger trigger; //触发方式
};


class AgreeBase{
public:
    AgreeBase(): ros_nh_(), ros_param_("~"), is_ready_(false) {}
    virtual ~AgreeBase() {}

    bool IsReady() const { 
        return  (frame_.trigger == Trigger::Cycle? true : is_ready_.load()); 
    }

    void Update(const std::vector<uint8_t>& data){
        std::lock_guard<std::mutex> lock(mutex_); 
        if(data.size() != 8) return; //不是一个完整的帧
        if(data[1] != GetFrameId()) return; //关键帧不匹配
        if(data[6] != BCC16(data)) return; //校验不通过
        Analyze(data); //协议解析
    }

    std::vector<uint8_t> Upload(){
        is_ready_.store(false); //标志位清除
        Package(frame_.data); //数据封装
        return GetData(); //获取数据
    }

    std::vector<uint8_t> GetData() const { return frame_.data; }
    std::string GetName() const { return frame_.name; }
    uint8_t GetFrameId() const { return frame_.id; }
    
protected:
    // ROS Handle
    ros::NodeHandle ros_nh_;
    ros::NodeHandle ros_param_;

    // 线程同步相关
    std::atomic<bool> is_ready_;
    std::mutex mutex_;

    //帧数据默认值
    struct Frame frame_ = {
        .name = "Unknow",
        .id = 0x00,
        .data = std::vector<uint8_t>(8,0xFF),
        .trigger = Trigger::Cycle
    };

    void SetFrameName(const std::string& name) { frame_.name = name; }
    void SetFrameID(const uint8_t id) { frame_.id = id; }
    void SetTrigger(const Trigger& trigger) { frame_.trigger = trigger; }

    /**
     * @brief 输入数据解析
     * 
     */
    virtual void Analyze(const std::vector<uint8_t>&) = 0; 

    /**
     * @brief 封装发送数据
     * 
     */
    virtual void Package(std::vector<uint8_t>&) = 0;

    /**
     * @brief 触发上报事件
     * 
     */
    void Ready() { is_ready_.store(true); }

    /**
     * @brief 十六位异或校验
     * 
     * @param data 输入数据 
     * @return uint8_t 异或计算出的校验码
     */
    uint8_t BCC16(const std::vector<uint8_t>& data){
        if(data.size() < 5) return 0xFF;
        return (data[0]^data[1]^data[2]^\
                data[3]^data[4]^data[5]);
    }
};

    using AgreeBasePtr = std::shared_ptr<AgreeBase>; //智能指针
    
} // end namespace ZROS

#endif /*__AGREE_BASE_H__*/