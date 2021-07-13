#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "agree_base.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>

namespace ZROS{

class Motor : public AgreeBase{
public:
    Motor(){
        SetFrameID(0x02); //关键帧ID
        SetTrigger(Trigger::Event);//触发方式 [循环]
        Init();
    }
private:
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber auto_vel_sub_;
    ros::Subscriber twist_vel_sub_;
    ros::Subscriber start_flag_sub_;
    ros::Publisher motor_pub_;

    int32_t start_flag_data_ = 0;

    float ROBOT_RADIUS = float(540.0/ 2.0/ 1000.0);
    float WHEEL_RADIUS = float(200.0/ 2.0/ 1000.0);

    void Init(){
        //Get Praram
        ros_param_.getParam("robot_radius", ROBOT_RADIUS);
        ros_param_.getParam("wheel_radius", WHEEL_RADIUS);

        //Ros Subscribe 
        cmd_vel_sub_ = ros_nh_.subscribe("/cmd_vel", 2, &Motor::callbackCmdVel ,this);
        auto_vel_sub_ = ros_nh_.subscribe("/auto_vel", 2, &Motor::callbackAutoVel ,this);
        twist_vel_sub_ = ros_nh_.subscribe("/twist_new", 2, &Motor::callbacktwistVel ,this);
        start_flag_sub_ = ros_nh_.subscribe("/start_flag", 2, &Motor::callbackStartFlag ,this);
        //Ros Publisher
        motor_pub_ = ros_nh_.advertise<std_msgs::Int32MultiArray>("/motor_error", 2);
    }
    void callbackCmdVel(const geometry_msgs::Twist& msg){
        if(start_flag_data_ == 0) update_vel(msg);
    }
    void callbackAutoVel(const geometry_msgs::Twist& msg){
        if(start_flag_data_ == 1) update_vel(msg);
    }
    void callbacktwistVel(const geometry_msgs::Twist& msg){
        if(start_flag_data_ == 2) update_vel(msg);
    }
    void callbackStartFlag(const std_msgs::Int32& msg){
        start_flag_data_ = msg.data;
    }

    /**
     * @brief 输入数据解析
     * 
     * @param data 输入数据
     */
    void Analyze(const std::vector<uint8_t>& data) override {
        std_msgs::Int32MultiArray error_array;
        error_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
        error_array.layout.dim[0].size = 4;
        error_array.layout.dim[0].stride = 1;
        error_array.layout.dim[0].label = "error";
        error_array.data.push_back(data[2]);
        error_array.data.push_back(data[3]);
        error_array.data.push_back(data[4]);
        error_array.data.push_back(data[5]);
        motor_pub_.publish(error_array);
    }

    /**
     * @brief 机器人速度转换位坐右轮的速度
     * 
     * @param data 
     */
    void update_vel(const geometry_msgs::Twist& data)
    {
        if(IsReady ()) return;
        float eps = 1e-5;
        if(fabs(data.angular.z) > eps ){
            if(fabs(data.linear.x) > eps){
                float radius = data.linear.x / data.angular.z;
                left_vel_data_ =  (radius - ROBOT_RADIUS) * data.angular.z;
                right_vel_data_ =  (radius + ROBOT_RADIUS) * data.angular.z;
            }else{
                left_vel_data_ = - data.angular.z * ROBOT_RADIUS;
                right_vel_data_ = data.angular.z * ROBOT_RADIUS; 
            }
        }else{
            left_vel_data_ = data.linear.x;
            right_vel_data_ = data.linear.x;
        }
        std::cout << "left_vel_data_: " << left_vel_data_ <<  "\tright_vel_data_: " << right_vel_data_ << std::endl;
        Ready();
    }

protected:
    float left_vel_data_; //左轮速度
    float right_vel_data_; //右轮速度
};


class LeftVel final : public Motor{
public:
    LeftVel(){
        SetFrameName("LeftVel"); //设置帧名字
    }
private:
    void Package(std::vector<uint8_t>& data) override{
        uint8_t *temp_vel_array = reinterpret_cast<uint8_t*>(&left_vel_data_);
        data[0] = 0x02;
        data[1] = 0x23;
        data[2] = temp_vel_array[0];
        data[3] = temp_vel_array[1];
        data[4] = temp_vel_array[2];
        data[5] = temp_vel_array[3];
        data[6] = BCC16(frame_.data);
        data[7] = 0xff;
    }
};

class RightVel final : public Motor{
public:
    RightVel(){
        SetFrameName("RightVel"); //设置帧名字
    }

private:
    void Package(std::vector<uint8_t>& data) override{
        uint8_t *temp_vel_array = reinterpret_cast<uint8_t*>(&right_vel_data_);
        data[0] = 0x02;
        data[1] = 0x24;
        data[2] = temp_vel_array[0];
        data[3] = temp_vel_array[1];
        data[4] = temp_vel_array[2];
        data[5] = temp_vel_array[3];
        data[6] = BCC16(frame_.data);
        data[7] = 0xff;
    }
};

} //end namespace ZROS

#endif /*__MOTOR_H__*/