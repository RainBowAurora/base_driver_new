#ifndef __BASE_DRIVER_OBSERVER_H__
#define __BASE_DRIVER_OBSERVER_H__

/**
 * @file baser_driver_observer.h
 * @author XiaoyuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2021-07-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <iostream>
#include <list>
#include <vector>
#include <memory>
#include "macros/declare_singleton.h"
#include "macros/disable_class_cpoy.h"
#include "thread_pool/thread_pool.h"
#include "../agreement/agree_base.h"

namespace ZROS{
class BaseDriverObserver final{
public:
    BaseDriverObserver() {}
    ~BaseDriverObserver() {}
    
    std::size_t Size() const { return list_observer_.size(); }
    bool Empty() const { return list_observer_.empty(); }

    /**
     * @brief 添加一个观察者
     * 
     * 
     * @param observer 
     */
    void Attach(AgreeBasePtr observer){
        list_observer_.push_back(observer);
    }

    /**
     * @brief 剔除一个观察者
     * 
     * @param observer 
     */
    void Detach(AgreeBasePtr observer){
        list_observer_.remove(observer);
    }

    /**
     * @brief 通知所有观察值，有新的消息需要处理
     * 
     */
    void Notify(){
        // HowManyObserver();
        for(auto iterator = list_observer_.begin();  \
            iterator != list_observer_.end(); iterator++){
            thread_pool_.add([=]{ //向线程池中丢任务
                (*iterator)->Update(message_);
            });
        }
    }

    /**
     * @brief Create a Message object
     * 
     * @param message 消息
     */
    void CreateMessage(std::vector<uint8_t>& message){
        if(message.empty()) return; //empty message 
        this->message_ = message;
        Notify();
    }

private:
    std::list<AgreeBasePtr> list_observer_; 
    ZROS::COMMON::ThreadPool thread_pool_; //线程池
    std::vector<uint8_t> message_; //消息

    /**
     * @brief 打印现在又多少个观察者
     * 
     */
    void HowManyObserver(){
        std::cout << "There are " << this->Size() << " observers in the list.\n";
    }

    DISABLE_COPY_AND_ASSIGN(BaseDriverObserver);
};

}


#endif /*__BASE_DRIVER_OBSERVER_H__*/