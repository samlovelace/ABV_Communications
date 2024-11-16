
#include "abv_comms/RosTopicManager.h"

RosTopicManager::RosTopicManager(/* args */) : rclcpp::Node("Test")
{

}

RosTopicManager::~RosTopicManager()
{
}

bool RosTopicManager::init()
{
    rclcpp::init(0, nullptr);

    if(!rclcpp::ok())
    {
        return false; 
    }

    return true; 
}

bool RosTopicManager::shutdown()
{
    rclcpp::shutdown(); 

    if(rclcpp::ok())
    {
        return false; 
    }

    return true;
}

