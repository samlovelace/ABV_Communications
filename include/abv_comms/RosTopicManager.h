#include <rclcpp/rclcpp.hpp>
#include "abv_idl/msg/abv_command.hpp"
#include "abv_idl/msg/abv_response.hpp"
#include "abv_idl/msg/abv_state.hpp"
#include <map>
#include <thread>


class RosTopicManager : public rclcpp::Node
{
public:
    RosTopicManager(/* args */);
    ~RosTopicManager();

    template<typename T>
    void createPublisher(const std::string& topicName) {
        auto publisher = this->create_publisher<T>(topicName, 10);
        mPublishers[topicName] = std::dynamic_pointer_cast<rclcpp::PublisherBase>(publisher);
    }

    template<typename T>
    void publishMessage(const std::string& topicName, const T& message) 
    {
        auto it = mPublishers.find(topicName);
        
        if (it != mPublishers.end()) 
        {
            // Cast PublisherBase back to Publisher<T>
            auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(it->second);
            if (pub) 
            {
                pub->publish(message);
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to cast publisher for topic: %s", topicName.c_str());
            }
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Publisher not found for topic: %s", topicName.c_str());
        }
    }

    template<typename T>
    void createSubscriber(const std::string& aTopicName, std::function<void(const typename T::SharedPtr)> aCallback)
    {
        auto subscriber = this->create_subscription<T>(aTopicName, 10, aCallback);
        mSubscribers[aTopicName] = std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriber);
    }

    // void createModeSubscriber(const std::string& topicName, std::function<void(const std_msgs::msg::String::SharedPtr)> callback) {
    //     auto subscriber = this->create_subscription<std_msgs::msg::String>(topicName, 10, callback);
    //     mSubscribers[topicName] = std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriber);

    //     mSubscribers[topicName].reset();
    // }

    // void createStateSubscriber(const std::string& topicName, std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr)> callback) {
    //     auto subscriber = this->create_subscription<std_msgs::msg::Float64MultiArray>(topicName, 10, callback);
    //     mSubscribers[topicName] = std::dynamic_pointer_cast<rclcpp::SubscriptionBase>(subscriber);
    // }

    bool init();
    //bool initializeController(CONTROL_MODE aMode);
    bool shutdown();

private:
    std::map<std::string, rclcpp::PublisherBase::SharedPtr> mPublishers;
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> mSubscribers; 

};



