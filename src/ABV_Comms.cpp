#include "abv_comms/ABV_Comms.h"

ABV_Comms::ABV_Comms(/* args */) : mTopicManager(std::make_shared<RosTopicManager>()), mModeCommandAckd(false), mVehicleState(6, 0)
{
    // setup needed topics 
    mTopicManager->createPublisher<abv_idl::msg::AbvCommand>("abv_command");
    mTopicManager->createSubscriber<abv_idl::msg::AbvResponse>("abv_response", std::bind(&ABV_Comms::responseCallback, this, std::placeholders::_1));
    mTopicManager->createSubscriber<abv_idl::msg::AbvState>("abv_state", std::bind(&ABV_Comms::stateCallback, this, std::placeholders::_1));
}

ABV_Comms::~ABV_Comms()
{
}

bool ABV_Comms::init()
{
    printf("ABV_Comms Library Initializing ROS2 Communications\n"); 
    rclcpp::init(0, nullptr);

    if(!rclcpp::ok())
    {
        return false; 
    }

    return true; 
}

void ABV_Comms::spinNode()
{
    printf("Spinning node\n");
    std::thread spinThread([this]() {
        rclcpp::spin(mTopicManager->get_node_base_interface());
    });
    spinThread.detach();
}

bool ABV_Comms::shutdown()
{
    rclcpp::shutdown(); 
    return true; 
}

bool ABV_Comms::initializeController(ABV_Comms::CONTROL_MODE aMode)
{
    // save the mode configured by the user
    mMode = aMode; 

    switch (aMode)
    {
    case ABV_Comms::CONTROL_MODE::THRUSTER:
        mCommand.set__type("thruster"); 
        break;
    case ABV_Comms::CONTROL_MODE::POSE:
        mCommand.set__type("pose");
        break;  
    default:
        break;
    }

    // // setup needed topics 
    // mTopicManager->createPublisher<abv_idl::msg::AbvCommand>("abv_command");
    // mTopicManager->createSubscriber<abv_idl::msg::AbvResponse>("abv_response", std::bind(&ABV_Comms::responseCallback, this, std::placeholders::_1));
    // mTopicManager->createSubscriber<abv_idl::msg::AbvState>("abv_state", std::bind(&ABV_Comms::stateCallback, this, std::placeholders::_1));
 
    abv_idl::msg::AbvCommand cmd;
    cmd.set__type("enable");

    while(!isModeCommandAckd())
    {
        static auto lastPrintTime = std::chrono::steady_clock::now();
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastPrintTime);

        if (elapsedTime.count() >= std::chrono::seconds(1).count())
        {
            printf("Sending ENABLE command: %s\n", cmd.type.c_str());
            lastPrintTime = currentTime;
        }

        mTopicManager->publishMessage<abv_idl::msg::AbvCommand>("abv_command", cmd);

        std::this_thread::sleep_for(std::chrono::milliseconds(25)); // Adjust the sleep duration as needed
    }

    printf("ABV_Controller Initialized!\n");

    return true;
}

void ABV_Comms::responseCallback(const abv_idl::msg::AbvResponse::SharedPtr aMsg)
{
    // Check if the response is an ack for the mode command
    if(aMsg->ack)
    {
        setModeCommandAckd(true);  
    }
}

void ABV_Comms::publishCommand(const std::string& aCommandType, const std::vector<float>& aCommand)
{
    // if the command size is not 3, then it is invalid
    if(3 != aCommand.size())
    {
        printf("Invalid command size. Expected 3, got %d\n", static_cast<int>(aCommand.size()));
        return;
    }

    mCommand.type = aCommandType; 

    // set the values in the data type to be published
    mCommand.data.x = aCommand[0];
    mCommand.data.y = aCommand[1];
    mCommand.data.yaw = aCommand[2];

    mTopicManager->publishMessage<abv_idl::msg::AbvCommand>("abv_command", mCommand);
}

void ABV_Comms::setModeCommandAckd(bool aAckd) 
{
    std::lock_guard<std::mutex> lock(mModeCommandAckdMutex);
    mModeCommandAckd = aAckd;
}

bool ABV_Comms::isModeCommandAckd() 
{
    std::lock_guard<std::mutex> lock(mModeCommandAckdMutex);
    return mModeCommandAckd;
}

void ABV_Comms::stateCallback(const abv_idl::msg::AbvState::SharedPtr aMsg)
{
    std::vector<double> state;
    
    state.push_back(aMsg->position.x);
    state.push_back(aMsg->position.y);
    state.push_back(aMsg->position.yaw);

    state.push_back(aMsg->velocity.x);
    state.push_back(aMsg->velocity.y);
    state.push_back(aMsg->velocity.yaw);

    setVehicleState(state);
}

void ABV_Comms::setVehicleState(const std::vector<double>& aState) 
{
    std::lock_guard<std::mutex> lock(mVehicleStateMutex);
    mVehicleState = aState;
}

// TO DO: add comments for user to understand what this function does
std::vector<double> ABV_Comms::getVehicleState() 
{
    std::lock_guard<std::mutex> lock(mVehicleStateMutex);
    return mVehicleState;
}

// TO DO: add comments for user to understand what this function does
std::vector<double> ABV_Comms::getVehiclePose()
{
    std::vector<double> pose;
    std::lock_guard<std::mutex> lock(mVehicleStateMutex);
    pose.push_back(mVehicleState[0]);
    pose.push_back(mVehicleState[1]);
    pose.push_back(mVehicleState[2]);
    return pose;
}

// TO DO: add comments for user to understand what this function does
std::vector<double> ABV_Comms::getVehicleVelocity()
{
    std::vector<double> velocity;
    std::lock_guard<std::mutex> lock(mVehicleStateMutex);
    velocity.push_back(mVehicleState[3]);
    velocity.push_back(mVehicleState[4]);
    velocity.push_back(mVehicleState[5]);
    return velocity;
}


