
#include "RosTopicManager.h"
#include <mutex>

class ABV_Comms
{
public:
    ABV_Comms(/* args */);
    ~ABV_Comms();

    static bool init();
    void spinNode();
    static bool shutdown();

    void publishCommand(const std::vector<double>& aCommand);

    enum class CONTROL_MODE {
        THRUSTER,
        POSE
    };

    /**
     * @brief Initialize the controller with the specified mode. 
     * @param aMode the mode to initialize the controller with. 
     * @return true if the controller was successfully initialized, false otherwise. 
     */
    bool initializeController(ABV_Comms::CONTROL_MODE aMode);
    
    /**
     * @brief Get the current state of the vehicle. 
     * @return a vector containing the vehicle state. The state is in the following order: [x, y, yaw, x_dot, y_dot, yaw_dot]
     */
    std::vector<double> getVehicleState();

    /** 
     * @brief Get the current pose of the vehicle. 
     * @return a vector containing the vehicle pose. The pose is in the following order: [x, y, yaw]
     */
    std::vector<double> getVehiclePose();

    /**
     * @brief Get the current velocity of the vehicle.
     * @return a vector containing the vehicle velocity. The velocity is in the following order: [x_dot, y_dot, yaw_dot]
     */
    std::vector<double> getVehicleVelocity();

private:

    void responseCallback(const abv_idl::msg::AbvResponse::SharedPtr aMsg);

    void setModeCommandAckd(bool aAckd);
    bool isModeCommandAckd();

    void stateCallback(const abv_idl::msg::AbvState::SharedPtr aMsg);
    void setVehicleState(const std::vector<double>& aState);

    std::shared_ptr<RosTopicManager> mTopicManager;
    CONTROL_MODE mMode;
    bool mModeCommandAckd; 
    std::mutex mModeCommandAckdMutex;

    std::vector<double> mVehicleState; 
    std::mutex mVehicleStateMutex;

    abv_idl::msg::AbvCommand mCommand;

};
