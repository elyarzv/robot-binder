/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/


#ifndef ASM_SAFETY_MUX_SELECTOR_H_
#define ASM_SAFETY_MUX_SELECTOR_H_

#define SOURCE_TIMEOUT 3

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "topic_tools/MuxSelect.h"
#include "dynamic_reconfigure/server.h"
#include "dynamic_reconfigure/client.h"

#include <atomic>
#include <memory>
#include <optional>

#include "ais_state_machine/Enums/phoenix_state_enums.h"
#include "ais_state_machine/DynamicParamsConfig.h"

#include "ais_safety/DynamicParamsConfig.h"

const std::string SAFE_CMD_VEL_TOPIC = "cmd_vel_safe";
const std::string STATE_MACHINE_CMD_VEL_TOPIC = "cmd_vel_state_machine";
namespace ais_state_machine
{

class SafetyMuxSelector
{
public:
    SafetyMuxSelector();

    void currentStateCallback(const std_msgs::Int32::ConstPtr &state_msg);

    typedef ais_state_machine::DynamicParamsConfig dyn_cfg_t;
    void dynamicReconfigureCallback(dyn_cfg_t &config, uint32_t level);
    bool setForwardSafety(bool enabled);

    bool callMuxSelector(const std::string mux_topic);
    void selectMux(StateTypes state);
    bool isSourceAlive();
protected:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::ServiceClient mux_selector_client_;
    ros::Time last_source_message_;

    std::optional<std::string> last_mux_topic_;
    std::atomic<double> last_mux_topic_publish_stamp_;

    std::shared_ptr< dynamic_reconfigure::Server<dyn_cfg_t> >
        dynamic_reconfigure_server_;

    std::atomic<bool> manual_safety_;

    std::optional<bool> last_forward_safety_;
    std::atomic<double> last_safety_cfg_publish_stamp_;

    std::shared_ptr< dynamic_reconfigure::Client<ais_safety::DynamicParamsConfig> >
        dynamic_reconfigure_client_;

}; // endclass SafetyMuxSelector

} // endnamespace ais_state_machine
#endif // ASM_SAFETY_MUX_SELECTOR_H_
