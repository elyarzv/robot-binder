/*
    AIS CONFIDENTIAL
    Author: Rakesh Shrestha (r.shrestha@ai-systems.ca)
*/

#include <string>
#include <set>

#include "ais_state_machine/SafetyMuxSelector/safety_mux_selector.h"

using namespace ais_state_machine;


const float SAFETY_CFG_PUBLISH_INTERVAL = 0.5;
const float MUX_TOPIC_PUBLISH_INTERVAL = 0.5;

SafetyMuxSelector::SafetyMuxSelector()
    : manual_safety_(false),
      last_forward_safety_(std::nullopt),
      last_mux_topic_(std::nullopt),
      last_mux_topic_publish_stamp_(0),
      last_safety_cfg_publish_stamp_(0)
{
    state_sub_ = nh_.subscribe(
        "/current_state", 1, &SafetyMuxSelector::currentStateCallback, this
    );

    mux_selector_client_ = nh_.serviceClient<topic_tools::MuxSelect>(
        "safety_cmd_vel_mux/select"
    );

    dynamic_reconfigure_server_
        = std::make_shared< dynamic_reconfigure::Server<dyn_cfg_t> >();

    dynamic_reconfigure::Server<dyn_cfg_t>::CallbackType dyn_config_cb
        = boost::bind(&SafetyMuxSelector::dynamicReconfigureCallback, this, _1, _2);

    dynamic_reconfigure_server_->setCallback(dyn_config_cb);

    typedef dynamic_reconfigure::Client<ais_safety::DynamicParamsConfig> dyn_cfg_client_t;
    dynamic_reconfigure_client_ = std::make_shared<dyn_cfg_client_t>("/safety_node");
}
bool SafetyMuxSelector::isSourceAlive(){
    if(ros::Time::now().toSec() - last_source_message_.toSec() < SOURCE_TIMEOUT) return true;
    else return false;
}
void SafetyMuxSelector::currentStateCallback(
    const std_msgs::Int32::ConstPtr &state_msg
)
{
    last_source_message_ = ros::Time::now();
    selectMux((StateTypes)state_msg->data);
}

void SafetyMuxSelector::dynamicReconfigureCallback(
    SafetyMuxSelector::dyn_cfg_t &config, uint32_t level
)
{
    manual_safety_ = config.MANUAL_SAFETY;
    ROS_INFO("Manual Safety: %d", (int)manual_safety_);
}

bool SafetyMuxSelector::setForwardSafety(bool enabled)
{
    ais_safety::DynamicParamsConfig cfg;

    // Don't read the configuration, this can get you stuck
    // if (!dynamic_reconfigure_client_->getCurrentConfiguration(cfg, ros::Duration(1.0)))
    // {
    //     ROS_WARN("Dynamic param get failed");
    //     return false;
    // }

    if (last_forward_safety_ && (*last_forward_safety_ == enabled))
    {
        auto elapsed = ros::Time::now().toSec() - (double)last_safety_cfg_publish_stamp_;
        if (elapsed < SAFETY_CFG_PUBLISH_INTERVAL)
        {
            // same cfg published not long ago, avoid spamming the server
            return true;
        }
    }

    cfg.FORWARD_SAFETY = enabled;
    cfg.BACKWARD_SAFETY = true;

    if (!dynamic_reconfigure_client_->setConfiguration(cfg))
    {
        ROS_WARN("Dynamic param set failed");
        return false;
    }

    last_forward_safety_ = enabled;
    last_safety_cfg_publish_stamp_ = ros::Time::now().toSec();

    return true;
}

bool SafetyMuxSelector::callMuxSelector(const std::string mux_topic)
{
    if (last_mux_topic_ && (*last_mux_topic_ == mux_topic))
    {
        auto elapsed = ros::Time::now().toSec() - (double)last_mux_topic_publish_stamp_;
        if (elapsed < MUX_TOPIC_PUBLISH_INTERVAL)
        {
            // same mux topic published not long ago, avoid spamming the server
            return true;
        }
    }

    topic_tools::MuxSelect srv;
    srv.request.topic = mux_topic;

    if (!mux_selector_client_.call(srv))
    {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Unable to selected mux to " << mux_topic);
        return false;
    }

    ROS_INFO_STREAM("Cmd vel topic selected: " << mux_topic);
    last_mux_topic_ = mux_topic;
    last_mux_topic_publish_stamp_ = ros::Time::now().toSec();

    return true;
}

void SafetyMuxSelector::selectMux(StateTypes state)
{
    const std::set<StateTypes> safety_states{
        StateTypes::MOVE_TO_NEXT_RAIL,
        StateTypes::ADJUST_TO_RAIL,
        // TODO: in these states the robot might still be on the rail, fix them
        // StateTypes::MOVE_ROBOT_OFF_RAIL,
        // StateTypes::MOUNT_ROBOT_TO_RAIL
    };

    std::string mux_topic;

    if (std::count(safety_states.begin(), safety_states.end(), state))
    {
        if (!setForwardSafety(true)) { return; }
        // robot off the rail, safety ON
        mux_topic = SAFE_CMD_VEL_TOPIC;
    }
    else if (state == StateTypes::MANUAL_CONTROL && manual_safety_)
    {
        if (!setForwardSafety(true)) { return; }
        mux_topic = SAFE_CMD_VEL_TOPIC;
    }
    else if (state == StateTypes::MOVE_ROBOT_OFF_RAIL)
    {
        if (!setForwardSafety(false)) { return; }
        mux_topic = SAFE_CMD_VEL_TOPIC;
    }
    else
    {
        setForwardSafety(true);
        // robot on the rail, safety OFF
        mux_topic = STATE_MACHINE_CMD_VEL_TOPIC;
    }

    callMuxSelector(mux_topic);
}
