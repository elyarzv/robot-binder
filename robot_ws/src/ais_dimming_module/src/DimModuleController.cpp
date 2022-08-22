#include "ais_dimming_module/DimModuleController.h"

using namespace ais_dimming_module;

DimModuleController::DimModuleController(ros::NodeHandle& nh, int update_freq)
    : AbstractDimModuleController(nh)
{
    ROS_INFO("Initiating DimModuleController");
    if (update_freq <= 0){
        update_freq_ = 10;
        ROS_WARN("DimModuleController Invalid frequency rate set! Using default value of 10");
    } else {
        update_freq_ = update_freq;
    }

    // Initialize ROS Subscriber Publishers and ServiceServers
    processed_lamp_status_pub_ = nh.advertise<std_msgs::UInt16>("processed_lamp_status", 1000);
    uv_irradiance_pub_ = nh.advertise<phoenix_msgs::DimmingIrradianceData>("uv_estimated_irradiance", 1000);
    dimmer_cmd_pub_ =
        nh.advertise<std_msgs::ByteMultiArray>("embedded/uv_lamp_dimmer_values", 1000);
    raw_lamp_status_sub_ =
        nh.subscribe("embedded/uv_lamp_status", 1, &DimModuleController::rawLampStatusCb, this);

    set_dimmer_service_ = nh.advertiseService("set_dimmer_values",
                                              &DimModuleController::setDimmerValuesService, this);
    // Determine number of robot lamps from YAML file (should be loaded via launch)
    MAX_LAMPS_COUNT_ = -1;
    if (nh.hasParam("max_lamp_number"))
    {
        nh_.getParam("max_lamp_number", MAX_LAMPS_COUNT_);
        initROSMessage();
    }

    uv_model_ = QuantitativeIrradianceModel();
    is_uv_powered_ = false;
    uv_state_timer_ = ros::Duration(0.0);

    UV_DECAY_TIME_TO_ZERO_ = 30.0;

    curr_uv_irradiance_ = 0.0;
    curr_uv_irradiance_percent_ = 0.0;
    last_uv_irradiance_ = 0.0;
    desired_irradiance_ = 0.0;
}

DimModuleController::~DimModuleController()
{
    ROS_INFO("Shutting down DimModuleController");
}

void DimModuleController::initROSMessage()
{
    // Sanitize inputs
    if (MAX_LAMPS_COUNT_ > 16)
    {
        ROS_ERROR("Code is not meant to handle more than 16bits of data!!");
        MAX_LAMPS_COUNT_ = -1;
        return;
    }
    // Initialize desired dimensions for ByteMultiArray structure
    desired_lamp_status_.layout.dim.push_back(std_msgs::MultiArrayDimension());
    desired_lamp_status_.layout.dim[0].size = MAX_LAMPS_COUNT_;
    desired_lamp_status_.layout.dim[0].stride = MAX_LAMPS_COUNT_;
    desired_lamp_status_.layout.dim[0].label = "lamp_size";

    // Prepopulate initial desired zero with 0s
    for (int i = 0; i < MAX_LAMPS_COUNT_; i++)
    {
        desired_lamp_status_.data.push_back(0);
    }
    

    raw_dimmer_values_.data = 0;
    MIN_RAW_DIM_VALUE_ = (int)pow(2.0, MAX_LAMPS_COUNT_);
    ROS_INFO("Robot configured with %d lamps!", MAX_LAMPS_COUNT_);
}

void DimModuleController::run()
{
    // Do not attempt to process data if MAX_LAMPS_COUNT_ is not set, means unknown robot config!
    if (MAX_LAMPS_COUNT_ == -1)
    {
        // TODO(Michael Chiou) Namespace this global parameter with localized namespace variable
        ROS_WARN("rosparam max_lamp_number not set! Waiting for config file to load");
        if (nh_.hasParam("max_lamp_number"))
        {
            nh_.getParam("max_lamp_number", MAX_LAMPS_COUNT_);
            initROSMessage();
        }
    }
    else  // Process and publish ros data (Start actual interface functionality)
    {
        ROS_INFO_ONCE("Publishing Data");
        // Feedback format has changed, further processing not needed at this time
        // processed_dimmer_values_ = processRawFeedback(raw_dimmer_values_);
        processed_dimmer_values_ = raw_dimmer_values_;
        updateUVIntensityCalculation();
        publishData();
        ros::spinOnce();
    }
}

bool DimModuleController::setDimmerValuesService(phoenix_msgs::SetLampStatus::Request& req,
                                                 phoenix_msgs::SetLampStatus::Response& res)
{

    ROS_DEBUG("DimmerValueService received!, %ld", req.lamp_values.size());
    // Check inputs for errors, return False and do not accept if invalid values are received
    // Check if total number of lamp values are correct!
    if (req.lamp_values.empty())
    {
        res.success = false;
        std::string msg = "Empty lamp values!";
        res.message = msg;
        return true;
    }
    else if (req.lamp_values.size() != MAX_LAMPS_COUNT_)
    {
        res.success = false;
        std::string msg = "Invalid Number of Lamp values received detected! Must be of size " +
                          std::to_string(MAX_LAMPS_COUNT_);
        res.message = msg;
        return true;
    }
    // Check if individual number of lamp values are within percentage range
    // Calculate simple mean irradiance value if valid
    double tmp = 0;
    for (auto& val : req.lamp_values)
    {
        if (val > 100 || val < 0)
        {
            res.success = false;
            res.message = "Invalid Lamp value detected! Must be with [0-100]";
            return true;
        }
        tmp += val;
    }
    // Calculate average desired radiance
    desired_irradiance_ = tmp / req.lamp_values.size();
    desired_irradiance_ = uv_model_.getMaxStableIrradiance() * desired_irradiance_/100.0f;
    desired_irradiance_ = std::max(desired_irradiance_, 30.0);
    // Set lamp values
    desired_lamp_status_.data = req.lamp_values;
    res.success = true;
    res.message = "New lamp intensity values accepted";
    return true;
}

void DimModuleController::rawLampStatusCb(const std_msgs::UInt16::ConstPtr& msg)
{
    ROS_DEBUG_THROTTLE(2, "Lamp_Status Topic Received! %d", msg->data);
    raw_dimmer_values_.data = msg->data;
    processed_lamp_status_pub_.publish(raw_dimmer_values_);
}

void DimModuleController::publishData()
{
    dimmer_cmd_pub_.publish(desired_lamp_status_);
    
    phoenix_msgs::DimmingIrradianceData uv_data;
    uv_data.mode_duration = uv_state_timer_;
    uv_data.percentage = curr_uv_irradiance_percent_;
    uv_data.irradiance = curr_uv_irradiance_;
    uv_data.max_stable_irradiance = uv_model_.getMaxStableIrradiance();
    uv_data.peak_irradiance = uv_model_.getMaxIrradiance();

    uv_irradiance_pub_.publish(uv_data);
}

std_msgs::UInt16 DimModuleController::processRawFeedback(std_msgs::UInt16 raw_data)
{
    std_msgs::UInt16 processed_data;
    /* // Disabled since raw processing is not needed for now
    // Detect if bit shifting right N bits MAY remove useful information (1 bits)
    if (raw_data.data <= MIN_RAW_DIM_VALUE_ && raw_data.data > 0) {
        ROS_WARN_THROTTLE(2,"Possible data corruption or incorrect lamp size configuration!");
    }*/
    // Right shift all trailing 0 bits
    processed_data.data = raw_data.data >> (sizeof(raw_data.data) * 8 - MAX_LAMPS_COUNT_);
    return processed_data;
}

void DimModuleController::updateUVIntensityCalculation()
{   
    ROS_INFO("duration: %lf", uv_state_timer_.toSec());
    if (isUVLampOn() && !is_uv_powered_)
    {
        is_uv_powered_ = true;
        if (uv_state_timer_.toSec() > UV_DECAY_TIME_TO_ZERO_)
        {
            curr_uv_irradiance_ = 0.0;
            uv_state_timer_ = ros::Duration(0.0);
        } else 
        {
            curr_uv_irradiance_ = last_uv_irradiance_;
            uv_state_timer_ = last_uv_state_timer_;
        }
        ROS_DEBUG("Lamp transitioning to ON state");
    }
    else if (!isUVLampOn() && is_uv_powered_)
    {
        is_uv_powered_ = false;
        last_uv_state_timer_ = uv_state_timer_;
        uv_state_timer_ = ros::Duration(0.0);

        last_uv_irradiance_ = curr_uv_irradiance_;
        curr_uv_irradiance_ = 0.0;

        ROS_DEBUG("Lamp transitioning to OFF state");
    }
    else // if (isUVLampOn() == is_uv_powered_)
    {
        if (isUVLampOn())
        {
            curr_uv_irradiance_ = uv_model_.calculateIrradiance( uv_state_timer_.toSec()); // y= mx+b
        } else {
            curr_uv_irradiance_ = 0;
        }
        // Implicit assumption that ros nodes will always run at desired frequency
        uv_state_timer_ += ros::Duration(1.0/update_freq_);
        ROS_DEBUG("No new lamp transition");
    }
    if (curr_uv_irradiance_ > desired_irradiance_)
    {
        uv_state_timer_ = ros::Duration(uv_model_.calculateTimeToIrradiance(desired_irradiance_));
        curr_uv_irradiance_ = desired_irradiance_;
    }
    curr_uv_irradiance_percent_ = curr_uv_irradiance_ / uv_model_.getMaxStableIrradiance();
}

bool DimModuleController::isUVLampOn()
{
    return (processed_dimmer_values_.data != 0);
}
