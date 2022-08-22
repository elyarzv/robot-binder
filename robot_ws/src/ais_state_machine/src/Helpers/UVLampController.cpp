#include "ais_state_machine/Helpers/UVLampController.h"

using namespace ais_state_machine;

UVLampController::UVLampController(ros::NodeHandle& nh) : nh_(nh) 
{
    uv_lamp_switch_pub_ = nh.advertise<std_msgs::UInt8>("embedded/power_inverter/switch", 1000);
    set_uv_intensity_client_ = nh.serviceClient<phoenix_msgs::SetLampStatus>("set_dimmer_values");
    uv_intensity_data_sub_ = nh.subscribe("uv_estimated_irradiance", 1, &UVLampController::getUVIrradianceCb, this);
    uv_status_sub_ = nh.subscribe("embedded/power_inverter/status", 1, &UVLampController::UVStatusCb, this);

    if (!nh_.hasParam("max_lamp_number")) {
        ROS_WARN("UV LampController: Missing %s/max_lamp_number parameter!", 
            nh_.getUnresolvedNamespace().c_str());
    }
    nh.param<int>("max_lamp_number", lamp_number, 6);

    for(int ctr = 0; ctr < lamp_number; ++ctr){
        lamp_vector_.data.push_back(0);
    }

    uv_irradiance_data_.irradiance = 0;
    uv_irradiance_data_.max_stable_irradiance = 99999;
    uv_irradiance_data_.mode_duration = ros::Duration(0.0);
    uv_irradiance_data_.peak_irradiance = 999999;
    uv_irradiance_data_.percentage = 0.0;
    uv_status = false;
    set_uv_intensity_client_.waitForExistence();
}

UVLampController::~UVLampController()
{

}

void UVLampController::setUVLampOn(bool flag)
{
    std_msgs::UInt8 uv_bulb;
    uv_bulb.data = flag ? 1 : 0;
    if(uv_status == flag){
        ROS_DEBUG_THROTTLE(1,"Repeated command sent to lamp controller");
    }else{
        uv_lamp_switch_pub_.publish(uv_bulb);
        if(!uv_bulb.data){
            setUVIntensity(0);
        }
    }
    ros::spinOnce();
}
bool UVLampController::isUVOn()
{
    return uv_status;
}

bool UVLampController::setUVIntensity(std::vector<float> vals){
    lamp_vector_.data.clear();
    int ctr = 0;
    ROS_DEBUG_THROTTLE(1,"Setting UV Intensity");
    for(float val : vals){
        val = std::max((float)0, val);
        val = std::min((float)100,val);
        lamp_vector_.data.push_back(val);
        //ROS_INFO("Lamp %d: %f", ++ctr,val);
    }
    phoenix_msgs::SetLampStatus req;
    req.request.lamp_values = lamp_vector_.data;

    if (set_uv_intensity_client_.exists() && set_uv_intensity_client_.call(req))
    {
        ROS_DEBUG("Successfully set UV lamp intensity");
        return true;
    }
    else
    {   
        ROS_DEBUG("Unable to set UV lamp intensity");
        return false;
    }


    return false;
}
bool UVLampController::setUVIntensity(int val){
    val = std::max(0, val);
    val = std::min(100,val);

    lamp_vector_.data.clear();
    for(int ctr = 0; ctr < lamp_number; ++ctr){
        lamp_vector_.data.push_back(val);
        ROS_INFO("Avg Lamp %d: %d", ctr, val);
    }
    phoenix_msgs::SetLampStatus req;
    req.request.lamp_values = lamp_vector_.data;

    if (set_uv_intensity_client_.exists() && set_uv_intensity_client_.call(req))
    {
        ROS_INFO("Successfully set UV lamp intensity to %d", val);
        return true;
    }
    else
    {   
        ROS_WARN("Unable to set UV lamp intensity");
        return false;
    }


    return false;
}

void UVLampController::getUVIrradianceCb(const phoenix_msgs::DimmingIrradianceData::ConstPtr& msg)
{
    uv_irradiance_data_ = *msg;
}

double UVLampController::getCurrentIrradiance()
{
    return uv_irradiance_data_.irradiance;
}

double UVLampController::getCurrentIrradiancePercent()
{
    return uv_irradiance_data_.percentage * 100;
}

void UVLampController::UVStatusCb(const std_msgs::Bool::ConstPtr& msg)
{
    uv_status = msg->data;
}