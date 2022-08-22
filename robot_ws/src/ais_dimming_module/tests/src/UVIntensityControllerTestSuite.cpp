#include "ais_dimming_module/UVIntensityControllerTestSuite.h"

UVIntensityControllerTest::UVIntensityControllerTest()
{
    set_uv_intensity_client_ = nh.serviceClient<phoenix_msgs::SetLampStatus>("set_dimmer_values");
    uv_dimmer_values_sub_ = nh.subscribe("/embedded/uv_lamp_dimmer_values", 1, 
                                            &UVIntensityControllerTest::lampValueCb, this);
    nh.param<int>("max_lamp_number", lamp_number, 6);

    for(int ctr = 0; ctr < lamp_number; ++ctr){
        sub_lamp_vector_.data.push_back(0);
        lamp_vector_.data.push_back(0);
    }
}

UVIntensityControllerTest::~UVIntensityControllerTest()
{

}

bool UVIntensityControllerTest::setUVIntensity(int val){
    val = std::max(0, val);
    val = std::min(100,val);

    for(int ctr = 0; ctr < lamp_vector_.data.size(); ++ctr){
        lamp_vector_.data[ctr] = val;
    }
    phoenix_msgs::SetLampStatus req;
    req.request.lamp_values = lamp_vector_.data;
    if (set_uv_intensity_client_.exists() && set_uv_intensity_client_.call(req))
    {
        ROS_INFO("Successfully set UV lamp intensity to %d", val);
        ros::Duration(1.0).sleep();
        return true;
    }
    else
    {   
        ROS_WARN("Unable to set UV lamp intensity");
        ros::Duration(1.0).sleep();
        return false;
    }
    ros::Duration(1.0).sleep();
    return false;
}

void UVIntensityControllerTest::lampValueCb(const std_msgs::ByteMultiArray::ConstPtr& msg)
{
    sub_lamp_vector_.layout = msg->layout;
    sub_lamp_vector_.data = msg->data;
}
std_msgs::ByteMultiArray UVIntensityControllerTest::getSubLampVector()
{
    return sub_lamp_vector_;
}
void UVIntensityControllerTest::SetUp()
{
    waitForInit();
    ros::Duration(1.0).sleep();


}

void UVIntensityControllerTest::waitForInit()
{
    while (!isNodeAlive() && ros::ok())
    {
        ros::Duration(0.1).sleep();
    }
    if (!ros::ok())
    {
        FAIL() << "ROS core not up while executing test";
    }
}
bool UVIntensityControllerTest::isNodeAlive()
{
    return (uv_dimmer_values_sub_.getNumPublishers() > 0);
}
    //void UVIntensityControllerTest::TearDown(){}