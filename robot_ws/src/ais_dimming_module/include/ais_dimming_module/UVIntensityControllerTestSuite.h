
#ifndef UV_INTENSITY_CONTROLLER_TEST_SUITE_H_
#define UV_INTENSITY_CONTROLLER_TEST_SUITE_H_

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "std_msgs/ByteMultiArray.h"
#include "phoenix_msgs/SetLampStatus.h"


class UVIntensityControllerTest : public ::testing::Test 
{
public:
    /**
     * @brief Construct a new UVIntensityControllerTest object
     * 
     */
    UVIntensityControllerTest();
    /**
     * @brief Destroy the UVIntensityControllerTest object
     * 
     */
    ~UVIntensityControllerTest();
    /**
     * @brief  Set UV Intensity via service call to dimming ROS node
     * 
     * @return true 
     * @return false 
     */
    bool setUVIntensity(int);
    /**
     * @brief Get the Sub Lamp Vector object
     * 
     * @return std_msgs::ByteMultiArray 
     */
    std_msgs::ByteMultiArray getSubLampVector();

protected:
    /**
     * @brief SetUp GoogleTest virtual function
     * 
     */
    void SetUp();
    //void TearDown();
private:
    /**
     * @brief Initialize test suite
     * 
     */
    void waitForInit();
    /**
     * @brief Check if Require Test Suite node is alive
     * 
     * @return true 
     * @return false 
     */
    bool isNodeAlive();
    /**
     * @brief callback for subscribing to lamp values received from dimming ROS node
     * 
     * @param msg 
     */
    void lampValueCb(const std_msgs::ByteMultiArray::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    ros::Subscriber uv_dimmer_values_sub_;
    ros::ServiceClient set_uv_intensity_client_;    

    std_msgs::ByteMultiArray lamp_vector_;
    std_msgs::ByteMultiArray sub_lamp_vector_;

    int lamp_number;
};
#endif // END UV_INTENSITY_CONTROLLER_TEST_SUITE_H_