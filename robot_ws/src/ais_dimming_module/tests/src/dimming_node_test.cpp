#include "ais_dimming_module/UVIntensityControllerTestSuite.h"



TEST_F(UVIntensityControllerTest, ChangeUVIntensity)
{   

    std_msgs::ByteMultiArray byte_array;
    byte_array.data.push_back(0);

    std_msgs::ByteMultiArray tmp = getSubLampVector();
    if (tmp.data.empty())
    {
        ROS_ERROR("ChangeUVIntensity Test code is broken due to internal changes?!");
    }
    ASSERT_FALSE(tmp.data.empty());
    
    EXPECT_EQ(getSubLampVector().data[0], byte_array.data[0]);
    setUVIntensity(50);
    ros::spinOnce();
    byte_array.data[0]= 50;
    ROS_ERROR("%d, %d", getSubLampVector().data[0], byte_array.data[0]);
    ASSERT_EQ(getSubLampVector().data[0], byte_array.data[0]);

    //EXPECT_EQ(getSubLampVector().data[0], byte_array.data[0]);
    
}

int main(int argc, char** argv) 
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "TestDimmingModuleIntensity");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::cout << "RUNNING TESTS" << std::endl;
    int ret = RUN_ALL_TESTS();
    spinner.stop();
    ros::shutdown();
    return ret;
}