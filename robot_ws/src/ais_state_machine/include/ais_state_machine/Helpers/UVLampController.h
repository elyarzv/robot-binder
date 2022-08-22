/* 
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_UV_LAMP_CONTROLLER_H_
#define ASM_UV_LAMP_CONTROLLER_H_

#include <algorithm>

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/ByteMultiArray.h"
#include "phoenix_msgs/SetLampStatus.h"
#include "phoenix_msgs/DimmingIrradianceData.h"

namespace ais_state_machine
{
/**
 * @brief UVLampController retains and controls functionality with regards to UV Lamps
 *        Implementation details can include seting UV Lamps On/Off or UV Lamp Intensity
 */
class UVLampController
{
    public:
        /*
        *   @brief Constructor
        */
        UVLampController(ros::NodeHandle&);
        /*
        *   @brief Destructor
        */
        ~UVLampController();

        /*
        * @brief takes in boolean value, if value is true, turn lamps on, else turn UV lamp off 
        */
        void setUVLampOn(bool);
        /*
        * @brief setUVIntensity calls ros service that sets dimming values sent to embedded
        *       boolean returns are for if successful (true) or not (false)
        */
        bool setUVIntensity(int intensity);
        /*
        * @brief setUVIntensity calls ros service that sets dimming values for each lamp sent to embedded
        *       boolean returns are for if successful (true) or not (false)
        */
        bool setUVIntensity(std::vector<float> intensity);

        /*
        * @brief callback for UV Irradiance data which should be published as feedback
        *
        */
        void getUVIrradianceCb(const phoenix_msgs::DimmingIrradianceData::ConstPtr&);
        void UVStatusCb(const std_msgs::Bool::ConstPtr&);

        double getCurrentIrradiance();
        double getCurrentIrradiancePercent();
        bool isUVOn();

    private:
        ros::NodeHandle nh_;
        ros::Publisher uv_lamp_switch_pub_;
        ros::Subscriber uv_intensity_data_sub_;
        ros::Subscriber uv_status_sub_;
        ros::ServiceClient set_uv_intensity_client_;
        ros::Time msg_recv;
        bool uv_status;
        std_msgs::ByteMultiArray lamp_vector_;
        
        int lamp_number;
        phoenix_msgs::DimmingIrradianceData uv_irradiance_data_;

};
} // end of namespace ais_state_machine

#endif // END ASM_UV_LAMP_CONTROLLER_H_