/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ADM_DIM_MODULE_CONTROLLER_H_
#define ADM_DIM_MODULE_CONTROLLER_H_

#include <math.h>
#include <string>
#include "ais_dimming_module/AbstractDimModuleController.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/UInt16.h"
#include "std_srvs/Trigger.h"
#include <phoenix_msgs/SetLampStatus.h>
#include "phoenix_msgs/DimmingIrradianceData.h"
#include "ais_dimming_module/IrradianceModels/QuantitativeIrradianceModel.h"

namespace ais_dimming_module
{
class DimModuleController : public AbstractDimModuleController
{
  public:
    /*
     * Constructor
     * @param nh: takes in global ROS NodeHandle
     * @param update_freq: checks refresh date
     */
    explicit DimModuleController(ros::NodeHandle& nh, int update_freq);
    /*
     * Destructor
     */
    ~DimModuleController() override;
    /*
     * execution loop for processing service requests and incoming lamp feedback
     */
    void run() override;

  private:
    /*
     * Service call to set desired lamp values from 0 to 100 (0% full off, 100% full power)
     */
    bool setDimmerValuesService(phoenix_msgs::SetLampStatus::Request&,
                                phoenix_msgs::SetLampStatus::Response&);
    /*
     * Subscriber callback to retrieve raw Dimmer_Value feedback
     */
    void rawLampStatusCb(const std_msgs::UInt16::ConstPtr&);

    /*
     * Helper function to publish ROS data
     */
    void publishData();
    /*
     * Handles and Sanitizes raw data received from HW side for UV lamp feedback
     */
    std_msgs::UInt16 processRawFeedback(std_msgs::UInt16 raw_data);
    /*
     * Initialize and sanitize ROS messages
     */
    void initROSMessage();

    /*
     *   Reset UV timers and calculate UV intensity estimation
     */
    void updateUVIntensityCalculation();

    /*
    *   Returns if processed_data is non-zero or not (indicates if UV lamps are POWERED, not
    necessarily on)
    */
    bool isUVLampOn();

  private:
    ros::NodeHandle nh_;
    ros::Publisher processed_lamp_status_pub_;
    ros::Publisher dimmer_cmd_pub_;
    
    ros::Publisher uv_irradiance_pub_;

    ros::Subscriber raw_lamp_status_sub_;
    ros::ServiceServer set_dimmer_service_;

    std_msgs::UInt16 raw_dimmer_values_;
    std_msgs::UInt16 processed_dimmer_values_;
    std_msgs::ByteMultiArray desired_lamp_status_;

    int MAX_LAMPS_COUNT_;
    int MIN_RAW_DIM_VALUE_;

    bool is_uv_powered_;
    int update_freq_;
    ros::Duration uv_state_timer_;
    ros::Duration last_uv_state_timer_;

    QuantitativeIrradianceModel uv_model_;
    double curr_uv_irradiance_percent_;
    double curr_uv_irradiance_;
    double last_uv_irradiance_;
    double desired_irradiance_;
    double UV_DECAY_TIME_TO_ZERO_;
};

};  // namespace ais_dimming_module

#endif