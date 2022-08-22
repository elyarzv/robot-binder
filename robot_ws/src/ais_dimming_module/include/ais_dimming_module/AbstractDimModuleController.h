/* 
    AIS CONFIDENTIAL
    author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ADM_ABSTRACT_DIM_MODULE_CONTROLLER_H_
#define ADM_ABSTRACT_DIM_MODULE_CONTROLLER_H_

#include "ros/ros.h"

namespace ais_dimming_module
{
class AbstractDimModuleController
{
public:
    /*
     * Constructor
     */
    AbstractDimModuleController(ros::NodeHandle& nh);
    /*
     * Virtual destructor
     */
    virtual ~AbstractDimModuleController() = default;
    /*
    * Execution Loop for Dim Module Controller. Run this function in a loop to process data
    */
    virtual void run() = 0;
private:
    
    ros::NodeHandle nh_;
};

}; // End namespace ais_dimming_module


#endif