#ifndef AIS_ROBOT_CRU_MANAGER_H_
#define AIS_ROBOT_CRU_MANAGER_H_

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <unistd.h>
#include <limits.h>

namespace ais_cru_manager
{
class RobotCRUManager
{
  public:
  explicit RobotCRUManager(ros::NodeHandle&);
  ~RobotCRUManager() = default;

  protected:
  /*
   * @brief gets hostname of host computer. Intended for use on production level images to
   * retrieve CRU information/name
   */

  bool getHostnameCb(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

  private:
  ros::NodeHandle nh_;
  ros::ServiceServer get_hostname_service_;
};
};  // namespace ais_cru_manager

#endif  // AIS_ROBOT_CRU_MANAGER_H_