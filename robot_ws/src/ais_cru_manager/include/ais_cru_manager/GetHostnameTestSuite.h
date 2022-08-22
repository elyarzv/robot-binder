#ifndef GET_HOSTNAME_TEST_SUITE_H_
#define GET_HOSTNAME_TEST_SUITE_H_

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <string>
#include <iostream>

using std::string;

class GetHostnameTest : public ::testing::Test
{
  public:
    /**
     * @brief Construct a new GetHostnameTest object
     *
     */
    GetHostnameTest();
    /**
     * @brief Destroy the GetHostnameTest object
     *
     */
    ~GetHostnameTest();
    /**
     * @brief Get the Hostname from device
     *
     * @return true if hostname retrieval was successful
     * @return false if hostname retreival failed
     */
    bool getHostname(string&);

  private:
    ros::NodeHandle nh;
    ros::ServiceClient get_hostname_client_;
    std_srvs::Trigger trigger_;
};
#endif  // END GET_HOSTNAME_TEST_SUITE_H_