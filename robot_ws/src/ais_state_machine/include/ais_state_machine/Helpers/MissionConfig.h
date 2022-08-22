/*
    AIS CONFIDENTIAL
    Author: Michael Chiou (m.chiou@ai-systems.ca)
*/

#ifndef ASM_MISSION_CONFIG_STRUCT_H_
#define ASM_MISSION_CONFIG_STRUCT_H_

#include <string>
#include <array>

using std::string;

namespace ais_state_machine
{

/* @brief This struct contains all mission configuration data that is used in a typical disinfection
 *        mission
 */

struct MissionConfig {

    bool use_uv_lamp;
    bool use_led_as_uv;
    bool use_stop_as_uv;
    bool need_return_init_rail_pos;
    bool is_forward_robot_rail_direction;
    bool return_to_rail_home_with_uv_on;
    bool irradiance_speed_control;
    float return_to_rail_home_speed;
    bool rail_is_left_adjacent;

    float uv_lamp_warmup_time;
    float max_rail_length;
    float max_tolerable_rail_length;
    float wheel_odom_error_rate;
    float robot_goal_stop_time;
    float max_rail_speed;
    float low_battery;
    float critical_battery;
    float estimated_battery_usage;
    float minimum_battery;
    
    // new mission params
    int start_rail_number;
    std::vector< std::array<int, 2> > rail_jobs;
    int num_rails;

    string mission_name;
    string mission_directory;
    string mission_path;

    std::vector<float> return_home_intensity;
    std::vector<float> rail_transition_intensity;

};

} // end ais_state_machine namespace

#endif // ASM_MISSION_CONFIG_STRUCT_H_
