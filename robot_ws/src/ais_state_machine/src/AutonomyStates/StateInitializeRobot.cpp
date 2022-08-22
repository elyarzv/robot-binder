#include "ais_state_machine/AutonomyStates/StateInitializeRobot.h"

#include "xmlrpcpp/XmlRpcValue.h"

using namespace ais_state_machine;

std::vector< std::vector<int> > readNestedListParam(const std::string param_name);

StateInitializeRobot::StateInitializeRobot(shared_ptr<AbstractStateMachineManager> state_manager, 
                                ros::NodeHandle& nh) 
    : AbstractState(state_manager), AbstractROSState(nh)
{
    next_state_name_ = StateTypes::INITIALIZE_ROBOT;

    // ToDo (MichaelChiou) load key names dynamically from YAML file
    param_list_.push_back("USE_UV_LAMP");
    param_list_.push_back("UV_LAMP_WARMUP_TIME");
    param_list_.push_back("USE_LED_AS_UV");
    param_list_.push_back("MISSION_NAME");
    param_list_.push_back("MISSION_DIR");
    param_list_.push_back("MAX_RAIL_LENGTH");
    param_list_.push_back("FORWARD_DIRECTION_ON_RAIL");
    param_list_.push_back("USE_STOP_AS_UV");
    param_list_.push_back("RETURN_TO_RAIL_HOME");
    param_list_.push_back("ROBOT_GOAL_STOP_TIME");
    param_list_.push_back("MAX_RAIL_SPEED");
    param_list_.push_back("RAIL_IS_LEFT_ADJACENT");
    param_list_.push_back("RETURN_TO_RAIL_HOME_WITH_UV_ON");
    param_list_.push_back("RETURN_TO_RAIL_HOME_SPEED");
    param_list_.push_back("RETURN_HOME_INTENSITY");
    param_list_.push_back("OFF_RAIL_DETECTION");
    param_list_.push_back("IRRADIANCE_SPEED_CONTROL");
    param_list_.push_back("LOW_BATTERY");
    param_list_.push_back("ESTIMATED_BATTERY_USAGE");
    param_list_.push_back("CRITICAL_BATTERY");
    param_list_.push_back("MINIMUM_BATTERY");

    param_list_.push_back("USE_NEW_MISSION_PARAMS");
    param_list_.push_back("START_RAIL_NUMBER");
    param_list_.push_back("RAIL_JOBS");

    param_list_.push_back("MISSION_WAIT_TIME");

    all_params_initialized = false;
    rosbag_publisher_ = nh.advertise<std_msgs::UInt8>("/record/start", 1000);
    // latched connection for more reliable publishing
    flashlight_publisher_ = nh.advertise<std_msgs::UInt8>("/embedded/flashlight_values", 1000, true);
}

StateInitializeRobot::~StateInitializeRobot() 
{
    ROS_WARN("Destroying State StateInitializeRobot");
}

void StateInitializeRobot::runState()
{
    // If params are not initialized
    if (!all_params_initialized) 
    {
        // Check if ROS params names can be found before loading
        bool check_flag = true;
        for (const auto& param : param_list_) {
            if (!nh_.hasParam(param.c_str())) {
                ROS_WARN("Missing %s/%s parameter!", nh_.getUnresolvedNamespace().c_str(), param.c_str());
                check_flag = false; 
            }
        }

        all_params_initialized = check_flag;

        
        if (all_params_initialized) {
            initializeAndLoadMissionParameters();
            if (state_manager_->mission_config_ptr_->use_uv_lamp && doesMissionHaveGoals())
            {   
                std::vector<float> goal_intensity = state_manager_->mission_goal_manager_->getGoalIntensity();
                state_manager_->uv_lamp_controller_ptr_->setUVIntensity(goal_intensity);
                state_manager_->uv_lamp_controller_ptr_->setUVLampOn(true);
            }
        }
    }
    
    // TODO (Michael Chiou) Add code to warm-up lamps here?

    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::WAITING);

    evaluateStateStatus();
}

bool StateInitializeRobot::isStateFinished()
{
    ROS_INFO("State InitializeRobot Status: %s", has_finished_state_ ? "true" : "false");
    return has_finished_state_;
}

StateTypes StateInitializeRobot::getNextStateName()
{
    return next_state_name_;
}

void StateInitializeRobot::evaluateStateStatus() 
{
    auto now = ros::Time::now().toSec();
    auto start_time = (mission_initialization_time_ + mission_wait_time_).toSec();
    if (now < start_time)
    {
        ROS_INFO_THROTTLE(5.0, "Waiting for %f seconds to start mission...", start_time - now);
        next_state_name_ = StateTypes::INITIALIZE_ROBOT;

        // send flashlight off message continuously while waiting
        // std_msgs::UInt8 flashlight_msg;
        // flashlight_msg.data = 0;
        // flashlight_publisher_.publish(flashlight_msg);
    }
    else if (all_params_initialized) {
        ROS_INFO("All params initialized!");
        if(doesMissionHaveGoals()){
            // flashlight on during mission
            std_msgs::UInt8 flashlight_msg;
            flashlight_msg.data = 3;
            flashlight_publisher_.publish(flashlight_msg);

            const auto current_rail_number = state_manager_->rail_status_ptr_->getCurrentRailNumber();
            const auto target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();

            ROS_INFO(
                "State InitializeRobot: current rail number: %d, target rail number: %d",
                current_rail_number, target_rail_number
            );

            if (target_rail_number <= 0)
            {
                ROS_WARN("Invalid target rail number!");
                next_state_name_ = StateTypes::INITIALIZE_ROBOT;
            }
            else if (current_rail_number == target_rail_number) {
                // robot already in desired rail number, start disinfecting
                next_state_name_ = StateTypes::RAIL_DISINFECTION;
                is_disinfecting_current_target_ = true;
            } else {
                // move robot off the rail and go to desired rail number
                next_state_name_ = StateTypes::MOVE_ROBOT_OFF_RAIL;
                is_disinfecting_current_target_ = false;
            }
        }else{
            ROS_ERROR("Mission empty");
            next_state_name_ = StateTypes::COMPLETE;
        }
    } else {
        ROS_WARN("Params NOT initialized!");
        next_state_name_ = StateTypes::INITIALIZE_ROBOT;
    }
}

void StateInitializeRobot::reset() 
{
    count = 0;
    next_state_name_ = StateTypes::INITIALIZE_ROBOT;
    all_params_initialized = false;
    state_manager_->mission_goal_manager_->reset();
    ROS_INFO("StateInitializeRobot Finished Resetting");
}

void StateInitializeRobot::finish()
{
    ROS_INFO("StateInitializeRobot is now exiting");
    std_msgs::UInt8 trigger;
    rosbag_publisher_.publish(trigger);
    state_manager_->status_manager_ptr_->resetPose();
    incrementMissionCounter();
    ros::spinOnce();
    state_manager_->status_manager_ptr_->setRobotOnRails(true);
    reset();
}

void StateInitializeRobot::incrementMissionCounter()
{
    /**
     * there can be multiple jobs (rows) in a mission now
     * Hence uses rail status manager instead of ros params to keep track of rail mission
     */
    // int rail_mission_ctr_;
    // nh_.getParam("RAIL_MISSION_COUNTER", rail_mission_ctr_);
    // ++rail_mission_ctr_;
    // nh_.setParam("RAIL_MISSION_COUNTER", rail_mission_ctr_);

    ROS_INFO(
        "State InitializeRobot: %d/%d rails completed in current sub job, "
        "%d/%ld jobs completed",
        state_manager_->rail_status_ptr_->getCurrentSubJobIdx(),
        state_manager_->rail_status_ptr_->getTotalSubJobs(),
        state_manager_->rail_status_ptr_->getCurrentJobIdx(),
        state_manager_->mission_config_ptr_->rail_jobs.size()
    );

    if (is_disinfecting_current_target_) {
        // robot has started to disinfect the target rail. Update the rail number for next round
        const auto old_target_rail_number = state_manager_->rail_status_ptr_->getTargetRailNumber();
        state_manager_->rail_status_ptr_->incrementRailTarget();
        ROS_INFO(
            "State InitializeRobot: Rail target updated to %d from %d",
            state_manager_->rail_status_ptr_->getTargetRailNumber(), old_target_rail_number
        );

    } else {
        ROS_INFO(
            "State InitializeRobot: Target Rail %d is not being disinfected",
            state_manager_->rail_status_ptr_->getTargetRailNumber()
        );
    }

    nh_.setParam("RAIL_MISSION_COUNTER", state_manager_->rail_status_ptr_->getCurrentRailNumber());

}
void StateInitializeRobot::init()
{
    ROS_INFO("StateInitializeRobot is initializing");
    reset();
    state_manager_->status_manager_ptr_->setRobotStatus(RobotStatus::WAITING);
    state_manager_->status_manager_ptr_->getUpdatedParams();
}

void StateInitializeRobot::interruptState()
{
    ROS_INFO("StateInitializeRobot is interrupted");
}

void StateInitializeRobot::initializeAndLoadMissionParameters()
{
    nh_.getParam("USE_UV_LAMP", state_manager_->mission_config_ptr_->use_uv_lamp);
    nh_.getParam("UV_LAMP_WARMUP_TIME", state_manager_->mission_config_ptr_->uv_lamp_warmup_time);
    nh_.getParam("USE_LED_AS_UV", state_manager_->mission_config_ptr_->use_led_as_uv);
    nh_.getParam("MISSION_DIR", state_manager_->mission_config_ptr_->mission_directory);
    nh_.getParam("MISSION_NAME", state_manager_->mission_config_ptr_->mission_name);
    nh_.getParam("MAX_RAIL_LENGTH", state_manager_->mission_config_ptr_->max_rail_length);
    nh_.getParam("FORWARD_DIRECTION_ON_RAIL", state_manager_->mission_config_ptr_->is_forward_robot_rail_direction);
    nh_.getParam("USE_STOP_AS_UV", state_manager_->mission_config_ptr_->use_stop_as_uv);
    nh_.getParam("RETURN_TO_RAIL_HOME", state_manager_->mission_config_ptr_->need_return_init_rail_pos);
    nh_.getParam("ROBOT_GOAL_STOP_TIME", state_manager_->mission_config_ptr_->robot_goal_stop_time);
    nh_.getParam("MAX_RAIL_SPEED", state_manager_->mission_config_ptr_->max_rail_speed);
    nh_.getParam("RAIL_IS_LEFT_ADJACENT", state_manager_->mission_config_ptr_->rail_is_left_adjacent);
    nh_.getParam("RETURN_TO_RAIL_HOME_WITH_UV_ON", state_manager_->mission_config_ptr_->return_to_rail_home_with_uv_on);
    nh_.getParam("RETURN_TO_RAIL_HOME_SPEED", state_manager_->mission_config_ptr_->return_to_rail_home_speed);
    nh_.getParam("RETURN_HOME_INTENSITY", state_manager_->mission_config_ptr_->return_home_intensity);
    nh_.getParam("RAIL_TRANSITION_INTENSITY", state_manager_->mission_config_ptr_->rail_transition_intensity);
    nh_.getParam("IRRADIANCE_SPEED_CONTROL", state_manager_->mission_config_ptr_->irradiance_speed_control);
    nh_.getParam("LOW_BATTERY", state_manager_->mission_config_ptr_->low_battery);
    nh_.getParam("CRITICAL_BATTERY", state_manager_->mission_config_ptr_->critical_battery);
    nh_.getParam("ESTIMATED_BATTERY_USAGE", state_manager_->mission_config_ptr_->estimated_battery_usage);
    nh_.getParam("MINIMUM_BATTERY", state_manager_->mission_config_ptr_->minimum_battery);

    initializeMissionParams();
    initializeMissionWaitTime();

    ROS_INFO("Printing ros params to log");
    system("rosparam dump");
    // assume 1m error for every 100m; 
    state_manager_->mission_config_ptr_->wheel_odom_error_rate = 1.0/100.0; 

    // How much can wheel odom be wrong before we absolutely know we must have reached end
    state_manager_->mission_config_ptr_->max_tolerable_rail_length = 
        state_manager_->mission_config_ptr_->max_rail_length +
        state_manager_->mission_config_ptr_->max_rail_length * 
        state_manager_->mission_config_ptr_->wheel_odom_error_rate;
    
    // Path to specified mission
    state_manager_->mission_config_ptr_->mission_path = 
        state_manager_->mission_config_ptr_->mission_directory +
        state_manager_->mission_config_ptr_->mission_name; 
    // Load the mission    
    state_manager_->mission_goal_manager_->loadMission(
        state_manager_->mission_config_ptr_->mission_path.c_str()
    );
    // Initialize rail mission status if not done already (only at the start of a mission)
    if (state_manager_->rail_status_ptr_->getCurrentRailNumber() <= 0) {
        state_manager_->rail_status_ptr_->initializeMissionStatus();
        mission_initialization_time_ = ros::Time::now();
        // flashlight off while waiting for mission to start
        std_msgs::UInt8 flashlight_msg;
        flashlight_msg.data = 0;
        flashlight_publisher_.publish(flashlight_msg);
    }

    // Estimate mission battery consumption
    //state_manager_->mission_config_ptr_->estimated_battery_usage = estimateBatteryUsage(); <-- NOT IMPLEMENTED, TODO
}

std::vector< std::vector<int> > readNestedListParam(ros::NodeHandle& nh, const std::string param_name)
{
    // adapted from https://stackoverflow.com/a/65391046
    std::vector< std::vector<int> > nested_vect;

    XmlRpc::XmlRpcValue trajectory;
    nh.getParam(param_name, trajectory);

    // To ensure the reading will happen if the data is provided in right format
    if (trajectory.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for (int i = 0; i < trajectory.size(); i++)
        {
            std::vector<int> row;
            XmlRpc::XmlRpcValue trajectoryObject = trajectory[i];

            if (trajectoryObject.getType() != XmlRpc::XmlRpcValue::TypeArray) continue;

            // Individual rows in the nested list
            for (int j = 0; j < trajectoryObject.size(); j++)
            {
                XmlRpc::XmlRpcValue coord = trajectoryObject[j];
                if (coord.getType() == XmlRpc::XmlRpcValue::TypeInt)
                {
                    row.push_back(static_cast<int>(coord));
                }
            }

            nested_vect.push_back(row);
        }
    }

    return nested_vect;
}

void StateInitializeRobot::initializeMissionParams() {
    nh_.getParam("START_RAIL_NUMBER", state_manager_->mission_config_ptr_->start_rail_number);

    // simulate new mission params even if you want to use old mission params or stress test for uniformity
    bool use_new_mission_params = false;
    bool stress_test = false;
    nh_.getParam("USE_NEW_MISSION_PARAMS", use_new_mission_params);
    nh_.getParam("STRESS_TEST", stress_test);

    if (use_new_mission_params)
    {
        auto rail_jobs = readNestedListParam(nh_, "RAIL_JOBS");

        state_manager_->mission_config_ptr_->rail_jobs.clear();
        state_manager_->mission_config_ptr_->rail_jobs.reserve(rail_jobs.size());
        for (const auto &job: rail_jobs)
        {
            if (job.size() < 2) continue;
            state_manager_->mission_config_ptr_->rail_jobs.push_back({
                job[0], job[1]
            });
        }

        auto rail_positions = readNestedListParam(nh_, "/rail_navigation/rail_positions");
        if (rail_positions.empty())
        {
            ROS_WARN("Rail positions empty, assigning arbitrary number of rails");

            int number_of_rail_missions;
            nh_.getParam("NUMBER_OF_RAIL_MISSIONS", number_of_rail_missions);
            state_manager_->mission_config_ptr_->num_rails = number_of_rail_missions * 4;
        }
        else
        {
            state_manager_->mission_config_ptr_->num_rails = rail_positions.size();
        }
    }
    else if (stress_test)
    {
        int number_of_rail_missions;
        int number_of_rails_per_mission;
        nh_.getParam("NUMBER_OF_RAIL_MISSIONS", number_of_rail_missions);
        nh_.getParam("STRESS_TEST_NUM_RAILS", number_of_rails_per_mission);

        // make num_rails large enough to only move to adjacent rail each time without going to navigation state
        state_manager_->mission_config_ptr_->num_rails = number_of_rails_per_mission * 4;

        // equivalent rail job using new params
        state_manager_->mission_config_ptr_->start_rail_number
            = (state_manager_->mission_config_ptr_->rail_is_left_adjacent)
                    ? 1 : number_of_rails_per_mission;

        state_manager_->mission_config_ptr_->rail_jobs.resize(number_of_rail_missions);

        bool is_current_job_left = state_manager_->mission_config_ptr_->rail_is_left_adjacent;

        for (int i = 0; i < number_of_rail_missions; ++i, is_current_job_left = !is_current_job_left)
        {
            if (is_current_job_left)
            {
                state_manager_->mission_config_ptr_->rail_jobs[i] = {
                    1, std::max(number_of_rails_per_mission - 1, 1)
                };
            }
            else
            {
                state_manager_->mission_config_ptr_->rail_jobs[i] = {
                    number_of_rails_per_mission,
                    number_of_rails_per_mission > 1 ? 2 : 1
                };
            }
       }
    }
    else
    {
        int number_of_rail_missions;
        nh_.getParam("NUMBER_OF_RAIL_MISSIONS", number_of_rail_missions);

        // make num_rails large enough to only move to adjacent rail each time without going to navigation state
        state_manager_->mission_config_ptr_->num_rails = number_of_rail_missions * 4;

        // equivalent rail job using new params
        state_manager_->mission_config_ptr_->rail_jobs.resize(1);

        if (state_manager_->mission_config_ptr_->rail_is_left_adjacent)
        {
            state_manager_->mission_config_ptr_->start_rail_number = 1;
            state_manager_->mission_config_ptr_->rail_jobs[0] = {1, number_of_rail_missions};
        }
        else
        {
            state_manager_->mission_config_ptr_->start_rail_number = number_of_rail_missions;
            state_manager_->mission_config_ptr_->rail_jobs[0] = {number_of_rail_missions, 1};
        }
    }

    ROS_INFO("rail jobs vect array");
    for (const auto &i: state_manager_->mission_config_ptr_->rail_jobs)
    {
        ROS_INFO("%d, %d", i[0], i[1]);
    }

    ROS_INFO("number of rails: %d", state_manager_->mission_config_ptr_->num_rails);
}

void StateInitializeRobot::initializeMissionWaitTime() {
    std::vector<int> hr_min;
    nh_.getParam("MISSION_WAIT_TIME", hr_min);
    if (hr_min.size() < 2)
    {
        ROS_WARN("invalid MISSION_WAIT_TIME param, setting it to 0");
        mission_wait_time_ = ros::Duration(0.0);
    }
    else
    {
        mission_wait_time_ = ros::Duration(
            (double)(hr_min[0] * 3600) + (hr_min[1] * 60)
        );
    }
    state_manager_->status_manager_ptr_->setMissionStartTime(ros::Time::now() + mission_wait_time_);
    ROS_INFO("Mission wait time set to %f secs", mission_wait_time_.toSec());
}

bool StateInitializeRobot::doesMissionHaveGoals(){
    return (state_manager_->mission_goal_manager_->getTotalGoalsInMission() > 0);
}
