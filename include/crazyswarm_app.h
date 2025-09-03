/*
* crazyswarm_app.h
*
* ---------------------------------------------------------------------
* Copyright (C) 2023 Matthew (matthewoots at gmail.com)
*
*  This program is free software; you can redistribute it and/or
*  modify it under the terms of the GNU General Public License
*  as published by the Free Software Foundation; either version 2
*  of the License, or (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
* ---------------------------------------------------------------------
*/

#include <memory>
#include <vector>
#include <regex>
#include <mutex>
#include <queue>
#include <string>
#include <ctime>

#include <Eigen/Dense>

#include "std_srvs/srv/empty.hpp"

#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/set_group_mask.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"

#include "crazyswarm_application/msg/user_command.hpp"
#include "crazyswarm_application/msg/agents_state_feedback.hpp"
#include "crazyswarm_application/msg/agent_state.hpp"

#include "crazyswarm_application/msg/named_pose_array.hpp"
#include "crazyswarm_application/msg/named_pose.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

// visualize obstacle
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include "common.h"
#include "agent.h"
#include "kdtree.h"

#include <CSVWriter.h>

using std_srvs::srv::Empty;

using crazyflie_interfaces::srv::Takeoff;
using crazyflie_interfaces::srv::Land;
using crazyflie_interfaces::srv::GoTo;
using crazyflie_interfaces::srv::SetGroupMask;
using crazyflie_interfaces::msg::VelocityWorld;

using crazyswarm_application::msg::UserCommand;
using crazyswarm_application::msg::AgentsStateFeedback;
using crazyswarm_application::msg::AgentState;

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::PointCloud2;

using crazyswarm_application::msg::NamedPoseArray;
using crazyswarm_application::msg::NamedPose;

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

using namespace common;

using namespace RVO;

namespace cs2
{

    class cs2_application : public rclcpp::Node
    {
        public:

            cs2_application()
                : Node("cs2_application"), clock(RCL_ROS_TIME), tf2_bc(this)
            {
                start_node_time = clock.now();

                // declare global commands
                this->declare_parameter("queue_size", 1);
                this->declare_parameter("visibility_expansion_factor", 1.0);
                this->declare_parameter("orca_static_expansion_factor", 1.0);
                this->declare_parameter("radio_connection_timeout", 1.0);
                this->declare_parameter("concave_obstacles", false);

                this->declare_parameter("trajectory_parameters.max_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.maximum_yaw_change", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_land_velocity", -1.0);
                this->declare_parameter("trajectory_parameters.takeoff_height", -1.0);
                this->declare_parameter("trajectory_parameters.reached_threshold", -1.0);
                this->declare_parameter("trajectory_parameters.planning_rate", -1.0);
                this->declare_parameter("trajectory_parameters.communication_radius", -1.0);
                this->declare_parameter("trajectory_parameters.protected_zone", -1.0);
                this->declare_parameter("trajectory_parameters.planning_horizon_scale", -1.0);
                this->declare_parameter("trajectory_parameters.height_range", std::vector<double>{});

                this->declare_parameter("log_path", "");

                max_queue_size = 
                    this->get_parameter("queue_size").get_parameter_value().get<int>();

                concave_obstacles = 
                    this->get_parameter("concave_obstacles").get_parameter_value().get<bool>();
                max_velocity = 
                    this->get_parameter("trajectory_parameters.max_velocity").get_parameter_value().get<double>();
                maximum_yaw_change = 
                    this->get_parameter("trajectory_parameters.maximum_yaw_change").get_parameter_value().get<double>();
                takeoff_land_velocity = 
                    this->get_parameter("trajectory_parameters.takeoff_land_velocity").get_parameter_value().get<double>();
                takeoff_height = 
                    this->get_parameter("trajectory_parameters.takeoff_height").get_parameter_value().get<double>();
                reached_threshold = 
                    this->get_parameter("trajectory_parameters.reached_threshold").get_parameter_value().get<double>();
                planning_rate = 
                    this->get_parameter("trajectory_parameters.planning_rate").get_parameter_value().get<double>();
                communication_radius = 
                    this->get_parameter("trajectory_parameters.communication_radius").get_parameter_value().get<double>();
                protected_zone = 
                    this->get_parameter("trajectory_parameters.protected_zone").get_parameter_value().get<double>();
                planning_horizon_scale = 
                    this->get_parameter("trajectory_parameters.planning_horizon_scale").get_parameter_value().get<double>();
                std::vector<double> height_range_vector = 
                    this->get_parameter("trajectory_parameters.height_range").get_parameter_value().get<std::vector<double>>();
                assert(height_range_vector.size() == 2);

                radio_connection_timeout = 
                    this->get_parameter("radio_connection_timeout").get_parameter_value().get<double>();
                visibility_expansion_factor = 
                    this->get_parameter("visibility_expansion_factor").get_parameter_value().get<double>();
                orca_static_expansion_factor = 
                    this->get_parameter("orca_static_expansion_factor").get_parameter_value().get<double>();
                log_path = 
                    this->get_parameter("log_path").get_parameter_value().get<std::string>();
                height_range = 
                    std::make_pair(height_range_vector[0], height_range_vector[1]);

                // TODO : get this as parameter from config 
                // smaclike
                int NUM_DRONES = 2;
                // n_agents / n_enemies
                int n_agents = 1;
                int n_enemies = NUM_DRONES - n_agents;

                // load crazyflies from params
                auto node_parameters_iface = this->get_node_parameters_interface();
                const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                    node_parameters_iface->get_parameter_overrides();

                auto cf_names = extract_names(parameter_overrides, "robots");
                for (const auto &name : cf_names) 
                {
                    RCLCPP_INFO(this->get_logger(), "creating agent map for '%s'", name.c_str());

                    // Remove cf_ from cf_XX
                    int id = id_from_key(name);
                    
                    std::vector<double> pos = parameter_overrides.at("robots." + name + ".initial_position").get<std::vector<double>>();
                    bool mission_capable = parameter_overrides.at("robots." + name + ".mission_capable").get<bool>();

                    agent_state a_s;
                    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
                    aff.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
                    a_s.t = clock.now();
                    a_s.transform = aff;
                    a_s.flight_state = IDLE;
                    a_s.radio_connection = false;
                    a_s.completed = false;
                    a_s.mission_capable = mission_capable;

                    RCLCPP_INFO(this->get_logger(), "(%s) %lf %lf %lf", name.c_str(), 
                        pos[0], pos[1], pos[2]);

                    // to get the index of string std::stoi(str_copy)
                    agents_states.insert(
                        std::pair<std::string, agent_state>(name, a_s));

                    std::function<void(const PoseStamped::SharedPtr)> pcallback = 
                        std::bind(&cs2_application::pose_callback,
                        this, std::placeholders::_1, --agents_states.end());
                    std::function<void(const Twist::SharedPtr)> vcallback = 
                        std::bind(&cs2_application::twist_callback,
                        this, std::placeholders::_1, --agents_states.end());
                    
                    agent_struct tmp;
                    tmp.go_to = this->create_client<GoTo>(name + "/go_to");
                    tmp.land = this->create_client<Land>(name + "/land");
                    tmp.set_group = this->create_client<SetGroupMask>(name + "/set_group_mask");
                    tmp.emergency = this->create_client<Empty>(name + "/emergency");
                    
                    pose_sub.insert({name, this->create_subscription<PoseStamped>(
                        name + "/pose", 14, pcallback)});
                    vel_sub.insert({name, this->create_subscription<Twist>(
                        name + "/vel", 14, vcallback)});

                    tmp.vel_world_publisher = 
                        this->create_publisher<VelocityWorld>(name + "/cmd_velocity_world", 25);
                        //this->create_publisher<geometry_msgs::msg::Twist>(name + "/cmd_vel_legacy", 25);
                        //this->create_publisher<FullState>(name + "/cmd_velocity_world", 25);

                    tmp.hover_world_publisher = 
                        this->create_publisher<Hover>(name + "/cmd_hover", 25);

                    tmp.cloud_publisher = 
                        this->create_publisher<PointCloud2>(name + "/obstacles", 25);

                    agents_comm.insert({name, tmp});
                
                    Agent new_rvo2_agent = Agent(
                        id, (float)(1/planning_rate), 10, (float)max_velocity, 
                        (float)communication_radius, (float)protected_zone, 
                        (float)(planning_horizon_scale * 1/planning_rate),
                        (float)height_range.first, (float)height_range.second);
                    rvo_agents.insert({name, new_rvo2_agent});

                    RCLCPP_INFO(this->get_logger(), "agent %s created", name.c_str());
                }

                std::vector<visibility_graph::obstacle> visibility_obstacle_list;
                load_obstacle_map(
                    parameter_overrides, visibility_expansion_factor, 
                    visibility_obstacle_list, concave_obstacles);
                visibility_obstacle_map.obs = visibility_obstacle_list;
                visibility_obstacle_map.inflation = protected_zone;
                std::vector<visibility_graph::obstacle> orca_obstacle_list;
                load_obstacle_map(
                    parameter_overrides, orca_static_expansion_factor, 
                    orca_obstacle_list, concave_obstacles);
                orca_obstacle_map.obs = orca_obstacle_list;
                orca_obstacle_map.inflation = protected_zone;

                pose_publisher = 
                    this->create_publisher<NamedPoseArray>("poses", 7);
                
                target_publisher = 
                    this->create_publisher<MarkerArray>("targets", 7);

                agent_state_publisher = 
                    this->create_publisher<AgentsStateFeedback>("agents", 7);

                subscription_user = 
                    this->create_subscription<UserCommand>("user", 30, std::bind(&cs2_application::user_callback, this, _1));

                takeoff_all_client = this->create_client<Takeoff>("/all/takeoff");
                land_all_client = this->create_client<Land>("/all/land");

                auto t_planning = (1/planning_rate) * 1000ms;
                handler_timer = this->create_wall_timer(
                    t_planning, std::bind(&cs2_application::handler_timer_callback, this));
                RCLCPP_INFO(this->get_logger(), "end_constructor");

                // rotate z -90 then x -90 for it to be FRD
                nwu_to_rdf = enu_to_rdf = Eigen::Affine3d::Identity();
                nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,0,1)));
                
                nwu_to_enu = nwu_to_rdf;
                enu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
                nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
            };

        private:

            // parameters
            int max_queue_size;

            bool center_origin;
            bool concave_obstacles;

            double max_velocity;
            double maximum_yaw_change;
            double takeoff_land_velocity;
            double takeoff_height;
            double reached_threshold;
            double planning_rate;
            double communication_radius;
            double protected_zone;
            double planning_horizon_scale;

            // TODO : get this as parameter from config 
            // smaclike
            int NUM_DRONES;
            int n_agents;
            int n_enemies;
            // threshold parameters
            double time_threshold;
            double observation_threshold;
            double visibility_expansion_factor;
            double orca_static_expansion_factor;

            double radio_connection_timeout;

            int observation_limit;

            std::string log_path;

            Eigen::Affine3d nwu_to_rdf;
            Eigen::Affine3d enu_to_rdf;
            Eigen::Affine3d nwu_to_enu;

            visibility_graph::global_map visibility_obstacle_map;
            visibility_graph::global_map orca_obstacle_map;

            rclcpp::Client<Takeoff>::SharedPtr takeoff_all_client;
            rclcpp::Client<Land>::SharedPtr land_all_client;

            std::map<std::string, agent_struct> agents_comm;
            std::map<std::string, agent_state> agents_states;

            std::map<std::string, rclcpp::Subscription<PoseStamped>::SharedPtr> pose_sub;
            std::map<std::string, rclcpp::Subscription<Twist>::SharedPtr> vel_sub;

            std::map<std::string, Agent> rvo_agents;

            std::pair<double, double> height_range;

            rclcpp::TimerBase::SharedPtr planning_timer;
            rclcpp::TimerBase::SharedPtr handler_timer;
        
            tf2_ros::TransformBroadcaster tf2_bc;

            std::mutex agent_update_mutex;

            rclcpp::Clock clock;

            rclcpp::Time start_node_time;

            kdtree *kd_tree;

            rclcpp::Subscription<UserCommand>::SharedPtr subscription_user;

            rclcpp::Publisher<NamedPoseArray>::SharedPtr pose_publisher;
            rclcpp::Publisher<AgentsStateFeedback>::SharedPtr agent_state_publisher;
            rclcpp::Publisher<MarkerArray>::SharedPtr target_publisher;
            
            // void conduct_planning(
            //     Eigen::Vector3d &desired, std::vector<std::pair<float, const Eval_agent>> &neighbors, std::string mykey, agent_state state);

            void conduct_planning(
                Eigen::Vector3d &desired, std::string mykey, agent_state state);

            void user_callback(const UserCommand::SharedPtr msg);

            void pose_callback(
                const PoseStamped::SharedPtr msg, 
                std::map<std::string, agent_state>::iterator state);
            
            void twist_callback(
                const Twist::SharedPtr msg, 
                std::map<std::string, agent_state>::iterator state);

            // timers
            void handler_timer_callback(); 

            void send_land_and_update(
                std::map<std::string, agent_state>::iterator s,
                std::map<std::string, agent_struct>::iterator c);
            
            // helper function
            int id_from_key(std::string key, int remove = 3);

            std::unique_ptr<PointCloud2> convert_cloud(
                const std::vector<std::pair<float, const Eval_agent>>& obstacles);
    };
}
