/*
* rviz_visualizer.cpp
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
#include "math.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#include "crazyswarm_application/msg/agents_state_feedback.hpp"
#include "crazyswarm_application/msg/agent_state.hpp"
#include "crazyswarm_application/msg/user_command.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "builtin_interfaces/msg/duration.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include "rclcpp/rclcpp.hpp"

#include "common.h"

#include <Eigen/Dense>

using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::TransformStamped;
using rviz_2d_overlay_msgs::msg::OverlayText;
using crazyswarm_application::msg::AgentsStateFeedback;
using crazyswarm_application::msg::AgentState;
using crazyswarm_application::msg::UserCommand;
using std_msgs::msg::ColorRGBA;

using namespace std::chrono_literals;

using namespace common;

using std::placeholders::_1;

class RvizVisualizer : public rclcpp::Node
{
    private:

        rclcpp::Clock clock;

        rclcpp::Publisher<OverlayText>::SharedPtr text_publisher;
        rclcpp::Publisher<Marker>::SharedPtr obstacle_publisher;
        rclcpp::Publisher<Marker>::SharedPtr orca_obstacle_publisher;
        rclcpp::Publisher<Marker>::SharedPtr visibility_obstacle_publisher;
        rclcpp::Publisher<MarkerArray>::SharedPtr laser_publisher;

        rclcpp::Subscription<AgentsStateFeedback>::SharedPtr agent_state_subscriber;
        rclcpp::Subscription<MarkerArray>::SharedPtr targets_subscriber;
        rclcpp::Subscription<UserCommand>::SharedPtr user_command_subscriber;

        rclcpp::TimerBase::SharedPtr visualizing_timer;

        //std::string mesh_path;

        tf2_ros::TransformBroadcaster tf2_bc;

        Eigen::Affine3d nwu_to_rdf;
        Eigen::Affine3d enu_to_rdf;

        bool concave_obstacles;

        std::vector<visibility_graph::obstacle> global_obstacle_list;
        std::vector<visibility_graph::obstacle> visibility_obstacle_list;
        std::vector<visibility_graph::obstacle> orca_obstacle_list;
        std::map<int, Eigen::Vector3d> latest_agent_positions;

        visualization_msgs::msg::Marker global_obs_visualize;
        visualization_msgs::msg::Marker visibility_obs_visualize;
        visualization_msgs::msg::Marker orca_obs_visualize;

        double visibility_expansion_factor;
        double orca_static_expansion_factor;
        int n_agents_;   // drones with id <= n_agents_ are allies
        common::string_dictionary dict;

        void visualizing_timer_callback()
        {
            obstacle_publisher->publish(global_obs_visualize);
            orca_obstacle_publisher->publish(orca_obs_visualize);
            visibility_obstacle_publisher->publish(visibility_obs_visualize);
        }

        void agents_state_callback(
            const AgentsStateFeedback::SharedPtr msg)
        {
            AgentsStateFeedback copy = *msg;
            OverlayText text_msg;
            text_msg.action = OverlayText::ADD;
            text_msg.horizontal_alignment = OverlayText::LEFT;
            text_msg.vertical_alignment = OverlayText::TOP;
            text_msg.width = 320;
            text_msg.height = 512;
            text_msg.text_size = 9.0;
            text_msg.line_width = 320;
            text_msg.font = "DejaVu Sans Mono";

            std::string text;
            std::map<int, agent_state> agents_map;

            for (auto &obs : copy.pre_obs)
            {
                auto &agent = obs.agent;
                std::string str_copy = agent.id;
                str_copy.erase(0, 3); // remove "cf_"
                int id = std::stoi(str_copy);

                agent_state state;
                state.flight_state = agent.flight_state;
                state.radio_connection = agent.connected;
                state.completed = agent.completed;
                agents_map.insert({id, state});
            }

            for (auto it = agents_map.begin(); it != agents_map.end(); it++)
                call_state_text(it, text);

            text_msg.text = text;
            text_publisher->publish(text_msg);
        }

        void targets_callback(const MarkerArray::SharedPtr msg)
        {
            for (const auto &marker : msg->markers)
            {
                if (marker.points.empty())
                    continue;
                latest_agent_positions[marker.id] = Eigen::Vector3d(
                    marker.points.front().x,
                    marker.points.front().y,
                    marker.points.front().z);
            }
        }

        void user_command_callback(const UserCommand::SharedPtr msg)
        {
            if (msg->cmd != dict.attack)
                return;

            RCLCPP_INFO(this->get_logger(), "rviz attack cmd received: n=%zu goal=(%.3f %.3f %.3f)",
                msg->uav_id.size(), msg->goal.x, msg->goal.y, msg->goal.z);

            MarkerArray laser_array;
            int marker_id = 0;
            const rclcpp::Time now = clock.now();

            if (msg->uav_id.empty())
                return;

            const int agent_id = parse_agent_id(msg->uav_id.front());
            auto pos_it = latest_agent_positions.find(agent_id);
            if (pos_it == latest_agent_positions.end()){
                RCLCPP_WARN(this->get_logger(), "rviz attack skipped: no cached /targets pose for %s",
                    msg->uav_id.front().c_str());
                return;
            }

            Eigen::Vector3d start = pos_it->second;
            Eigen::Vector3d end(msg->goal.x, msg->goal.y, msg->goal.z);
            if (msg->uav_id.size() >= 2)
            {
                const int target_id = parse_agent_id(msg->uav_id[1]);
                auto target_pos_it = latest_agent_positions.find(target_id);
                if (target_pos_it != latest_agent_positions.end())
                {
                    end = target_pos_it->second;
                    RCLCPP_INFO(this->get_logger(),
                        "rviz attack target id=%s resolved to current pos (%.3f %.3f %.3f)",
                        msg->uav_id[1].c_str(), end.x(), end.y(), end.z());
                }
            }
            clip_to_closest_obstacle(start, end);

            // Faction-based laser colours matching SimpleCAAviary convention:
            //   ally   → laserHitColor  [0,    1,    1   ]  cyan
            //   enemy  → laserMissColor [0.99, 0.75, 0.79]  pink/rose
            const bool is_ally = (agent_id > 0 && agent_id <= n_agents_);
            const float lr = is_ally ? 0.00f : 0.99f;
            const float lg = is_ally ? 1.00f : 0.75f;
            const float lb = is_ally ? 1.00f : 0.79f;

            Marker laser;
            laser.header.frame_id = "/world";
            laser.header.stamp = now;
            laser.ns = "laser_attack";
            laser.id = marker_id++;
            laser.type = Marker::LINE_LIST;
            laser.action = Marker::ADD;
            laser.pose.orientation.w = 1.0;
            laser.scale.x = 0.06;
            laser.color.r = lr;
            laser.color.g = lg;
            laser.color.b = lb;
            laser.color.a = 1.0;
            builtin_interfaces::msg::Duration laser_lifetime;
            laser_lifetime.sec = 2;
            laser.lifetime = laser_lifetime;

            geometry_msgs::msg::Point p0;
            p0.x = start.x(); p0.y = start.y(); p0.z = start.z();
            geometry_msgs::msg::Point p1;
            p1.x = end.x(); p1.y = end.y(); p1.z = end.z();
            laser.points.push_back(p0);
            laser.points.push_back(p1);
            laser_array.markers.push_back(laser);

            Marker impact;
            impact.header = laser.header;
            impact.ns = "laser_impact";
            impact.id = marker_id++;
            impact.type = Marker::SPHERE;
            impact.action = Marker::ADD;
            impact.pose.position = p1;
            impact.pose.orientation.w = 1.0;
            impact.scale.x = 0.06;
            impact.scale.y = 0.06;
            impact.scale.z = 0.06;
            impact.color.r = lr;
            impact.color.g = lg;
            impact.color.b = lb;
            impact.color.a = 1.0;
            impact.lifetime = laser_lifetime;
            laser_array.markers.push_back(impact);

            if (!laser_array.markers.empty())
                laser_publisher->publish(laser_array);
        }

        int parse_agent_id(const std::string &name) const
        {
            std::smatch match;
            if (std::regex_search(name, match, std::regex("(\\d+)$")))
                return std::stoi(match[1].str());
            return -1;
        }

        void clip_to_closest_obstacle(const Eigen::Vector3d &start, Eigen::Vector3d &end) const
        {
            std::pair<Eigen::Vector3d, Eigen::Vector3d> segment{start, end};
            std::vector<Eigen::Vector3d> intersections;
            get_line_polygon_intersection(orca_obstacle_list, segment, intersections);
            if (intersections.empty())
                return;

            double closest_dist = (end - start).norm();
            Eigen::Vector3d closest_point = end;
            for (const auto &p : intersections)
            {
                const double dist = (p - start).norm();
                if (dist < closest_dist)
                {
                    closest_dist = dist;
                    closest_point = p;
                }
            }
            end = closest_point;
        }

        void get_line_polygon_intersection(
            const std::vector<visibility_graph::obstacle> &obs_list,
            const std::pair<Eigen::Vector3d, Eigen::Vector3d> &s_e,
            std::vector<Eigen::Vector3d> &intersections) const
        {
            for (const auto &obs : obs_list)
            {
                const int obs_vert_size = obs.v.size();

                Eigen::Vector3d tmp_pop(obs.c[0], obs.c[1], obs.h.first);
                Eigen::Vector3d tmp_n(0, 0, -1);
                Eigen::Vector3d result;
                if (visibility_graph::get_line_plane_intersection(s_e, tmp_n, tmp_pop, result))
                    intersections.push_back(result);

                tmp_pop = Eigen::Vector3d(obs.c[0], obs.c[1], obs.h.second);
                tmp_n = Eigen::Vector3d(0, 0, 1);
                if (visibility_graph::get_line_plane_intersection(s_e, tmp_n, tmp_pop, result))
                    intersections.push_back(result);

                for (int i = 0; i < obs_vert_size; i++)
                {
                    const int next_i = (i + 1) % obs_vert_size;
                    tmp_pop = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
                    tmp_n = Eigen::Vector3d(
                        obs.v[next_i].y() - obs.v[i].y(),
                        -obs.v[next_i].x() + obs.v[i].x(),
                        0.0);
                    if (visibility_graph::get_line_plane_intersection(s_e, tmp_n, tmp_pop, result))
                        intersections.push_back(result);
                }
            }
        }

        void call_state_text(
            std::map<int, agent_state>::iterator agent,
            std::string &text)
        {
            // Header line: "cf_N  rc:y  task:y"
            text += "<span style='color:#00fff0;'>cf_" + std::to_string(agent->first) + "</span>";
            text += "  rc:" + std::string(agent->second.radio_connection ? "<span style='color:#00ff00;'>y</span>" : "<span style='color:#ff4444;'>n</span>");
            text += "  task:" + std::string(agent->second.completed ? "<span style='color:#00ff00;'>done</span>" : "----");
            text += "\n";

            // State line: "  state: MOVE_VELOCITY"
            text += "  state: ";
            switch (agent->second.flight_state)
            {
                case IDLE:              text += "IDLE";           break;
                case TAKEOFF:           text += "<span style='color:#ffff00;'>TAKEOFF</span>";  break;
                case MOVE:              text += "<span style='color:#00ff88;'>MOVE</span>";     break;
                case MOVE_VELOCITY:     text += "<span style='color:#00ff88;'>MOVE_VEL</span>"; break;
                case INTERNAL_TRACKING: text += "<span style='color:#aaffaa;'>TRACKING</span>"; break;
                case HOVER:             text += "<span style='color:#44aaff;'>HOVER</span>";    break;
                case LAND:              text += "<span style='color:#ffaa00;'>LAND</span>";     break;
                case EMERGENCY:         text += "<span style='color:#ff0000;'>EMERGENCY</span>"; break;
                default:                text += "???";            break;
            }
            text += "\n---\n";
        }

        void obstacles_to_vertices_vector(
            std::vector<visibility_graph::obstacle> &obs_list,
            visualization_msgs::msg::Marker &msg,
            Eigen::Vector3d rgb, int id)
        {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> vect_vert;

            // vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> format
            for (visibility_graph::obstacle &obs : obs_list)
            {
                // Get the centroid 
                Eigen::Vector2d centroid;
                std::vector<Eigen::Vector2d> vert;             

                int obs_vert_pair_size, obs_hori_pair_size;
                obs_vert_pair_size = obs_hori_pair_size = obs.v.size();

                // Add lines for verticals
                for (int i = 0; i < obs_vert_pair_size; i++)
                {
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                    vert_pair.first = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
                    vert_pair.second = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.second);
                    vect_vert.push_back(vert_pair);
                }
                // Add lines for horizontals
                for (int i = 0; i < obs_hori_pair_size; i++)
                {
                    std::pair<Eigen::Vector3d, Eigen::Vector3d> vert_pair;
                    vert_pair.first = Eigen::Vector3d(
                        obs.v[i % obs_hori_pair_size].x(), 
                        obs.v[i % obs_hori_pair_size].y(), obs.h.first);
                    vert_pair.second = Eigen::Vector3d(
                        obs.v[(i+1) % obs_hori_pair_size].x(), 
                        obs.v[(i+1) % obs_hori_pair_size].y(), obs.h.first);
                    vect_vert.push_back(vert_pair);

                    vert_pair.first = Eigen::Vector3d(
                        obs.v[i % obs_hori_pair_size].x(), 
                        obs.v[i % obs_hori_pair_size].y(), obs.h.second);
                    vert_pair.second = Eigen::Vector3d(
                        obs.v[(i+1) % obs_hori_pair_size].x(), 
                        obs.v[(i+1) % obs_hori_pair_size].y(), obs.h.second);
                    vect_vert.push_back(vert_pair);
                }

            }

            msg.header.frame_id = "/world";
            msg.header.stamp = clock.now();
            msg.type = visualization_msgs::msg::Marker::LINE_LIST;
            msg.action = visualization_msgs::msg::Marker::ADD;

            msg.id = id;

            msg.color.r = rgb.x();
            msg.color.g = rgb.y();
            msg.color.b = rgb.z();

            msg.color.a = 0.75;

            msg.scale.x = 0.05;

            // Create the vertices line list
            for (auto &vert_pair : vect_vert)
            {
                geometry_msgs::msg::Point p1, p2;
                p1.x = vert_pair.first.x();
                p2.x = vert_pair.second.x();

                p1.y = vert_pair.first.y();
                p2.y = vert_pair.second.y();

                p1.z = vert_pair.first.z();
                p2.z = vert_pair.second.z();

                msg.points.push_back(p1);
                msg.points.push_back(p2);
            }
        }

    public:

        RvizVisualizer()
        : Node("rviz_visualizer"), clock(RCL_ROS_TIME), tf2_bc(this)
        {
            text_publisher = this->create_publisher<OverlayText>("rviz/text", 10);
            
            obstacle_publisher = this->create_publisher<Marker>("rviz/obstacles", 10);
            orca_obstacle_publisher = this->create_publisher<Marker>("rviz/orca", 10);
            visibility_obstacle_publisher = this->create_publisher<Marker>("rviz/visibility", 10);
            laser_publisher = this->create_publisher<MarkerArray>("rviz/laser", 10);

            visualizing_timer = this->create_wall_timer(
                1000ms, std::bind(&RvizVisualizer::visualizing_timer_callback, this));
        
            //this->declare_parameter("mesh_path", "");
            this->declare_parameter("concave_obstacles", false);
            this->declare_parameter("visibility_expansion_factor", 1.0);
            this->declare_parameter("orca_static_expansion_factor", 1.0);
            this->declare_parameter("n_agents", 1);

            visibility_expansion_factor =
                this->get_parameter("visibility_expansion_factor").get_parameter_value().get<double>();
            orca_static_expansion_factor =
                this->get_parameter("orca_static_expansion_factor").get_parameter_value().get<double>();
            n_agents_ =
                this->get_parameter("n_agents").get_parameter_value().get<int>();
            
            //mesh_path = 
            //    this->get_parameter("mesh_path").get_parameter_value().get<std::string>();
            concave_obstacles = 
                this->get_parameter("concave_obstacles").get_parameter_value().get<bool>();

            // load obstacles from params
            auto node_parameters_iface = this->get_node_parameters_interface();
            const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
                node_parameters_iface->get_parameter_overrides();

            // actual map
            load_obstacle_map(
                parameter_overrides, 0.0, global_obstacle_list,
                concave_obstacles);
            obstacles_to_vertices_vector(
                global_obstacle_list, global_obs_visualize, 
                Eigen::Vector3d(1.0, 1.0, 1.0), 1);
            // visibility map
            load_obstacle_map(
                parameter_overrides, visibility_expansion_factor, 
                visibility_obstacle_list, concave_obstacles);
            obstacles_to_vertices_vector(
                visibility_obstacle_list, visibility_obs_visualize, 
                Eigen::Vector3d(0.0, 1.0, 1.0), 2);
            // visibility map
            load_obstacle_map(
                parameter_overrides, orca_static_expansion_factor, 
                orca_obstacle_list, concave_obstacles);
            obstacles_to_vertices_vector(
                orca_obstacle_list, orca_obs_visualize, 
                Eigen::Vector3d(1.0, 1.0, 0.0), 3);

            agent_state_subscriber = 
                this->create_subscription<AgentsStateFeedback>("agents",
                2, std::bind(&RvizVisualizer::agents_state_callback, this, _1));
            targets_subscriber =
                this->create_subscription<MarkerArray>("targets",
                10, std::bind(&RvizVisualizer::targets_callback, this, _1));
            user_command_subscriber =
                this->create_subscription<UserCommand>("user",
                10, std::bind(&RvizVisualizer::user_command_callback, this, _1));
        
            // rotate z -90 then x -90 for it to be RDF
            nwu_to_rdf = enu_to_rdf = Eigen::Affine3d::Identity();
            nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0,0,1)));
            
            enu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
            nwu_to_rdf.rotate(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(1,0,0)));
                
        }        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizVisualizer>());
    rclcpp::shutdown();
    return 0;
}
