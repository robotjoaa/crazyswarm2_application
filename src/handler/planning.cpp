/*
* planning.cpp
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

#include "crazyswarm_app.h"

void cs2::cs2_application::conduct_planning(
    Eigen::Vector3d &desired, std::string mykey, agent_state state) 
{
    bool my_faction = (id_from_key(mykey) <= this->n_agents); 
    auto it = rvo_agents.find(mykey);
    if (it == rvo_agents.end())
        return;

    if (agents_states.size() > 1)
    {
        // Construct KD-tree
        kd_tree = kd_create(3);

        for (auto &[key, agent] : agents_states)
        { 
            if (strcmp(mykey.c_str(), key.c_str()) == 0)
                continue;
            Eval_agent *node = new Eval_agent;
            node->position_ = agent.transform.translation().cast<float>();
            node->velocity_ = agent.velocity.cast<float>();
            node->radius_ = (float)protected_zone;
            bool faction = (id_from_key(key) <= this->n_agents);
            node->type_ = (my_faction != faction) ? ENEMY : ALLY;
            node->id_ = id_from_key(key);
            int v = kd_insert3(
                kd_tree, node->position_.x(), 
                node->position_.y(), node->position_.z(),
                node);
        }

        struct kdres *neighbours;
        neighbours = kd_nearest_range3(
            kd_tree, state.transform.translation().x(), 
            state.transform.translation().y(), 
            state.transform.translation().z(),
            communication_radius);

        float communication_radius_sq_float = (float)(communication_radius * communication_radius);

        // clear agent neighbour before adding in new neighbours and obstacles
        it->second.clearAgentNeighbor();

        while (!kd_res_end(neighbours))
        {
            double pos[3];
            Eval_agent *agent = (Eval_agent*)kd_res_item(neighbours, pos);
            
            it->second.insertAgentNeighbor(*agent, communication_radius_sq_float);
            // store range query result so that we dont need to query again for rewire;
            kd_res_next(neighbours); // go to next in kd tree range query result
        }

        kd_free(kd_tree);
    }

    // add in the static obstacles as static neighbours
    // find the obstacles that lie on the plane first
    if (!orca_obstacle_map.obs.empty())
    {
        float communication_radius_sq_float = (float)(communication_radius * communication_radius);
        visibility_graph::global_map copy = orca_obstacle_map;
        copy.start_end.first = state.transform.translation();
        copy.t = visibility_graph::get_affine_transform(
            copy.start_end.first, Eigen::Vector3d(0.0, 0.0, 0.0), "nwu");

        // check all obstacles and add in edges that may have collision
        std::vector<visibility_graph::obstacle> rot_polygons;
        std::vector<Eigen::Vector3d> debug_point_vertices;
        visibility_graph::get_polygons_on_plane(
            copy, Eigen::Vector3d(0.0, 0.0, 1.0), 
            rot_polygons, debug_point_vertices, true);
        
        // TODO: check other directions also 
        // visibility_graph::get_polygons_on_plane(
        //     copy, Eigen::Vector3d(0.0, 0.0, 1.0), 
        //     rot_polygons, debug_point_vertices, true);

        // threshold is communication_radius
        for (auto &poly : rot_polygons)
        {
            for (size_t i = 0; i < poly.v.size(); i++)
            {
                size_t j = (i + 1) % (poly.v.size());
                double distance;
                Eigen::Vector2d closest_point;

                visibility_graph::get_point_to_line(
                    Eigen::Vector2d::Zero(), poly.v[i], poly.v[j],
                    distance, closest_point);
                if (distance < communication_radius)
                {
                    // distribute from poly.v[i] poly.v[j]
                    Eigen::Vector3d vi = 
                        copy.t.inverse() * Eigen::Vector3d(poly.v[i].x(), poly.v[i].y(), 0.0);
                    Eigen::Vector3d vj = 
                        copy.t.inverse() * Eigen::Vector3d(poly.v[j].x(), poly.v[j].y(), 0.0);
                    size_t div = (size_t)std::ceil((vj - vi).norm() / (protected_zone * 1.5));
                    double separation = (vj - vi).norm() / div;
                    Eigen::Vector3d dir = (vj - vi).normalized();

                    for (size_t j = 1; j < div-1; j++)
                    {
                        Eigen::Vector3d pos = vi + dir * j * separation;
                        
                        if ((pos - state.transform.translation()).norm() < communication_radius)
                        {
                            Eval_agent static_point;
                            static_point.position_ = pos.cast<float>();
                            static_point.velocity_ = Eigen::Vector3f::Zero();
                            static_point.radius_ = (float)protected_zone/3;
                            static_point.type_ = OBSTACLE; // static obstacle
                            static_point.id_ = 0;
                            it->second.insertAgentNeighbor(static_point, communication_radius_sq_float);
                        }
                        // std::cout << mykey << " obstacle_static " << static_point.position_.transpose() << std::endl;
                    }
                }
            }
        }
    }

    if (!it->second.noNeighbours())
    {
        // assign neighbors
        // const auto &tmp = it->second.getNeighbors();
        agent_update_mutex.lock();
        it->second.updateState(
            state.transform.translation().cast<float>(), 
            state.velocity.cast<float>(), 
            desired.cast<float>());
        agent_update_mutex.unlock();
        
        it->second.computeNewVelocity();
        Eigen::Vector3f new_desired = it->second.getVelocity();
        desired = new_desired.cast<double>();
    }
}

void cs2::cs2_application::handler_timer_callback() 
{
    AgentsStateFeedback agents_feedback;
    MarkerArray target_array;

    double rad_to_deg = 180.0 / M_PI;
    // RCLCPP_INFO(this->get_logger(),"handler_timer_callback");
    // Iterate through the agents
    for (auto &[key, agent] : agents_states)
    {
        rclcpp::Time now = this->get_clock()->now();
        //RCLCPP_INFO(this->get_logger(),"handler_timer_callback %s %ld",key.c_str(),agent.flight_state);
        switch (agent.flight_state)
        {
            case IDLE: case EMERGENCY:
            {
                // RCLCPP_INFO(this->get_logger(),"IDLE");
                break;
            }

            case HOVER: 
            {
                Eigen::Vector3d trans = agent.transform.translation();
                double pose_difference = 
                    (agent.previous_target - trans).norm();
                RCLCPP_INFO(this->get_logger(), "%s pose diff : %.3lf, prev_target : (%.3lf %.3lf %.3lf), agent : (%.3lf %.3lf %.3lf)", 
                    key.c_str(), pose_difference, agent.previous_target.x(), agent.previous_target.y(), agent.previous_target.z(), \
                    trans.x(), trans.y(), trans.z());
                Eigen::Vector3d vel_target;
                if (pose_difference < max_velocity)
                    vel_target = 
                        (agent.previous_target - agent.transform.translation()); 
                else
                    vel_target = 
                        (agent.previous_target - agent.transform.translation()).normalized() * max_velocity;
                RCLCPP_INFO(this->get_logger(), "%s hover before : %.3lf %.3lf %.3lf", 
                    key.c_str(), vel_target.x(), vel_target.y(), vel_target.z());
                // conduct_planning(vel_target, key, agent);
                // RCLCPP_INFO(this->get_logger(), "%s hover after : %.3lf %.3lf %.3lf",
                //     key.c_str(), vel_target.x(), vel_target.y(), vel_target.z());
                // VelocityWorld vel_msg;
                // vel_msg.header.stamp = clock.now();
                // vel_msg.vel.x = vel_target.x();
                // vel_msg.vel.y = vel_target.y();
                // vel_msg.vel.z = vel_target.z();
                // // vel_msg.height = agent.previous_target.z();
                // // vel_msg.yaw = agent.previous_yaw * rad_to_deg;
                // vel_msg.yaw_rate = 0.0;

                Hover hover_msg;
                hover_msg.header.stamp = clock.now();
                hover_msg.vx = 0;
                hover_msg.vy = 0;
                hover_msg.yaw_rate = 0;
                hover_msg.z_distance = trans.z();
                // hover_msg.z_distance = 1.0;


                // vel_msg.linear.x = trans.x() + vel_target.x() * 1/this->planning_rate;
                // vel_msg.linear.y = trans.y() + vel_target.y() * 1/this->planning_rate;
                // vel_msg.linear.z = trans.z() + vel_target.z() * 1/this->planning_rate;
                // Eigen::Vector3d rpy = 
                //         euler_rpy(agent.transform.linear());
                // vel_msg.linear.x = trans.x() + vel_target.x();
                // vel_msg.linear.y = trans.y() + vel_target.y();
                // vel_msg.linear.z = trans.z() + vel_target.z();
                // vel_msg.angular.z = rpy.z() * rad_to_deg;

                auto it = agents_comm.find(key);
                if (it != agents_comm.end()){
                    //it->second.vel_world_publisher->publish(vel_msg);
                    it->second.hover_world_publisher->publish(hover_msg);
                // agent.completed = false;
                }
                break;
            }
            case TAKEOFF: case LAND: case MOVE:
            {
                bool is_land = 
                    (agent.flight_state == LAND);

                if (agent.target_queue.empty())
                {
                    // we do not need to handle the velocity here since:
                    // cffirmware land service handles it for us
                    // after popping the takeoff/goto queue till it is empty, change state to hover
                    agent.flight_state = is_land ? IDLE : HOVER;
                    agent.completed = true;
                    break;
                }
                double pose_difference = 
                    (agent.target_queue.front() - agent.transform.translation()).norm();
                if (pose_difference < reached_threshold)
                {
                    agent.previous_target = agent.target_queue.front();
                    Eigen::Vector3d rpy = 
                        euler_rpy(agent.transform.linear());
                    agent.previous_yaw = rpy.z();
                    
                    agent_update_mutex.lock();
                    agent.target_queue.pop();
                    agent_update_mutex.unlock();
                }
                
                if (is_land)
                    agent.completed = true;

                break;
            }
            case MOVE_VELOCITY: case INTERNAL_TRACKING:
            {
                rclcpp::Time start = clock.now();

                if (agent.target_queue.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "MOVE VELOCITY no target");
                    // move velocity
                    if (agent.flight_state == MOVE_VELOCITY)
                    {
                        agent.flight_state = HOVER;
                        agent.completed = true;
                    }
                    // internal tracking
                    else
                    {
                        auto it = agents_comm.find(key);
                        if (it == agents_comm.end())
                            continue;

                        send_land_and_update(agents_states.find(key), it);                        
                        agent.completed = true;
                    }
                    
                    break;
                }

                Eigen::Vector3d trans = agent.transform.translation();
                double pose_difference = 
                    (agent.target_queue.front() - agent.transform.translation()).norm();
                Eigen::Vector3d goal = agent.target_queue.front();
                RCLCPP_INFO(this->get_logger(), "[Goto Velocity] %s pose diff : %.3lf, goal : (%.3lf %.3lf %.3lf), agent : (%.3lf %.3lf %.3lf)", 
                    key.c_str(), pose_difference, goal.x(), goal.y(), goal.z(), \
                    trans.x(), trans.y(), trans.z());
                //VelocityWorld vel_msg;
                //geometry_msgs::msg::Twist vel_msg;
                //Hover hover_msg;
                Eigen::Vector3d vel_target;
                // // vel_msg.height = agent.target_queue.front().z();

                if (pose_difference < reached_threshold)
                {
                    vel_target = Eigen::Vector3d::Zero();
                    agent.previous_target = agent.target_queue.front();
                    Eigen::Vector3d rpy = 
                        euler_rpy(agent.transform.linear());
                    agent.previous_yaw = rpy.z();

                    agent_update_mutex.lock();
                    agent.target_queue.pop();
                    agent_update_mutex.unlock();

                    break;
                }
                else if (pose_difference < max_velocity)
                    vel_target = 
                        (agent.target_queue.front() - agent.transform.translation()); 
                else
                {
                    vel_target = 
                        (agent.target_queue.front() - agent.transform.translation()).normalized() * max_velocity;
                }

                conduct_planning(vel_target, key, agent);

                double duration_seconds = (clock.now() - start).seconds();
                RCLCPP_INFO(this->get_logger(), "go_to_velocity %s (%.3lf %.3lf %.3lf) time (%.3lfms)", 
                    key.c_str(), vel_target.x(), vel_target.y(), vel_target.z(), duration_seconds * 1000.0);

                VelocityWorld vel_msg;

                vel_msg.header.stamp = clock.now();
                vel_msg.vel.x = vel_target.x();
                vel_msg.vel.y = vel_target.y();
                vel_msg.vel.z = vel_target.z();
                // vel_msg.yaw_rate = 0.0;
                
                // check the difference in heading
                Eigen::Vector3d rpy = 
                    euler_rpy(agent.transform.linear());
                
                double yaw_target;
                if (agent.target_yaw - rpy.z() * rad_to_deg < -180.0)
                    yaw_target = agent.target_yaw - (rpy.z() * rad_to_deg - 360.0);
                else if (agent.target_yaw - rpy.z() * rad_to_deg > 180.0)
                    yaw_target = agent.target_yaw - (rpy.z() * rad_to_deg + 360.0);
                else
                    yaw_target = agent.target_yaw - rpy.z() * rad_to_deg;

                // // std::cout << yaw_target << std::endl;
                double dir = yaw_target / std::abs(yaw_target);
                if (!std::isnan(dir))
                {
                    yaw_target = std::min(std::abs(yaw_target), maximum_yaw_change);                
                    yaw_target *= dir;
                    if (std::abs(yaw_target - agent.target_yaw) > maximum_yaw_change * 1.5)
                        yaw_target += rpy.z() * rad_to_deg;
                    else
                        yaw_target = agent.target_yaw;
                }
                else
                    yaw_target += rpy.z() * rad_to_deg;
                // vel_msg.angular.z = yaw_target;
                // std::cout << rpy.z() << "/" << wrap_pi(yaw_target) / rad_to_deg << std::endl;
                // vel_msg.yaw = wrap_pi(yaw_target);
                double yaw_rate_target;
                if (wrap_pi(yaw_target) > 1e-5)
                    yaw_rate_target = 0.15;
                else if (wrap_pi(yaw_target) < -1e-5)
                    yaw_rate_target = -0.15;
                else
                    yaw_rate_target = 0.0;
                vel_msg.yaw_rate = yaw_rate_target;
                
                auto it = agents_comm.find(key);
                if (it != agents_comm.end()){
                    it->second.vel_world_publisher->publish(vel_msg);
                    
                    //publish neighbor
                    auto rvo_it = rvo_agents.find(key);
                    float communication_radius_sq_float = (float)(communication_radius * communication_radius);
                    // only updated visibility is valid to make observation
                    rvo_it->second.updateVisibility(communication_radius_sq_float);
                    // auto cloud_msg = convert_cloud(rvo_it->second.getNeighbors());
                    // it->second.cloud_publisher->publish(std::move(cloud_msg));
                    // RCLCPP_INFO(this->get_logger(), "MOVE VELOCITY published");
                }
                break;
            }

            default:
                break;
            
        }

        std::string str_copy = key;
        // Remove cf_ from cf_XX
        int id = cs2::cs2_application::id_from_key(key);

        Marker target;
        target.header.frame_id = "/world";
        target.header.stamp = clock.now();
        target.type = visualization_msgs::msg::Marker::LINE_STRIP;
        target.id = id;
        target.action = visualization_msgs::msg::Marker::ADD;
        target.pose.orientation.x = 0.0;
        target.pose.orientation.y = 0.0;
        target.pose.orientation.z = 0.0;
        target.pose.orientation.w = 1.0;
        target.scale.x = 0.005;
        target.color.r = 0.5;
        target.color.g = 0.5;
        target.color.b = 1.0;
        target.color.a = 1.0;

        if(!agent.target_queue.empty())
        {
            // copy to prevent deleting the main target queue
            std::queue<Eigen::Vector3d> target_copy = agent.target_queue;

            geometry_msgs::msg::Point p;
            p.x = agent.transform.translation().x();
            p.y = agent.transform.translation().y();
            p.z = agent.transform.translation().z();
            target.points.push_back(p);

            while (!target_copy.empty())
            {
                geometry_msgs::msg::Point p;
                p.x = target_copy.front().x();
                p.y = target_copy.front().y();
                p.z = target_copy.front().z();
                target.points.push_back(p);

                target_copy.pop();
            }
        }

        AgentState agentstate;
        agentstate.id = key;
        agentstate.flight_state = agent.flight_state;
        agent.radio_connection = 
            ((clock.now() - agent.t).seconds() < radio_connection_timeout);
        agentstate.connected = agent.radio_connection;
        agentstate.completed = agent.completed;
        agentstate.mission_capable = agent.mission_capable;

        // make preObservation
        PreObservation preobs;
        preobs.agent = agentstate;

        if(agent.mission_capable){
            std::vector<bool> can_move;
            compute_can_move(key, agent, can_move);
            preobs.can_move = can_move;
            
            auto it_comm = agents_comm.find(key);
            if (it_comm != agents_comm.end()){
                auto move_marker = convert_move(agent.transform.translation(), can_move);
                it_comm->second.move_publisher->publish(std::move(move_marker));
            }

            auto rvo_it = rvo_agents.find(key);
            std::vector<Neighbor> vis_ally;
            std::vector<Neighbor> vis_enemy; 
            convert_neighbors(rvo_it->second, vis_ally, vis_enemy); 
            preobs.visible_ally = vis_ally;
            preobs.visible_enemy = vis_enemy;
        }else{
            preobs.can_move = std::vector<bool>({false, false, false, false, false, false});
            preobs.visible_ally = std::vector<Neighbor>();
            preobs.visible_enemy = std::vector<Neighbor>();
        }
        agents_feedback.pre_obs.push_back(preobs);
        target_array.markers.push_back(target);
    }

    // publish the flight state message
    agents_feedback.header.stamp = clock.now();
    agent_state_publisher->publish(agents_feedback);

    // publish the target data
    target_publisher->publish(target_array);
}

std::unique_ptr<PointCloud2> cs2::cs2_application::convert_cloud(
    const std::vector<std::pair<float, const Eval_agent>>& obstacles)
{
    auto cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud_msg->header.frame_id = "/world";    
    cloud_msg->header.stamp = clock.now();
    cloud_msg->height = 1;
    cloud_msg->width = obstacles.size();
    cloud_msg->is_dense = false;

    // Define fields x,y,z
    sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(obstacles.size());

    // Fill in data
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

    for (const auto& obs : obstacles) {
        *iter_x = obs.second.position_.x();
        *iter_y = obs.second.position_.y();
        *iter_z = obs.second.position_.z();
        ++iter_x; ++iter_y; ++iter_z;
    }
    return cloud_msg;
}   

std::unique_ptr<MarkerArray> cs2::cs2_application::convert_move(
    Eigen::Vector3d trans, const std::vector<bool>& can_move)
{
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();

    for (int i = 0; i < can_move.size(); i++){
        visualization_msgs::msg::Marker move_msg;
        move_msg.header.frame_id = "/world";
        move_msg.header.stamp = clock.now();
        move_msg.type = visualization_msgs::msg::Marker::ARROW;
        move_msg.action = visualization_msgs::msg::Marker::ADD;
        move_msg.id = i;

        Eigen::Vector3d dpos;
        switch(i) {
            case 0:
                dpos = Eigen::Vector3d(0, 1, 0);
                break;
            case 1:
                dpos = Eigen::Vector3d(0, -1, 0);
                break;
            case 2:
                dpos = Eigen::Vector3d(1, 0, 0);
                break;
            case 3:
                dpos = Eigen::Vector3d(-1, 0, 0);
                break;
            case 4:
                dpos = Eigen::Vector3d(0, 0, 1);
                break;
            case 5:
                dpos = Eigen::Vector3d(0, 0, -1);
                break;
        }
        dpos *= 0.5*0.55;
        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = trans.x();
        p_start.y = trans.y();
        p_start.z = trans.z();
        p_end.x = trans.x() + dpos.x();
        p_end.y = trans.y() + dpos.y();
        p_end.z = trans.z() + dpos.z();

        move_msg.points.push_back(p_start);
        move_msg.points.push_back(p_end);

        move_msg.color.a = 1.0;
        if (can_move[i]){
            move_msg.color.r = 0.0;
            move_msg.color.g = 0.0;
            move_msg.color.b = 1.0;
        }
        else{
            move_msg.color.r = 1.0;
            move_msg.color.g = 0.0;
            move_msg.color.b = 0.0;
        }

        marker_array->markers.push_back(move_msg);
    }

    return marker_array;
}   


void cs2::cs2_application::convert_neighbors(
    const Agent& agent, 
    std::vector<Neighbor>& vis_ally, 
    std::vector<Neighbor>& vis_enemy
)
{
    if (!agent.isneighborValid())
        return;

    for (const auto ally : agent.getNeighbors(ALLY)){
        if(ally.second.type_ == ALLY){
            Neighbor tmp_msg;
            tmp_msg.id = ally.second.id_;
            
            geometry_msgs::msg::Point pos;
            pos.x = ally.second.position_[0];
            pos.y = ally.second.position_[1];
            pos.z = ally.second.position_[2];
            tmp_msg.position = pos;
            
            geometry_msgs::msg::Point vel;
            vel.x = ally.second.velocity_[0];
            vel.y = ally.second.velocity_[1];
            vel.z = ally.second.velocity_[2];
            tmp_msg.velocity = vel;

            tmp_msg.radius = ally.second.radius_;
            tmp_msg.distance = ally.first;
            vis_ally.push_back(tmp_msg);
        }
        else{
            RCLCPP_INFO(this->get_logger(),"convert_neighbors: this should not happen");
        }
    }
    for (const auto enemy : agent.getNeighbors(ENEMY)){
        if(enemy.second.type_ == ENEMY){
            Neighbor tmp_msg;
            tmp_msg.id = enemy.second.id_;
            
            geometry_msgs::msg::Point pos;
            pos.x = enemy.second.position_[0];
            pos.y = enemy.second.position_[1];
            pos.z = enemy.second.position_[2];
            tmp_msg.position = pos;
            
            geometry_msgs::msg::Point vel;
            vel.x = enemy.second.velocity_[0];
            vel.y = enemy.second.velocity_[1];
            vel.z = enemy.second.velocity_[2];
            tmp_msg.velocity = vel;

            tmp_msg.radius = enemy.second.radius_;
            tmp_msg.distance = enemy.first;
            vis_ally.push_back(tmp_msg);
        }
        else{
            RCLCPP_INFO(this->get_logger(),"convert_neighbors: this should not happen");
        }
    }
}   

void cs2::cs2_application::compute_can_move(
    std::string mykey, 
    const agent_state& agent, 
    std::vector<bool> &result
)
{
    Eigen::Vector3d trans = agent.transform.translation();
    Eigen::Vector3d rpy = euler_rpy(agent.transform.linear()); //radian
    float yaw = rpy.z();
    float check_value = 0.5 * 0.55; //TODO: get pref_speed through parameter

    std::vector<Eigen::Vector3d> intersections;

    for(int i = 0; i<6; i++){
        Eigen::Vector3d dpos;
        switch(i) { 
            case 0: 
                dpos = Eigen::Vector3d(0, 1, 0);
                break;
            case 1:
                dpos = Eigen::Vector3d(0, -1, 0);
                break;
            case 2:
                dpos = Eigen::Vector3d(1, 0, 0);
                break;
            case 3:
                dpos = Eigen::Vector3d(-1, 0, 0);
                break;
            case 4:
                dpos = Eigen::Vector3d(0, 0, 1);
                break;
            case 5:
                dpos = Eigen::Vector3d(0, 0, -1);
                break;
        }
        // move position w.r.t. current yaw 
        Eigen::Vector3d dir = Eigen::Vector3d(
            dpos.x()*std::cos(yaw) - dpos.y()*std::sin(yaw),
            dpos.x()*std::sin(yaw) + dpos.y()*std::sin(yaw),
            dpos.z()
        );
        std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e;
        s_e.first = trans;
        s_e.second = trans + dir * check_value;
        
        visibility_graph::global_map copy = orca_obstacle_map;
        get_line_polygon_intersection(copy, s_e, intersections);
        std::vector<std::pair<std::string, float>> distance_list;
       
        if (!intersections.empty()){
            result.push_back(false); //can't move 
        }
        else{ 
            // find unit closest to the moving line
            for (auto &[key, other_agent] : agents_states){
                if (strcmp(mykey.c_str(), key.c_str()) == 0)
                    continue;
                if (!other_agent.mission_capable) // do not consider dead agent
                    continue;
            
                Eigen::Vector3d other_trans = other_agent.transform.translation();
                std::pair<std::string, float> tmp_pair;
                tmp_pair.first = key; 
                double dist_to_line; 
                if (!get_point_to_line_3d(other_trans, s_e.first, s_e.second, dist_to_line))
                    continue;
                tmp_pair.second = dist_to_line;
                distance_list.push_back(tmp_pair);
            }
            auto it = std::min_element(
                distance_list.begin(),
                distance_list.end(),
                [](const auto& lhs, const auto& rhs) {
                    return lhs.second < rhs.second;
                }
            );
            
            // if closest unit is closer than radius, can't move 
            if (it != distance_list.end()){
                // auto rvo_current = rvo_agents.find(mykey);
                // auto rvo_other = rvo_agents.find(it->first);
                // TODO: Add public getter for radius or make it accessible
                double combined_r = 0.275 + 0.275; // Default radius (protected zone)
                if (it->second < combined_r)
                    result.push_back(false);
            }
            result.push_back(true);
        }
    }
}

size_t cs2::cs2_application::do_laser_action(
    std::string mykey, const agent_state& agent,
    Eigen::Vector3d target_point
){
    Eigen::Vector3d trans = agent.transform.translation();
    std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e;
    s_e.first = trans;
    s_e.second = target_point;

    visibility_graph::global_map copy = orca_obstacle_map;
    std::vector<Eigen::Vector3d> intersections;
    get_line_polygon_intersection(copy, s_e, intersections);

    // find unit that is closer than the obstacle 

    double closest_obs_distance = communication_radius; 
    size_t result = 0;
    if (!intersections.empty()){
        for(auto &p : intersections){
            double tmp_dist = (p - trans).norm(); 
            if (tmp_dist < closest_obs_distance)
                closest_obs_distance = tmp_dist;
        }
    }
    // for visible ally and visible enemy, insert if it is closer than closest_obs_distance
    std::vector<std::pair<float, RVO::Eval_agent>> sorted_neighbor;
    auto rvo_current = rvo_agents.find(mykey);
    auto vis_ally = rvo_current->second.getNeighbors(ALLY);
    auto vis_enemy = rvo_current->second.getNeighbors(ENEMY);

    // if there exists neighbor
    if (!vis_ally.empty() || !vis_enemy.empty()){
        for (const auto& item : vis_ally) {
            if (item.first < closest_obs_distance) {
                sorted_neighbor.push_back(std::make_pair(item.first, item.second));
            }
        }

        for (const auto& item : vis_enemy) {
            if (item.first < closest_obs_distance) {
                sorted_neighbor.push_back(std::make_pair(item.first, item.second));
            }
        }

        std::sort(sorted_neighbor.begin(), sorted_neighbor.end(),
            [](const auto& lhs, const auto& rhs) {
                return lhs.first < rhs.first; // Sort by distance
            });

        // and close enough (hit radius) to the line segment
        for (auto &[dist, eval_agent]: sorted_neighbor){
            Eigen::Vector3d other_trans = eval_agent.position_.cast<double>();
            double dist_to_line; 
            if (!get_point_to_line_3d(other_trans, s_e.first, s_e.second, dist_to_line))
                continue;
            if (dist_to_line < 0.06){ // refer to URDF collision radius
                // TODO : provide hit radius as parameter
                result = eval_agent.id_;
                break;
            }
        }
    }
    return result;
}

void cs2::cs2_application::get_line_polygon_intersection(
    visibility_graph::global_map g_m, std::pair<Eigen::Vector3d, Eigen::Vector3d> s_e, std::vector<Eigen::Vector3d>& intersections
)
{   
    for (visibility_graph::obstacle &obs : g_m.obs){
        // for each obs as a polygon, get_line_plane_intersection
        int obs_vert_size = obs.v.size();
        // doesn't matter if it's concave obstacle 

        // bottom 
        Eigen::Vector3d tmp_pop = Eigen::Vector3d(obs.c[0], obs.c[1], obs.h.first);
        Eigen::Vector3d tmp_n = Eigen::Vector3d(0,0,-1);
        Eigen::Vector3d result; 
        if(visibility_graph::get_line_plane_intersection(s_e, tmp_n, tmp_pop, result)){
            intersections.push_back(result);
        }
        // top
        tmp_pop = Eigen::Vector3d(obs.c[0], obs.c[1], obs.h.second);
        tmp_n = Eigen::Vector3d(0,0,1);
        if(visibility_graph::get_line_plane_intersection(s_e, tmp_n, tmp_pop, result)){
            intersections.push_back(result);
        }
        
        // base vertices
        for (int i = 0; i < obs_vert_size; i++){
            i = i % obs_vert_size;
            Eigen::Vector3d tmp_pop = Eigen::Vector3d(obs.v[i].x(), obs.v[i].y(), obs.h.first);
            Eigen::Vector3d tmp_n = Eigen::Vector3d(obs.v[i+1].y() - obs.v[i].y(), - obs.v[i+1].x() + obs.v[i].x(), 0);
            Eigen::Vector3d result; 
            if(visibility_graph::get_line_plane_intersection(s_e, tmp_n, tmp_pop, result)){
                intersections.push_back(result);
            }
        }
    }
}

bool cs2::cs2_application::get_point_to_line_3d(
        Eigen::Vector3d p, Eigen::Vector3d start, Eigen::Vector3d end,
        double &distance){
    Eigen::Vector3d tmp1 = p - start; 
    Eigen::Vector3d tmp2 = end - start; 
    Eigen::Vector3d cross = tmp1.cross(tmp2);
    double epsilon = 0.0001;
    
    double t = tmp1.dot(tmp2) / tmp2.dot(tmp2);
    if (t < 0.0 || t > 1.0)
        return false;
    Eigen::Vector3d proj = start + t * tmp2; 

    if(tmp2.norm() < epsilon) 
        return false;
    else {
        distance = (p - proj).norm();
    }
    return true;
}
