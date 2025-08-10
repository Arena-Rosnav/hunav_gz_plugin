/***********************************************************************/
/**                                                                    */
/** HuNavSystemPlugin_fortress.cpp                                     */
/**                                                                    */
/** Copyright (c) 2025, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

#include <stdio.h>
#include <string>
#include <chrono>
#include <stdexcept>
#include <sstream>
#include <cmath>

#include <hunav_gz_plugin/HuNavSystemPlugin.h>
#include <gz/plugin/Register.hh>
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>

// ADD ARENA_PEOPLE_MSGS INCLUDE
#include <arena_people_msgs/msg/pedestrians.hpp>
#include <arena_people_msgs/msg/pedestrian.hpp>

// Fix for chrono duration traits error
namespace gz::sim::traits {
  template<>
  struct HasEqualityOperator<std::chrono::steady_clock::duration> {
    static constexpr bool value = true;
  };
}

using namespace std::chrono_literals;

/////////////////////////////////////////////////
HuNavSystemPluginIGN::HuNavSystemPluginIGN()
{
   walls_loaded_ = false;  
}

HuNavSystemPluginIGN::~HuNavSystemPluginIGN()
{
}

/////////////////////////////////////////////////
void HuNavSystemPluginIGN::Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                            gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr)
{
  (void)_eventMgr;
  counter_ = 0;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  std::string nodename = "hunav_plugin_node_" + std::to_string(_entity);
  this->rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str());
  
  worldEntity_ = _entity; 

  // Get plugin parameters
  sdf_ = _sdf->Clone();

  if (sdf_->HasElement("namespace"))
    namespace_ = sdf_->Get<std::string>("namespace");
  else
    namespace_ = "/";

  if(namespace_.empty() || namespace_.back() != '/')
    namespace_.push_back('/');

  if (sdf_->HasElement("global_frame_to_publish"))
    globalFrame_ = sdf_->Get<std::string>("global_frame_to_publish");
  else
    globalFrame_ = "map";

  if (sdf_->HasElement("use_navgoal_to_start"))
    waitForGoal_ = sdf_->Get<bool>("use_navgoal_to_start");
  else
  {
    waitForGoal_ = false;
    gzwarn << "PARAMETER USE_NAVGOAL_TO_START IS NOT IN THE WORLD FILE!!" << std::endl;
  }

  if (waitForGoal_)
  {
    goalReceived_ = false;
    if (sdf_->HasElement("navgoal_topic"))
      goalTopic_ = sdf_->Get<std::string>("navgoal_topic");
    else
      goalTopic_ = "goal_pose";
  }
  else
  {
    goalReceived_ = true;
  }

  // Read models to be ignored
  if (sdf_->HasElement("ignore_models"))
  {
    sdf::ElementPtr modelElem = sdf_->GetElement("ignore_models")->GetElement("model");
    while (modelElem)
    {
      ignoreModels_.push_back(modelElem->Get<std::string>());
      gzmsg << "Ignoring model: " << (modelElem->Get<std::string>()) << std::endl;
      modelElem = modelElem->GetNextElement("model");
    }
  }
  reset_ = false;
  
  // Update rate
  auto update_rate = sdf_->Get<double>("update_rate", 1000.0).first;
  if (update_rate > 0.0)
  {
    update_rate_secs_ = 1.0 / update_rate;
  }
  else
  {
    update_rate_secs_ = 0.0;
  }
  gzmsg << "update_rate: " << update_rate << ", secs: " << update_rate_secs_ << std::endl;

  // Subscribe to arena_peds topic (EXACTLY like HumanSystemPlugin!)
  std::string full_topic = namespace_ + "arena_peds";
  
  static auto pedestrians_sub = rosnode_->create_subscription<arena_people_msgs::msg::Pedestrians>(
    full_topic,
    rclcpp::QoS(10),
    [this](const arena_people_msgs::msg::Pedestrians::SharedPtr msg) {
      // Store current pedestrians data (EXACTLY like HumanSystemPlugin!)
      current_pedestrians_ = msg;
      RCLCPP_DEBUG(rosnode_->get_logger(), "Received %zu pedestrians update", msg->pedestrians.size());
    });
  
  RCLCPP_INFO(this->rosnode_->get_logger(), "Subscribed to %s", full_topic.c_str());

  RCLCPP_INFO(this->rosnode_->get_logger(), "Creating delete_actors service...");
  delete_actors_service_ = this->rosnode_->create_service<arena_people_msgs::srv::DeleteActors>(
    namespace_ + "delete_actors",
    std::bind(&HuNavSystemPluginIGN::deleteActorsCallback, this, std::placeholders::_1, std::placeholders::_2)
);
  
  rosSrvResetClient_ = this->rosnode_->create_client<hunav_msgs::srv::ResetAgents>(namespace_ + "reset_agents");
  
  //Service to get the Wall Segments for the obstacle avoidance of the Pedestrians
  wall_client_ = this->rosnode_->create_client<hunav_msgs::srv::GetWalls>(namespace_ + "get_walls");

  //Initialize Wall Segments
  try
  {
    loadWallData();
  } 
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Failed to load wall data: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  gzmsg << "Plugin HuNavPluginIGN configured!" << std::endl;
}

void HuNavSystemPluginIGN::loadWallData()
{
    if (!wall_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(rosnode_->get_logger(), "GetWalls service not available - walls will not be avoided");
        walls_loaded_ = false;
        return;
    }

    RCLCPP_INFO(rosnode_->get_logger(), "GetWalls service available. Loading wall data...");

    auto request = std::make_shared<hunav_msgs::srv::GetWalls::Request>();
    auto future = wall_client_->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(rosnode_, future, std::chrono::seconds(5)) == 
        rclcpp::FutureReturnCode::SUCCESS) {
        
        auto response = future.get();
        cached_wall_segments_.clear();
        
        for (const auto& wall_msg : response->walls) {
            cached_wall_segments_.emplace_back(
                wall_msg.id,
                gz::math::Vector3d(wall_msg.start.x, wall_msg.start.y, wall_msg.start.z),
                gz::math::Vector3d(wall_msg.end.x, wall_msg.end.y, wall_msg.end.z),
                wall_msg.length,
                wall_msg.height
            );
        }
        
        walls_loaded_ = true;
        RCLCPP_INFO(rosnode_->get_logger(), "Loaded %zu wall segments for obstacle avoidance", 
                    cached_wall_segments_.size());

        for (const auto& segment : cached_wall_segments_) {
            RCLCPP_INFO(rosnode_->get_logger(), 
                        "Wall %d: [%.1f,%.1f]->[%.1f,%.1f] L=%.1fm H=%.1fm",
                        segment.id, segment.start.X(), segment.start.Y(),
                        segment.end.X(), segment.end.Y(), segment.length, segment.height);
        }
    } else {
        RCLCPP_ERROR(rosnode_->get_logger(), "Failed to load wall data from service");
    }
}

/////////////////////////////////////////////////
/**
 * @brief Get the closest obstacle for each pedestrian using arena_peds data
 * @param _ecm EntityComponentManager
 */
void HuNavSystemPluginIGN::getObstaclesFromArenaPeds(const gz::sim::EntityComponentManager& _ecm)
{
  if (!current_pedestrians_ || current_pedestrians_->pedestrians.empty()) {
    RCLCPP_DEBUG(rosnode_->get_logger(), "No arena_peds data available yet");
    return;
  }

  // Use current_pedestrians_ data (EXACTLY like HumanSystemPlugin!)
  for (const auto& ped : current_pedestrians_->pedestrians)
  {
    double minDist = 7.0;
    
    // Create agent position from arena_peds data
    gz::math::Vector3d actor_pos(ped.position.position.x, ped.position.position.y, ped.position.position.z);
    
    // Clear previous obstacles for this pedestrian
    obstacle_data_[ped.name].clear();

    // ADD WALL SEGMENTS AS VIRTUAL OBSTACLES 
    if (walls_loaded_ && !cached_wall_segments_.empty())
    {
      if(firstObstaclePrint_)
        RCLCPP_INFO(this->rosnode_->get_logger(), "Processing %zu wall segments as obstacles for %s", 
                    cached_wall_segments_.size(), ped.name.c_str());

      double wall_thickness = 0.3;
      for (const auto& wall_segment : cached_wall_segments_)
      {
        gz::math::Vector3d wall_direction = wall_segment.end - wall_segment.start;
        wall_direction.Normalize();
        
        gz::math::Vector3d wall_size;
        if (std::abs(wall_direction.X()) > std::abs(wall_direction.Y())) {
          wall_size = gz::math::Vector3d(wall_segment.length, wall_thickness, wall_segment.height);
        } else {
          wall_size = gz::math::Vector3d(wall_thickness, wall_segment.length, wall_segment.height);
        }
        
        gz::math::AxisAlignedBox wall_bb = gz::math::AxisAlignedBox(
          wall_segment.center - wall_size / 2,
          wall_segment.center + wall_size / 2
        );
        
        auto closest_point = ClosestPointOnBox(actor_pos, wall_bb);
        double distance = (closest_point - actor_pos).Length();
        
        if(distance < minDist)
        {
          geometry_msgs::msg::Point point;
          point.x = closest_point.X();
          point.y = closest_point.Y();
          point.z = closest_point.Z();
          obstacle_data_[ped.name].push_back(point);
        }
      }
    }

    // Add other entities as obstacles (same logic as before)
    std::unordered_map<gz::sim::Entity, gz::math::Pose3d> obs_entities;
    _ecm.Each<gz::sim::components::Pose>([&](const gz::sim::Entity& _entity, const gz::sim::components::Pose* _enty_pose)
    {
      (void)_enty_pose;
      auto enty_name = _ecm.Component<gz::sim::components::Name>(_entity);
      if(enty_name && std::find(ignoreModels_.begin(), ignoreModels_.end(), enty_name->Data()) == ignoreModels_.end())
      {
        // Don't consider self or other pedestrians
        if (enty_name->Data() != ped.name)
        {
          auto wp = worldPose(_entity, _ecm);
          obs_entities[_entity] = wp;
        }
      }
      return true;
    });

    // Process obstacles (same logic as before)
    for (const auto& [obs_enty, obs_pose] : obs_entities)
    {
      auto geometry = _ecm.Component<gz::sim::components::Geometry>(obs_enty);
      if (geometry)
      {
        gz::math::AxisAlignedBox obs_bb;
        if (geometry->Data().Type() == sdf::GeometryType::BOX)
        {
          auto box = geometry->Data().BoxShape();
          obs_bb = gz::math::AxisAlignedBox(
            obs_pose.Pos() - box->Size() / 2,
            obs_pose.Pos() + box->Size() / 2
          );
        }
        else if (geometry->Data().Type() == sdf::GeometryType::SPHERE)
        {
          auto sphere = geometry->Data().SphereShape();
          double radius = sphere->Radius();
          obs_bb = gz::math::AxisAlignedBox(
            obs_pose.Pos() - gz::math::Vector3d(radius, radius, radius),
            obs_pose.Pos() + gz::math::Vector3d(radius, radius, radius)
          );
        }
        else if (geometry->Data().Type() == sdf::GeometryType::CYLINDER)
        {
          auto cylinder = geometry->Data().CylinderShape();
          double radius = cylinder->Radius();
          double height = cylinder->Length();
          obs_bb = gz::math::AxisAlignedBox(
            obs_pose.Pos() - gz::math::Vector3d(radius, radius, height / 2),
            obs_pose.Pos() + gz::math::Vector3d(radius, radius, height / 2)
          );
        }
        else
        {
          continue;
        }
        
        auto closest_point = ClosestPointOnBox(actor_pos, obs_bb);
        double distance = (closest_point - actor_pos).Length();
        if(distance < minDist)
        {
          geometry_msgs::msg::Point point;
          point.x = closest_point.X();
          point.y = closest_point.Y();
          point.z = 0.0;
          obstacle_data_[ped.name].push_back(point);
        }
      }
    }
  }
  firstObstaclePrint_ = false;
}

bool HuNavSystemPluginIGN::callResetAgentsService()
{
  if (!rosSrvResetClient_) {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Reset agents service client not available");
    return false;
  }
  
  try {
    if (!rosSrvResetClient_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->rosnode_->get_logger(), "Reset agents service not available within timeout");
      return false;
    }
    
    auto request = std::make_shared<hunav_msgs::srv::ResetAgents::Request>();
    
    request->robot.id = 0;
    request->robot.name = "jackal";
    request->robot.type = hunav_msgs::msg::Agent::ROBOT;
    
    request->current_agents.header.stamp = rosnode_->get_clock()->now();
    request->current_agents.header.frame_id = "map";
    request->current_agents.agents.clear();
    
    RCLCPP_INFO(this->rosnode_->get_logger(), "Calling HuNav ResetAgents service...");
    
    auto future = rosSrvResetClient_->async_send_request(request);
    auto status = future.wait_for(std::chrono::seconds(5));
    
    if (status == std::future_status::ready) {
      auto response = future.get();
      if (response && response->ok) {
        RCLCPP_ERROR(this->rosnode_->get_logger(), "HuNav ResetAgents service successful");
        return true;
      } else {
        RCLCPP_ERROR(this->rosnode_->get_logger(), "HuNav ResetAgents service failed");
        return false;
      }
    } else {
      RCLCPP_ERROR(this->rosnode_->get_logger(), "HuNav ResetAgents service timeout");
      return false;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Error calling ResetAgents service: %s", e.what());
    return false;
  }
}

void HuNavSystemPluginIGN::deleteActorsCallback(
    const std::shared_ptr<arena_people_msgs::srv::DeleteActors::Request> request,
    std::shared_ptr<arena_people_msgs::srv::DeleteActors::Response> response)
{
    (void)request;
    
    RCLCPP_INFO(this->rosnode_->get_logger(), "=== DELETE ACTORS CALLBACK CALLED ===");
    
    obstacle_data_.clear();
    
    response->success = true;
    response->deleted_count = 0;
    
    RCLCPP_INFO(this->rosnode_->get_logger(), "Cleared obstacle data");
}

/////////////////////////////////////////////////
void HuNavSystemPluginIGN::PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm)
{
  // Spin ROS node to process callbacks (LIKE HumanSystemPlugin!)
  rclcpp::spin_some(rosnode_);
  
  counter_++;

  if (_info.paused) {
    if(counter_ == 1000)
    {
      gzmsg << "Simulation paused" << std::endl;
      counter_ = 0;
    }
    return;
  }

  // Check if we have arena_peds data (LIKE HumanSystemPlugin!)
  if (!current_pedestrians_ || current_pedestrians_->pedestrians.empty()) {
    RCLCPP_DEBUG(rosnode_->get_logger(), "No arena_peds data available yet");
    return;
  }

  // Time delta
  std::chrono::duration<double> dtDuration = _info.simTime - lastUpdate_;
  double dt = dtDuration.count();

  lastUpdate_ = _info.simTime;
  rosLastUpdate_ = this->rosnode_->get_clock()->now();

  // OBSTACLE DETECTION from arena_peds data
  getObstaclesFromArenaPeds(_ecm);

  // CREATE PUBLISHER AND PUBLISH
  static auto obstacle_pub = this->rosnode_->create_publisher<hunav_msgs::msg::Agents>(
      namespace_ + "hunav_closest_obstacles", 10);

  hunav_msgs::msg::Agents obstacle_data;
  obstacle_data.header.frame_id = globalFrame_;
  obstacle_data.header.stamp = this->rosnode_->get_clock()->now();
  
  std::vector<hunav_msgs::msg::Agent> obstacle_agents;
  
  if (current_pedestrians_ && !current_pedestrians_->pedestrians.empty()) {
    for (const auto& ped : current_pedestrians_->pedestrians) {
      hunav_msgs::msg::Agent obs_agent;
      obs_agent.name = ped.name;
      
      if (obstacle_data_.find(ped.name) != obstacle_data_.end()) {
        obs_agent.closest_obs = obstacle_data_[ped.name];
      } else {
        obs_agent.closest_obs.clear();
      }
      
      obstacle_agents.push_back(obs_agent);
    }
  }
  
  obstacle_data.agents = obstacle_agents;
  obstacle_pub->publish(obstacle_data);
  
  RCLCPP_DEBUG(rosnode_->get_logger(), "Published obstacles for %zu agents", obstacle_agents.size());
}

GZ_ADD_PLUGIN(HuNavSystemPluginIGN, gz::sim::System, HuNavSystemPluginIGN::ISystemConfigure, HuNavSystemPluginIGN::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(HuNavSystemPluginIGN, "HuNavSystemPluginIGN")