// Copyright (c) 2022 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************
 * A ROS2 Action Server that takes as an input a vector of poses, and using 
 * the navigation stack, flies the drone to each waypoint, stops and corrects
 * the yaw and takes an image.  
 ***************************************************************************/
#include <chrono>
#include <functional>
#include <memory>
#include <future>
#include <thread>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "coverage_planner_interfaces/action/execute_along_path.hpp"
#include "navigation_interfaces/action/navigate_to_pose.hpp"
#include "camera_lite_interfaces/srv/save.hpp"

#define FILENAME_FORMAT "%Y%m%d_%H%M%S.jpg"
#define FILENAME_SIZE 20

enum class ActionStatus {VIRGIN, REJECTED, PROCESSING, SUCCEEDED, ABORTED, CANCELED, UNKNOWN};

class ExecutePathActionServer : public rclcpp::Node
{
public:
  using ExecuteAlongPath = coverage_planner_interfaces::action::ExecuteAlongPath;
  using GoalHandleExecuteAlongPath = rclcpp_action::ServerGoalHandle<ExecuteAlongPath>;
  
  using NavigateToPose = navigation_interfaces::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit ExecutePathActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("execute_path_server", options)
  {
    using namespace std::placeholders;
    
    // Declare some parameters
    this->declare_parameter<std::string>("images_folder", "./");
    
    this->save_client_ptr_ = this->create_client<camera_lite_interfaces::srv::Save>("camera/save_picture"); 
    
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "nav_lite/navigate_to_pose");

    this->action_server_ = rclcpp_action::create_server<ExecuteAlongPath>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "coverage_server/photogrammetry",
      std::bind(&ExecutePathActionServer::handle_goal, this, _1, _2),
      std::bind(&ExecutePathActionServer::handle_cancel, this, _1),
      std::bind(&ExecutePathActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<ExecuteAlongPath>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::Client<camera_lite_interfaces::srv::Save>::SharedPtr save_client_ptr_;
  bool goal_done_;
  std::atomic_bool _halt_requested;
  ActionStatus action_status;
  
  // Save Picture Service Client ///////////////////////////////////////////////////////////////////////////////////////////////
  std::string generate_filenane() 
  { 
    // Read a path where the images must be stored, and validate
    std::string save_path;
    this->get_parameter("images_folder", save_path);
    if ( !(save_path.back() == '/') ) {
      save_path += '/';
    }
    
    // generate a file name
    static char name[FILENAME_SIZE];
    time_t now = time(0);
    strftime(name, sizeof(name), FILENAME_FORMAT, localtime(&now));
    RCLCPP_DEBUG(this->get_logger(), "Generated name : %s", name);
    
    
    return save_path + name;
  }
  
  bool take_picture(std::string filename)
  {
    using namespace std::chrono_literals;
    
    while (!save_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<camera_lite_interfaces::srv::Save::Request>();
    request->name = filename;
    auto result_future = save_client_ptr_->async_send_request(request);
    
    std::future_status status;
    do {
      status = result_future.wait_for(250ms);  // Not spinning here!  We are in a thread, and the spinning is taken cared of elsewhere.
      if (status == std::future_status::deferred) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future deferred");
      } else if (status == std::future_status::timeout) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future timeout");
      } else if (status == std::future_status::ready) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Future ready!");
      }  
    } while ((status != std::future_status::ready) && (!_halt_requested)); 
   
    bool service_result = false;
    if (status == std::future_status::ready)
    {
      auto result = result_future.get();
      service_result = result->result; 
    }
    
    return service_result;
  
  }

  // Navigate to Pose Service Methods ///////////////////////////////////////////////////////////////////////////////////////////////////
  
  bool fly_to_waypoint(geometry_msgs::msg::Pose pose, std::string behavior_tree)
  {
    rclcpp::Rate loop_rate(5);   //Set the frequency at 5Hz
    using namespace std::placeholders;
    
    action_status = ActionStatus::VIRGIN;
    
    this->goal_done_ = false;

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return false;
    }

    auto goal_msg = NavigateToPose::Goal();  //navigation_interfaces::action::NavigateToPose::Goal
    goal_msg.pose.pose.position.x = pose.position.x; 
    goal_msg.pose.pose.position.y = pose.position.y;
    goal_msg.pose.pose.position.z = pose.position.z;
  
    goal_msg.pose.pose.orientation.x = pose.orientation.x;
    goal_msg.pose.pose.orientation.y = pose.orientation.y;
    goal_msg.pose.pose.orientation.z = pose.orientation.z;
    goal_msg.pose.pose.orientation.w = pose.orientation.w;
    
    goal_msg.behavior_tree = behavior_tree;
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      std::bind(&ExecutePathActionServer::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ExecutePathActionServer::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ExecutePathActionServer::result_callback, this, _1);
    
    auto future_goal_handle_ = std::make_shared<
      std::shared_future<GoalHandleNavigateToPose::SharedPtr>>(
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options));

    _halt_requested.store(false);
    while (!_halt_requested && ((action_status == ActionStatus::VIRGIN) || (action_status == ActionStatus::PROCESSING)))
    {   
      loop_rate.sleep();
    }  
  
    return (action_status == ActionStatus::SUCCEEDED);
  }

  void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Photogtammetry goal was rejected by server");
      action_status = ActionStatus::REJECTED;
    } else {
      RCLCPP_INFO(this->get_logger(), "Photogrammetry goal accepted by server, waiting for result");
      action_status = ActionStatus::PROCESSING;
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Distance remaining %.2f",
      feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        action_status = ActionStatus::SUCCEEDED;
        RCLCPP_DEBUG(this->get_logger(), "Navigation action successfull.");  
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        action_status = ActionStatus::ABORTED;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        action_status = ActionStatus::CANCELED;
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        action_status = ActionStatus::UNKNOWN;
        return;
    }

  }

  // Action Server Methods ///////////////////////////////////////////////////////////////////////////////////////
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteAlongPath::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request to execute over %d waypoints", goal->path.poses.size());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteAlongPath> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    _halt_requested.store(true);
    
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  
  void execute(const std::shared_ptr<GoalHandleExecuteAlongPath> goal_handle)
  {
    // RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ExecuteAlongPath::Feedback>();
    auto result = std::make_shared<ExecuteAlongPath::Result>();
    auto & missed_waypoints = result->missed_waypoints;
    
    auto start_time = this->now();

    for (size_t i = 1; (i < goal->path.poses.size()) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->flight_time = this->now() - start_time;
        // Mission not complete.  Add current and all subsequent waypoints to missed_waypoint vector.
        for(size_t j = i; j<goal->path.poses.size(); j++) {
          missed_waypoints.push_back( (int)j );
        }  
        // result->missed_waypoints = missed_waypoints;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      
      // Fly to a waypoint.  
      if(fly_to_waypoint(goal->path.poses[i], goal->behavior_tree)) {      
        // BUT what if i never arrived at the waypoint?  Navigation could have failed?
        // if successfull, take a picture, geotag the image
        take_picture(generate_filenane());
        
      } else {  
        // If unsuccessfull,
        missed_waypoints.push_back(i);
      }
      
      // Publish feedback
      feedback->current_waypoint = i;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback: waypoint %d", i);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->flight_time = this->now() - start_time;
      result->missed_waypoints = missed_waypoints;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteAlongPath> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&ExecutePathActionServer::execute, this, _1), goal_handle}.detach();
  }
  
  // General Functions /////////////////////////////////////////////////////////////////////////////
  bool is_goal_done() const
  {
    return this->goal_done_;
  }
  
};  // class ExecutePathActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ExecutePathActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
