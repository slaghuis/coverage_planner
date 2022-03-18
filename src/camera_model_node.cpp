// Copyright 2022 Xeni Robotics Foundation.
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

/*******************************************************************
 * ROS2 Service to calcuate the maximum flight height to achieve a
 * required spatial resolution on images taken with a camera of given 
 * resolution and fixed optics
 *******************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <memory>
#include <math.h>

#include "coverage_planner_interfaces/srv/camera_model.hpp"

using CameraModel = coverage_planner_interfaces::srv::CameraModel;
using namespace std::chrono_literals;

class CameraModelClass: public rclcpp::Node
{
  public:
    CameraModelClass()
      : Node("camera_model_node")
    {
      this->declare_parameter<float>("angle_of_view", 1.08559479);   // 62.2 degrees in radians
      this->declare_parameter<int>("image_resolution_x", 1920);     // image width (pixels)
      this->declare_parameter<int>("image_resolution_y", 1080);     // image height (pixels)
      
      service_ = create_service<CameraModel>("coverage_planner/camera_model", 
        std::bind(&CameraModelClass::handle_service, this, std::placeholders::_1, std::placeholders::_2));       
    }
    
  private:
    std::string parameter_string_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<CameraModel>::SharedPtr service_;
    
    void handle_service(
      const std::shared_ptr<CameraModel::Request> request,
      const std::shared_ptr<CameraModel::Response> response)
    {  
      float angle_of_view;
      int resolution_x;
      int resolution_y;
    
      this->get_parameter("angle_of_view", angle_of_view);
      this->get_parameter("image_resolution_x", resolution_x);
      this->get_parameter("image_resolution_y", resolution_y);
    
      float rd = request->spatial_resolution;
      float h = resolution_x / ( 2 * rd * tan( (float) angle_of_view / 2));
    
      response->max_height = h;
      response->projected_area_w =  2*h * tan( (float) angle_of_view/2);
      response->projected_area_h = response->projected_area_w / ( (float) resolution_x / (float) resolution_y);
    }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraModelClass>());
  rclcpp::shutdown();
  return 0;
}