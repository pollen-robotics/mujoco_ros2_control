// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

#include <algorithm>
#include <chrono>
#include <vector>
// TODO merge with inner mujoco ros2 control

// MuJoCo data structures - only used for joint/sensor mapping, not simulation
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(
    node->get_logger(), "Initializing mujoco_ros2_control node (WebSocket mode)...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // Load model only for joint/sensor information (not for simulation)
  // The actual simulation runs on the external MuJoCo server via WebSocket
  char error[1000] = "Could not load binary model";
  if (
    std::strlen(model_path.c_str()) > 4 &&
    !std::strcmp(model_path.c_str() + std::strlen(model_path.c_str()) - 4, ".mjb"))
  {
    mujoco_model = mj_loadModel(model_path.c_str(), 0);
  }
  else
  {
    mujoco_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
  }
  if (!mujoco_model)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to load model for joint mapping: %s", error);
    return -1;
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(), "MuJoCo model loaded for joint/sensor mapping (not simulation)");

  // Create dummy data (not used for actual simulation)
  mujoco_data = mj_makeData(mujoco_model);

  // Initialize mujoco control (will connect to WebSocket)
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(node->get_logger(), "MuJoCo ROS2 controller initialized in WebSocket mode!");

  // NOTE: No rendering or camera initialization since we're not running the MuJoCo engine
  // The external MuJoCo server handles visualization and cameras
  RCLCPP_INFO_STREAM(node->get_logger(), "Rendering and cameras handled by external MuJoCo server");
  RCLCPP_INFO_STREAM(node->get_logger(), "Let's toto this");

  // Timing variables for control loop performance monitoring
  std::vector<double> control_times;
  auto last_stats_time = std::chrono::steady_clock::now();

  // Frequency control variables using real time
  auto last_control_update = std::chrono::steady_clock::now();
  // constexpr double CONTROL_FREQ = 1000.0;
  auto control_period = mujoco_control.get_control_period();
  // log control period
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Control period: " << control_period.seconds() << " seconds");

  double control_period_seconds = control_period.seconds();
  // Main control loop - only handle ROS2 control updates
  while (rclcpp::ok())
  {
    auto now = std::chrono::steady_clock::now();

    if (std::chrono::duration<double>(now - last_control_update).count() >= control_period_seconds)
    {
      // RCLCPP_INFO_STREAM(node->get_logger(), "Going to do a control update");

      auto control_start = std::chrono::steady_clock::now();
      mujoco_control.update();
      auto control_end = std::chrono::steady_clock::now();

      double control_time_ms =
        std::chrono::duration<double, std::milli>(control_end - control_start).count();
      control_times.push_back(control_time_ms);

      last_control_update = now;
      // RCLCPP_INFO_STREAM(node->get_logger(), "Control update done");

      // Log control statistics every 60 seconds
      auto current_time = now;
      auto time_since_last_stats =
        std::chrono::duration<double>(current_time - last_stats_time).count();

      if (time_since_last_stats >= 60.0)
      {
        RCLCPP_INFO(
          node->get_logger(), "=== Control Performance Stats (60s window, WebSocket mode) ===");

        // Control stats
        if (!control_times.empty())
        {
          double min_time = *std::min_element(control_times.begin(), control_times.end());
          double max_time = *std::max_element(control_times.begin(), control_times.end());
          double sum = std::accumulate(control_times.begin(), control_times.end(), 0.0);
          double mean_time = sum / control_times.size();
          double hz = control_times.size() / time_since_last_stats;

          RCLCPP_INFO(
            node->get_logger(),
            "Control    | Updates:%4zu | Hz: %6.1f | Mean: %5.2fms | Min: %5.2fms | Max: %5.2fms",
            control_times.size(), hz, mean_time, min_time, max_time);
        }

        RCLCPP_INFO(
          node->get_logger(),
          "WebSocket mode: Rendering and cameras handled by external MuJoCo server");
        RCLCPP_INFO(
          node->get_logger(), "================================================================");

        // Reset for next window
        control_times.clear();
        last_stats_time = current_time;
      }

      // RCLCPP_INFO_STREAM(node->get_logger(), "Yet another loop iteration");
    }
    // Small sleep to prevent CPU spinning
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // Cleanup - only delete model/data used for mapping
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  RCLCPP_INFO_STREAM(
    node->get_logger(), "MuJoCo ROS2 control node (WebSocket mode) shutdown complete");

  return 0;
}