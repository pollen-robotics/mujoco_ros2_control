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

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

// MuJoCo data structures
mjModel *mujoco_model = nullptr;
mjData *mujoco_data = nullptr;

// main function
int main(int argc, const char **argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "mujoco_ros2_control_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_INFO_STREAM(node->get_logger(), "Initializing mujoco_ros2_control node...");
  auto model_path = node->get_parameter("mujoco_model_path").as_string();

  // load and compile model
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
    mju_error("Load model error: %s", error);
  }

  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco model has been successfully loaded !");
  // make data
  mujoco_data = mj_makeData(mujoco_model);

  // initialize mujoco control
  auto mujoco_control = mujoco_ros2_control::MujocoRos2Control(node, mujoco_model, mujoco_data);

  mujoco_control.init();
  RCLCPP_INFO_STREAM(
    node->get_logger(), "Mujoco ros2 controller has been successfully initialized !");

  // initialize mujoco visualization environment for rendering and cameras
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }
  auto rendering = mujoco_ros2_control::MujocoRendering::get_instance();
  rendering->init(mujoco_model, mujoco_data);
  RCLCPP_INFO_STREAM(node->get_logger(), "Mujoco rendering has been successfully initialized !");

  auto cameras = std::make_unique<mujoco_ros2_control::MujocoCameras>(node);
  cameras->init(mujoco_model);

  auto start = std::chrono::system_clock::now();
  // Some computation here

  // Timing variables for rendering, camera, and control
  std::vector<double> rendering_times;
  std::vector<double> camera_times;
  std::vector<double> control_times;
  auto last_stats_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  int camera_update_count = 0;

  // Frequency control variables using real time
  auto last_control_update = std::chrono::steady_clock::now();
  auto last_render_update = std::chrono::steady_clock::now();
  auto last_cam_update = std::chrono::steady_clock::now();
  constexpr double CONTROL_FREQ = 1000.0;
  constexpr double RENDER_FREQ = 30.0;
  constexpr double CAMERA_FREQ = 5.0;

  // run main loop with enforced frequencies using real time
  while (rclcpp::ok() && !rendering->is_close_flag_raised())
  {
    auto now = std::chrono::steady_clock::now();

    if (std::chrono::duration<double>(now - last_control_update).count() >= 1.0 / CONTROL_FREQ)
    {
      auto control_start = std::chrono::steady_clock::now();
      mujoco_control.update();
      auto control_end = std::chrono::steady_clock::now();

      double control_time_ms =
        std::chrono::duration<double, std::milli>(control_end - control_start).count();
      control_times.push_back(control_time_ms);

      last_control_update = now;
    }

    if (std::chrono::duration<double>(now - last_render_update).count() >= 1.0 / RENDER_FREQ)
    {
      auto render_start = std::chrono::steady_clock::now();
      rendering->update();
      auto render_end = std::chrono::steady_clock::now();

      double render_time_ms =
        std::chrono::duration<double, std::milli>(render_end - render_start).count();
      rendering_times.push_back(render_time_ms);
      frame_count++;

      last_render_update = now;
    }

    if (std::chrono::duration<double>(now - last_cam_update).count() >= 1.0 / CAMERA_FREQ)
    {
      auto camera_start = std::chrono::steady_clock::now();
      cameras->update(mujoco_model, mujoco_data);
      auto camera_end = std::chrono::steady_clock::now();

      double camera_time_ms =
        std::chrono::duration<double, std::milli>(camera_end - camera_start).count();
      camera_times.push_back(camera_time_ms);
      camera_update_count++;

      last_cam_update = now;
    }

    // Log combined statistics every 5 seconds
    auto current_time = now;
    auto time_since_last_stats =
      std::chrono::duration<double>(current_time - last_stats_time).count();

    if (time_since_last_stats >= 60.0)
    {
      RCLCPP_INFO(node->get_logger(), "=== Performance Stats (60s window) ===");

      // Rendering stats
      if (!rendering_times.empty())
      {
        double min_time = *std::min_element(rendering_times.begin(), rendering_times.end());
        double max_time = *std::max_element(rendering_times.begin(), rendering_times.end());
        double sum = std::accumulate(rendering_times.begin(), rendering_times.end(), 0.0);
        double mean_time = sum / rendering_times.size();
        double fps = frame_count / time_since_last_stats;

        RCLCPP_INFO(
          node->get_logger(),
          "Rendering  | Frames: %3d | FPS: %5.1f | Mean: %5.2fms | Min: %5.2fms | Max: %5.2fms",
          frame_count, fps, mean_time, min_time, max_time);
      }

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

      // Camera stats
      if (!camera_times.empty())
      {
        double min_time = *std::min_element(camera_times.begin(), camera_times.end());
        double max_time = *std::max_element(camera_times.begin(), camera_times.end());
        double sum = std::accumulate(camera_times.begin(), camera_times.end(), 0.0);
        double mean_time = sum / camera_times.size();
        double hz = camera_update_count / time_since_last_stats;

        RCLCPP_INFO(
          node->get_logger(),
          "Camera     | Updates:%4d | Hz: %6.1f | Mean: %5.2fms | Min: %5.2fms | Max: %5.2fms",
          camera_update_count, hz, mean_time, min_time, max_time);
      }

      // Summary line
      double total_render_time =
        rendering_times.empty()
          ? 0.0
          : std::accumulate(rendering_times.begin(), rendering_times.end(), 0.0);
      double total_control_time =
        control_times.empty() ? 0.0
                              : std::accumulate(control_times.begin(), control_times.end(), 0.0);
      double total_camera_time =
        camera_times.empty() ? 0.0 : std::accumulate(camera_times.begin(), camera_times.end(), 0.0);
      double total_time = total_render_time + total_control_time + total_camera_time;

      RCLCPP_INFO(
        node->get_logger(),
        "Total CPU  | Render: %5.1fms | Control: %5.1fms | Camera: %5.1fms | Total: %5.1fms",
        total_render_time, total_control_time, total_camera_time, total_time);

      RCLCPP_INFO(node->get_logger(), "==========================================");

      // Reset for next window
      rendering_times.clear();
      control_times.clear();
      camera_times.clear();
      frame_count = 0;
      camera_update_count = 0;
      last_stats_time = current_time;
    }
  }

  rendering->close();
  cameras->close();

  // free MuJoCo model and data
  mj_deleteData(mujoco_data);
  mj_deleteModel(mujoco_model);

  return 1;
}