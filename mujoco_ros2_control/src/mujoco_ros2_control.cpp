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

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{

// Object Publisher Implementation
MujocoObjectPublisher::MujocoObjectPublisher(rclcpp::Node::SharedPtr &node)
    : node_(node), last_publish_time_(0.0)
{
  object_poses_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>("/mujoco/item_position", 10);
}

void MujocoObjectPublisher::init(mjModel *mujoco_model)
{
  body_names_.clear();
  body_indices_.clear();

  // Look for custom text field with tracked bodies list
  std::set<std::string> tracked_body_names;
  bool found_tracked_list = false;

  for (int i = 0; i < mujoco_model->ntext; i++)
  {
    const char *name = mujoco_model->names + mujoco_model->name_textadr[i];
    if (std::string(name) == "tracked_bodies")
    {
      const char *data = mujoco_model->text_data + mujoco_model->text_adr[i];
      std::string body_list(data);

      // Parse comma-separated list
      std::stringstream ss(body_list);
      std::string body_name;
      while (std::getline(ss, body_name, ','))
      {
        // Trim whitespace
        body_name.erase(0, body_name.find_first_not_of(" \t"));
        body_name.erase(body_name.find_last_not_of(" \t") + 1);
        if (!body_name.empty())
        {
          tracked_body_names.insert(body_name);
        }
      }
      found_tracked_list = true;
      RCLCPP_INFO(
        node_->get_logger(), "Found tracked_bodies list with %zu bodies",
        tracked_body_names.size());
      break;
    }
  }

  if (!found_tracked_list)
  {
    RCLCPP_INFO(
      node_->get_logger(), "No 'tracked_bodies' custom text found - no bodies will be tracked");
    RCLCPP_INFO(
      node_->get_logger(),
      "Add <custom><text name=\"tracked_bodies\" data=\"body1,body2,body3\"/></custom> to your XML "
      "to track specific bodies");
    return;
  }

  // Find and add only the bodies specified in the tracked list
  for (int i = 1; i < mujoco_model->nbody; i++)  // Skip world body at index 0
  {
    const char *name = mujoco_model->names + mujoco_model->name_bodyadr[i];
    std::string body_name = std::string(name);

    if (tracked_body_names.find(body_name) != tracked_body_names.end())
    {
      body_names_.push_back(body_name);
      body_indices_.push_back(i);
    }
  }

  RCLCPP_INFO(
    node_->get_logger(), "Object publisher initialized for %zu tracked bodies", body_names_.size());

  // Log all tracked body names for debugging
  for (size_t i = 0; i < body_names_.size(); i++)
  {
    RCLCPP_INFO(
      node_->get_logger(), "Tracking body %d: '%s'", body_indices_[i], body_names_[i].c_str());
  }

  // Warn about bodies in the list that weren't found in the model
  for (const auto &requested_body : tracked_body_names)
  {
    bool found = false;
    for (const auto &tracked_body : body_names_)
    {
      if (tracked_body == requested_body)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      RCLCPP_WARN(
        node_->get_logger(), "Requested body '%s' not found in MuJoCo model",
        requested_body.c_str());
    }
  }
}

void MujocoObjectPublisher::update(mjModel *mujoco_model, mjData *mujoco_data, double sim_time)
{
  // Publish at 10Hz
  if (sim_time - last_publish_time_ < 1.0 / PUBLISH_RATE) return;

  auto time_stamp = node_->now();

  // Publish only the filtered/tracked bodies
  for (size_t idx = 0; idx < body_names_.size(); idx++)
  {
    int i = body_indices_[idx];  // Get the original MuJoCo body index

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = time_stamp;
    pose_msg.header.frame_id = body_names_[idx];  // Body name as frame_id

    // Position (xpos is nbody x 3 array)
    pose_msg.pose.position.x = mujoco_data->xpos[3 * i + 0];
    pose_msg.pose.position.y = mujoco_data->xpos[3 * i + 1];
    pose_msg.pose.position.z = mujoco_data->xpos[3 * i + 2];

    // Orientation (xquat is nbody x 4 array, stored as w,x,y,z)
    pose_msg.pose.orientation.w = mujoco_data->xquat[4 * i + 0];
    pose_msg.pose.orientation.x = mujoco_data->xquat[4 * i + 1];
    pose_msg.pose.orientation.y = mujoco_data->xquat[4 * i + 2];
    pose_msg.pose.orientation.z = mujoco_data->xquat[4 * i + 3];

    object_poses_pub_->publish(pose_msg);
  }

  last_publish_time_ = sim_time;
}

// Main MujocoRos2Control Implementation
MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data)
    : node_(node),
      mj_model_(mujoco_model),
      mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
{
  // Initialize object publisher
  object_publisher_ = std::make_unique<MujocoObjectPublisher>(node);
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();

  if (cm_thread_.joinable()) cm_thread_.join();
}

std::string MujocoRos2Control::get_robot_description()
{
  // Getting robot description from parameter first. If not set trying from topic
  std::string robot_description;

  auto node = std::make_shared<rclcpp::Node>(
    "robot_description_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  if (node->has_parameter("robot_description"))
  {
    robot_description = node->get_parameter("robot_description").as_string();
    return robot_description;
  }

  RCLCPP_WARN(
    logger_,
    "Failed to get robot_description from parameter. Will listen on the ~/robot_description "
    "topic...");

  auto robot_description_sub = node->create_subscription<std_msgs::msg::String>(
    "robot_description", rclcpp::QoS(1).transient_local(),
    [&](const std_msgs::msg::String::SharedPtr msg)
    {
      if (!msg->data.empty() && robot_description.empty()) robot_description = msg->data;
    });

  while (robot_description.empty() && rclcpp::ok())
  {
    rclcpp::spin_some(node);
    RCLCPP_INFO(node->get_logger(), "Waiting for robot description message");
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }

  return robot_description;
}

void MujocoRos2Control::init()
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  std::string urdf_string = this->get_robot_description();

  // setup actuators and mechanism control node.
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try
  {
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  }
  catch (const std::runtime_error &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

  try
  {
    robot_hw_sim_loader_.reset(new pluginlib::ClassLoader<MujocoSystemInterface>(
      "mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"));
  }
  catch (pluginlib::LibraryLoadException &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create hardware interface loader:  " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<hardware_interface::ResourceManager>();

  try
  {
    resource_manager->load_urdf(urdf_string, false, false);
  }
  catch (...)
  {
    RCLCPP_ERROR(logger_, "Error while initializing URDF!");
  }

  for (const auto &hardware : control_hardware_info)
  {
    std::string robot_hw_sim_type_str_ = hardware.hardware_class_type;
    std::unique_ptr<MujocoSystemInterface> mujoco_system;
    try
    {
      mujoco_system = std::unique_ptr<MujocoSystemInterface>(
        robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    }
    catch (pluginlib::PluginlibException &ex)
    {
      RCLCPP_ERROR_STREAM(logger_, "The plugin failed to load. Error: " << ex.what());
      continue;
    }

    urdf::Model urdf_model;
    urdf_model.initString(urdf_string);
    if (!mujoco_system->init_sim(mj_model_, mj_data_, urdf_model, hardware))
    {
      RCLCPP_FATAL(logger_, "Could not initialize robot simulation interface");
      return;
    }

    resource_manager->import_component(std::move(mujoco_system), hardware);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager->set_component_state(hardware.name, state);
  }

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace());
  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate"))
  {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);

  // Initialize object publisher
  object_publisher_->init(mj_model_);
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(sim_time);
  int sim_time_nanosec = static_cast<int>((sim_time - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);

  mj_step1(mj_model_, mj_data_);

  if (sim_period >= control_period_)
  {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }
  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  mj_step2(mj_model_, mj_data_);

  // Update object poses publisher
  object_publisher_->update(mj_model_, mj_data_, sim_time);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

}  // namespace mujoco_ros2_control