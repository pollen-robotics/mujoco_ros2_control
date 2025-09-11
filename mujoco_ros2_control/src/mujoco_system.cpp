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

#include "mujoco_ros2_control/mujoco_system.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

// WebSocket includes
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>

namespace mujoco_ros2_control
{

// Static singleton WebSocket manager with centralized odometry
class WebSocketManager
{
public:
  static WebSocketManager &getInstance()
  {
    static WebSocketManager instance;
    return instance;
  }

  bool connect(const std::string &url = "ws://127.0.0.1:8765")
  {
    if (connected_)
    {
      return true;  // Already connected
    }

    ws_.setUrl(url);

    // Use the correct IXWebSocket callback API
    ws_.setOnMessageCallback(
      [this](const ix::WebSocketMessagePtr &msg)
      {
        if (msg->type == ix::WebSocketMessageType::Message)
        {
          handleMessage(msg);
        }
        else if (msg->type == ix::WebSocketMessageType::Open)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          connected_ = true;
          std::cout << "WebSocket connected!" << std::endl;
        }
        else if (msg->type == ix::WebSocketMessageType::Close)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          connected_ = false;
          std::cout << "WebSocket disconnected. Code: " << msg->closeInfo.code
                    << ", Reason: " << msg->closeInfo.reason << std::endl;
        }
        else if (msg->type == ix::WebSocketMessageType::Error)
        {
          std::cout << "WebSocket error: " << msg->errorInfo.reason << std::endl;
        }
      });

    ws_.start();

    // Wait for connection
    while (!connected_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (connected_)
    {
      // Request model info immediately after connection
      nlohmann::json cmd;
      cmd["type"] = "get_model_info";
      ws_.sendText(cmd.dump());
    }

    return connected_;
  }

  void sendCommand(const nlohmann::json &cmd)
  {
    if (connected_)
    {
      ws_.sendText(cmd.dump());
    }
  }

  void sendJointControl(const std::string &joint_name, const std::string &mode, double value)
  {
    if (connected_)
    {
      nlohmann::json cmd;
      cmd["type"] = "set_joint_control";
      cmd["joint_name"] = joint_name;
      cmd["mode"] = mode;
      cmd["value"] = value;
      ws_.sendText(cmd.dump());
    }
  }

  void sendBulkControl(const std::vector<std::tuple<std::string, std::string, double>> &commands)
  {
    if (connected_ && !commands.empty())
    {
      nlohmann::json cmd;
      cmd["type"] = "set_bulk_control";
      cmd["joints"] = nlohmann::json::array();

      for (const auto &[name, mode, value] : commands)
      {
        nlohmann::json joint_cmd;
        joint_cmd["name"] = name;
        joint_cmd["mode"] = mode;
        joint_cmd["value"] = value;
        cmd["joints"].push_back(joint_cmd);
      }

      ws_.sendText(cmd.dump());
    }
  }

  bool getLatestState(
    std::vector<double> &qpos, std::vector<double> &qvel, std::vector<double> &qfrc_applied,
    std::vector<double> &sensordata, double &time)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_received_)
    {
      return false;
    }

    qpos = cached_qpos_;
    qvel = cached_qvel_;
    qfrc_applied = cached_qfrc_applied_;
    sensordata = cached_sensor_data_;
    time = cached_time_;

    return true;
  }

  bool isConnected() const { return connected_; }

  void shutdown()
  {
    if (connected_)
    {
      ws_.stop();
      connected_ = false;
    }

    // Cleanup odometry publisher
    if (odom_initialized_)
    {
      if (executor_)
      {
        executor_->cancel();
      }
      if (spin_thread_.joinable())
      {
        spin_thread_.join();
      }
      odom_initialized_ = false;
    }
  }

  // Centralized odometry publishing
  void publishOdometry(const rclcpp::Time &time)
  {
    if (!odom_initialized_)
    {
      initializeOdometry();
    }

    if (odom_publisher_)
    {
      auto odom = nav_msgs::msg::Odometry();
      odom.header.stamp = builtin_interfaces::msg::Time();
      odom.header.stamp.sec = static_cast<int32_t>(time.seconds());
      odom.header.stamp.nanosec =
        static_cast<uint32_t>((time.seconds() - odom.header.stamp.sec) * 1e9);
      odom.header.frame_id = "odom_mujoco";
      odom.child_frame_id = "base_link";
      odom_publisher_->publish(odom);
    }
  }

private:
  WebSocketManager() { ix::initNetSystem(); }

  ~WebSocketManager() { shutdown(); }

  void initializeOdometry()
  {
    if (odom_initialized_) return;

    node_ = rclcpp::Node::make_shared("mujoco_websocket_odom_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom_mujoco", 10);
    odom_initialized_ = true;

    std::cout << "Centralized odometry publisher initialized (WebSocket mode)." << std::endl;
  }

  void handleMessage(const ix::WebSocketMessagePtr &msg)
  {
    if (!msg->binary)
    {
      try
      {
        auto j = nlohmann::json::parse(msg->str);
        std::string type = j.value("type", "");

        if (type == "state" && j.contains("state"))
        {
          std::lock_guard<std::mutex> lock(state_mutex_);

          auto state = j["state"];
          cached_time_ = state.value("time", 0.0);

          if (state.contains("qpos") && state["qpos"].is_array())
          {
            cached_qpos_ = state["qpos"].get<std::vector<double>>();
          }

          if (state.contains("qvel") && state["qvel"].is_array())
          {
            cached_qvel_ = state["qvel"].get<std::vector<double>>();
          }

          if (state.contains("qfrc_applied") && state["qfrc_applied"].is_array())
          {
            cached_qfrc_applied_ = state["qfrc_applied"].get<std::vector<double>>();
          }

          if (state.contains("sensordata") && state["sensordata"].is_array())
          {
            cached_sensor_data_ = state["sensordata"].get<std::vector<double>>();
          }

          state_received_ = true;
        }
        else if (type == "model_info")
        {
          // Store model info if needed
          std::cout << "Received model info from server" << std::endl;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "WebSocket JSON parse error: " << e.what() << std::endl;
      }
    }
  }

  ix::WebSocket ws_;
  std::mutex state_mutex_;
  bool connected_ = false;
  bool state_received_ = false;

  // Cached state
  std::vector<double> cached_qpos_;
  std::vector<double> cached_qvel_;
  std::vector<double> cached_qfrc_applied_;
  std::vector<double> cached_sensor_data_;
  double cached_time_ = 0.0;

  // Centralized odometry members
  bool odom_initialized_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

MujocoSystem::MujocoSystem() : logger_(rclcpp::get_logger("")) {}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
{
  return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
{
  RCLCPP_INFO(
    rclcpp::get_logger("mujoco_ros2_control"), "Total command interfaces: %ld",
    command_interfaces_.size());
  for (const auto &iface : command_interfaces_)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("mujoco_ros2_control"), "  - Command interface: %s/%s",
      iface.get_name().c_str(), iface.get_interface_name().c_str());
  }
  return std::move(command_interfaces_);
}

hardware_interface::return_type MujocoSystem::read(
  const rclcpp::Time &time, const rclcpp::Duration & /* period */)
{
  auto &wsManager = WebSocketManager::getInstance();

  if (!wsManager.isConnected())
  {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000, "WebSocket not connected");
    return hardware_interface::return_type::ERROR;
  }

  std::vector<double> qpos, qvel, qfrc_applied, sensordata;
  double sim_time;

  if (!wsManager.getLatestState(qpos, qvel, qfrc_applied, sensordata, sim_time))
  {
    // No fresh state, but that's OK - we'll use previous values
    return hardware_interface::return_type::OK;
  }

  // Update joint states from WebSocket data
  for (size_t i = 0; i < joint_states_.size(); ++i)
  {
    if (joint_states_[i].mj_pos_adr < qpos.size())
    {
      joint_states_[i].position = qpos[joint_states_[i].mj_pos_adr];
    }

    if (joint_states_[i].mj_vel_adr < qvel.size())
    {
      joint_states_[i].velocity = qvel[joint_states_[i].mj_vel_adr];
    }

    if (joint_states_[i].mj_vel_adr < qfrc_applied.size())
    {
      joint_states_[i].effort = qfrc_applied[joint_states_[i].mj_vel_adr];
    }
  }

  // Update FT sensor data
  for (auto &data : ft_sensor_data_)
  {
    if (data.force.mj_sensor_index + 2 < sensordata.size())
    {
      data.force.data.x() = -sensordata[data.force.mj_sensor_index];
      data.force.data.y() = -sensordata[data.force.mj_sensor_index + 1];
      data.force.data.z() = -sensordata[data.force.mj_sensor_index + 2];
    }

    if (data.torque.mj_sensor_index + 2 < sensordata.size())
    {
      data.torque.data.x() = -sensordata[data.torque.mj_sensor_index];
      data.torque.data.y() = -sensordata[data.torque.mj_sensor_index + 1];
      data.torque.data.z() = -sensordata[data.torque.mj_sensor_index + 2];
    }
  }

  // Use centralized odometry publishing
  wsManager.publishOdometry(time);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration &period)
{
  auto &wsManager = WebSocketManager::getInstance();

  if (!wsManager.isConnected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // Update mimic joints
  for (auto &joint_state : joint_states_)
  {
    if (joint_state.is_mimic)
    {
      joint_state.position_command =
        joint_state.mimic_multiplier *
        joint_states_.at(joint_state.mimicked_joint_index).position_command;
      joint_state.velocity_command =
        joint_state.mimic_multiplier *
        joint_states_.at(joint_state.mimicked_joint_index).velocity_command;
      joint_state.effort_command =
        joint_state.mimic_multiplier *
        joint_states_.at(joint_state.mimicked_joint_index).effort_command;
    }
  }

  // Apply joint limits for position control
  for (auto &joint_state : joint_states_)
  {
    if (joint_state.is_position_control_enabled)
    {
      joint_state.position_command =
        std::min(joint_state.joint_limits.max_position, joint_state.position_command);
      joint_state.position_command =
        std::max(joint_state.joint_limits.min_position, joint_state.position_command);
    }

    // Handle PID control if enabled
    if (joint_state.is_pid_enabled && joint_state.is_position_control_enabled)
    {
      double error = joint_state.position_command - joint_state.position;
      joint_state.effort_command =
        joint_state.position_pid.computeCommand(error, period.nanoseconds());
    }

    if (joint_state.is_pid_enabled && joint_state.is_velocity_control_enabled)
    {
      double error = joint_state.velocity_command - joint_state.velocity;
      joint_state.effort_command =
        joint_state.velocity_pid.computeCommand(error, period.nanoseconds());
    }

    if (joint_state.is_effort_control_enabled)
    {
      double min_eff = joint_state.joint_limits.has_effort_limits
                         ? -1 * joint_state.joint_limits.max_effort
                         : std::numeric_limits<double>::lowest();
      min_eff = std::max(min_eff, joint_state.min_effort_command);

      double max_eff = joint_state.joint_limits.has_effort_limits
                         ? joint_state.joint_limits.max_effort
                         : std::numeric_limits<double>::max();
      max_eff = std::min(max_eff, joint_state.max_effort_command);

      joint_state.effort_command = clamp(joint_state.effort_command, min_eff, max_eff);
    }
  }

  // Send commands via WebSocket using bulk control for efficiency
  std::vector<std::tuple<std::string, std::string, double>> commands;

  for (const auto &joint_state : joint_states_)
  {
    std::string mode;
    double value = 0.0;

    // Determine control mode and value for this joint
    if (joint_state.is_position_control_enabled && !joint_state.is_pid_enabled)
    {
      mode = "position";
      value = joint_state.position_command;
    }
    else if (joint_state.is_velocity_control_enabled && !joint_state.is_pid_enabled)
    {
      mode = "velocity";
      value = joint_state.velocity_command;
    }
    else if (joint_state.is_effort_control_enabled || joint_state.is_pid_enabled)
    {
      mode = "effort";
      value = joint_state.effort_command;
    }
    else
    {
      continue;  // Skip joints with no active control
    }

    commands.push_back({joint_state.name, mode, value});
  }

  // Send bulk command if there are any commands to send
  if (!commands.empty())
  {
    wsManager.sendBulkControl(commands);
  }

  return hardware_interface::return_type::OK;
}

bool MujocoSystem::init_sim(
  mjModel *mujoco_model, mjData *mujoco_data, const urdf::Model &urdf_model,
  const hardware_interface::HardwareInfo &hardware_info)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;
  logger_ = rclcpp::get_logger("mujoco_system");

  // Connect to WebSocket (singleton ensures only one connection)
  auto &wsManager = WebSocketManager::getInstance();
  if (!wsManager.connect())
  {
    RCLCPP_ERROR(logger_, "Failed to connect to WebSocket server");
    return false;
  }

  RCLCPP_INFO(logger_, "Connected to MuJoCo WebSocket server");

  register_joints(urdf_model, hardware_info);
  register_sensors(urdf_model, hardware_info);

  // Send initial reset command with initial joint positions
  nlohmann::json reset_cmd;
  reset_cmd["type"] = "reset";
  reset_cmd["state"]["qpos"] = std::vector<double>(mj_model_->nq, 0.0);
  reset_cmd["state"]["qvel"] = std::vector<double>(mj_model_->nv, 0.0);

  // Set initial positions from joint_states_
  for (const auto &joint_state : joint_states_)
  {
    if (joint_state.mj_pos_adr < mj_model_->nq)
    {
      reset_cmd["state"]["qpos"][joint_state.mj_pos_adr] = joint_state.position;
    }
  }

  wsManager.sendCommand(reset_cmd);

  RCLCPP_INFO(logger_, "MujocoSystem initialization complete (WebSocket mode)");

  return true;
}

// Rest of the implementation remains the same
void MujocoSystem::register_joints(
  const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info)
{
  joint_states_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    int mujoco_joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
    if (mujoco_joint_id == -1)
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find joint in mujoco model, joint name: " << joint.name);
      continue;
    }

    // save information in joint_states_ variable
    JointState joint_state;
    joint_state.name = joint.name;
    joint_state.mj_joint_type = mj_model_->jnt_type[mujoco_joint_id];
    joint_state.mj_pos_adr = mj_model_->jnt_qposadr[mujoco_joint_id];
    joint_state.mj_vel_adr = mj_model_->jnt_dofadr[mujoco_joint_id];
    joint_state.mj_act_adr = mj_name2id(mj_model_, mjOBJ_ACTUATOR, joint.name.c_str());
    joint_states_.at(joint_index) = joint_state;
    JointState &last_joint_state = joint_states_.at(joint_index);

    // get joint limit from urdf
    get_joint_limits(urdf_model.getJoint(last_joint_state.name), last_joint_state.joint_limits);

    // check if mimicked
    if (joint.parameters.find("mimic") != joint.parameters.end())
    {
      const auto mimicked_joint = joint.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(
        hardware_info.joints.begin(), hardware_info.joints.end(),
        [&mimicked_joint](const hardware_interface::ComponentInfo &info)
        { return info.name == mimicked_joint; });
      if (mimicked_joint_it == hardware_info.joints.end())
      {
        throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }
      last_joint_state.is_mimic = true;
      last_joint_state.mimicked_joint_index =
        std::distance(hardware_info.joints.begin(), mimicked_joint_it);

      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        last_joint_state.mimic_multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      else
      {
        last_joint_state.mimic_multiplier = 1.0;
      }
    }

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo &interface_info)
    {
      if (!interface_info.initial_value.empty())
      {
        double value = std::stod(interface_info.initial_value);
        return value;
      }
      else
      {
        return 0.0;
      }
    };

    // state interfaces
    for (const auto &state_if : joint.state_interfaces)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("mujoco_ros2_control"), "Joint %s has state interface: %s",
        joint.name.c_str(), state_if.name.c_str());

      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_POSITION, &last_joint_state.position);
        last_joint_state.position = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &last_joint_state.velocity);
        last_joint_state.velocity = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        state_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_EFFORT, &last_joint_state.effort);
        last_joint_state.effort = get_initial_value(state_if);
      }
    }

    auto get_min_value = [this](const hardware_interface::InterfaceInfo &interface_info)
    {
      if (!interface_info.min.empty())
      {
        double value = std::stod(interface_info.min);
        return value;
      }
      else
      {
        return -1 * std::numeric_limits<double>::max();
      }
    };

    auto get_max_value = [this](const hardware_interface::InterfaceInfo &interface_info)
    {
      if (!interface_info.max.empty())
      {
        double value = std::stod(interface_info.max);
        return value;
      }
      else
      {
        return std::numeric_limits<double>::max();
      }
    };

    // command interfaces
    for (const auto &command_if : joint.command_interfaces)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("mujoco_ros2_control"), "Joint %s has command interface: %s",
        joint.name.c_str(), command_if.name.c_str());

      if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
      {
        command_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_POSITION, &last_joint_state.position_command);
        last_joint_state.is_position_control_enabled = true;
        last_joint_state.position_command = last_joint_state.position;
        last_joint_state.min_position_command = get_min_value(command_if);
        last_joint_state.max_position_command = get_max_value(command_if);
      }
      else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      {
        command_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_VELOCITY, &last_joint_state.velocity_command);
        last_joint_state.is_velocity_control_enabled = true;
        last_joint_state.velocity_command = last_joint_state.velocity;
        last_joint_state.min_velocity_command = get_min_value(command_if);
        last_joint_state.max_velocity_command = get_max_value(command_if);
      }
      else if (command_if.name == hardware_interface::HW_IF_EFFORT)
      {
        command_interfaces_.emplace_back(
          joint.name, hardware_interface::HW_IF_EFFORT, &last_joint_state.effort_command);
        last_joint_state.is_effort_control_enabled = true;
        last_joint_state.effort_command = last_joint_state.effort;
        last_joint_state.min_effort_command = get_min_value(command_if);
        last_joint_state.max_effort_command = get_max_value(command_if);
      }

      if (command_if.name.find("_pid") != std::string::npos)
      {
        last_joint_state.is_pid_enabled = true;
      }
    }

    // Get PID gains, if needed
    if (last_joint_state.is_pid_enabled)
    {
      last_joint_state.position_pid = get_pid_gains(joint, hardware_interface::HW_IF_POSITION);
      last_joint_state.velocity_pid = get_pid_gains(joint, hardware_interface::HW_IF_VELOCITY);
    }
  }
}

void MujocoSystem::register_sensors(
  const urdf::Model & /* urdf_model */, const hardware_interface::HardwareInfo &hardware_info)
{
  ft_sensor_data_.resize(hardware_info.sensors.size());

  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);

    FTSensorData sensor_data;
    sensor_data.name = sensor.name;
    sensor_data.force.name = sensor.name + "_force";
    sensor_data.torque.name = sensor.name + "_torque";

    int force_sensor_id =
      mj_name2id(mj_model_, mjtObj::mjOBJ_SENSOR, sensor_data.force.name.c_str());
    int torque_sensor_id =
      mj_name2id(mj_model_, mjtObj::mjOBJ_SENSOR, sensor_data.torque.name.c_str());

    if (force_sensor_id == -1 || torque_sensor_id == -1)
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find sensor in mujoco model, sensor name: " << sensor.name);
      continue;
    }

    sensor_data.force.mj_sensor_index = mj_model_->sensor_adr[force_sensor_id];
    sensor_data.torque.mj_sensor_index = mj_model_->sensor_adr[torque_sensor_id];

    ft_sensor_data_.at(sensor_index) = sensor_data;
    auto &last_sensor_data = ft_sensor_data_.at(sensor_index);

    for (const auto &state_if : sensor.state_interfaces)
    {
      if (state_if.name == "force.x")
      {
        state_interfaces_.emplace_back(
          sensor.name, state_if.name, &last_sensor_data.force.data.x());
      }
      else if (state_if.name == "force.y")
      {
        state_interfaces_.emplace_back(
          sensor.name, state_if.name, &last_sensor_data.force.data.y());
      }
      else if (state_if.name == "force.z")
      {
        state_interfaces_.emplace_back(
          sensor.name, state_if.name, &last_sensor_data.force.data.z());
      }
      else if (state_if.name == "torque.x")
      {
        state_interfaces_.emplace_back(
          sensor.name, state_if.name, &last_sensor_data.torque.data.x());
      }
      else if (state_if.name == "torque.y")
      {
        state_interfaces_.emplace_back(
          sensor.name, state_if.name, &last_sensor_data.torque.data.y());
      }
      else if (state_if.name == "torque.z")
      {
        state_interfaces_.emplace_back(
          sensor.name, state_if.name, &last_sensor_data.torque.data.z());
      }
    }
  }
}

void MujocoSystem::set_initial_pose()
{
  // Initial pose is now set via WebSocket in init_sim
  // This function is kept for compatibility but doesn't need to do anything
}

void MujocoSystem::get_joint_limits(
  urdf::JointConstSharedPtr urdf_joint, joint_limits::JointLimits &joint_limits)
{
  if (urdf_joint->limits)
  {
    joint_limits.min_position = urdf_joint->limits->lower;
    joint_limits.max_position = urdf_joint->limits->upper;
    joint_limits.max_velocity = urdf_joint->limits->velocity;
    joint_limits.max_effort = urdf_joint->limits->effort;
  }
}

control_toolbox::Pid MujocoSystem::get_pid_gains(
  const hardware_interface::ComponentInfo &joint_info, std::string command_interface)
{
  double kp, ki, kd, i_max, i_min;
  std::string key;
  key = command_interface + std::string(PARAM_KP);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    kp = std::stod(joint_info.parameters.at(key));
  }
  else
  {
    kp = 0.0;
  }

  key = command_interface + std::string(PARAM_KI);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    ki = std::stod(joint_info.parameters.at(key));
  }
  else
  {
    ki = 0.0;
  }

  key = command_interface + std::string(PARAM_KD);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    kd = std::stod(joint_info.parameters.at(key));
  }
  else
  {
    kd = 0.0;
  }

  bool enable_anti_windup = false;
  key = command_interface + std::string(PARAM_I_MAX);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    i_max = std::stod(joint_info.parameters.at(key));
    enable_anti_windup = true;
  }
  else
  {
    i_max = std::numeric_limits<double>::max();
  }

  key = command_interface + std::string(PARAM_I_MIN);
  if (joint_info.parameters.find(key) != joint_info.parameters.end())
  {
    i_min = std::stod(joint_info.parameters.at(key));
    enable_anti_windup = true;
  }
  else
  {
    i_min = std::numeric_limits<double>::lowest();
  }

  return control_toolbox::Pid(kp, ki, kd, i_max, i_min, enable_anti_windup);
}

MujocoSystem::~MujocoSystem()
{
  // Individual MujocoSystem instances no longer manage odometry
  // The WebSocket singleton handles centralized odometry cleanup
  RCLCPP_INFO(rclcpp::get_logger("mujoco_system"), "MujocoSystem destructor called.");
}

}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)