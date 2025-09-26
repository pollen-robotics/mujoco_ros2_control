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

// Forward declarations to avoid direct mujoco dependency
struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_ mjData;
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

// WebSocket includes
#include <ixwebsocket/IXNetSystem.h>
#include <ixwebsocket/IXWebSocket.h>
#include <condition_variable>
#include <mutex>
#include <nlohmann/json.hpp>

// Camera support includes
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// Tracked bodies support includes
#include <geometry_msgs/msg/pose_stamped.hpp>

// Odometry support includes
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace mujoco_ros2_control
{

// Structure to hold camera publisher information
struct CameraPublisher
{
  std::string name;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub;
  bool has_depth;
  int width;
  int height;
  double fovy;
  std::string frame_name;
};

// Structure to hold tracked body publisher information
struct TrackedBodyPublisher
{
  std::string name;
  int body_id;
  std::string frame_id;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
};

// Structure to hold odometry data
struct OdometryData
{
  std::string frame_id;
  std::string child_frame_id;
  std::vector<double> position;          // [x, y, z]
  std::vector<double> orientation;       // [w, x, y, z] quaternion
  std::vector<double> linear_velocity;   // [vx, vy, vz]
  std::vector<double> angular_velocity;  // [wx, wy, wz]
};

// Robot description structure to replace Mujoco model queries
struct RobotDescription
{
  struct JointInfo
  {
    std::string name;
    int joint_type;
    int pos_adr;
    int vel_adr;
    int act_adr;
  };

  struct SensorInfo
  {
    std::string name;
    int sensor_adr;
  };

  struct ModelDimensions
  {
    int nq;  // number of positions
    int nv;  // number of velocities
  };

  ModelDimensions dimensions;
  std::vector<JointInfo> joints;
  std::vector<SensorInfo> sensors;

  // Helper methods
  int findJointIndex(const std::string &name) const
  {
    for (size_t i = 0; i < joints.size(); ++i)
    {
      if (joints[i].name == name)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  int findSensorIndex(const std::string &name) const
  {
    for (size_t i = 0; i < sensors.size(); ++i)
    {
      if (sensors[i].name == name)
      {
        return static_cast<int>(i);
      }
    }
    return -1;
  }
};

// Static singleton WebSocket manager with centralized odometry, camera, and tracked bodies support
class WebSocketManager
{
public:
  static WebSocketManager &getInstance()
  {
    static WebSocketManager instance;
    return instance;
  }

  // bool connect(const std::string &url = "ws://127.0.0.1:8765")
  bool connect(const std::string &url = "ws://host.docker.internal:8765")
  // bool connect(const std::string &url = "ws://192.168.1.94:8765")
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
      // Request model info and robot description immediately after connection
      nlohmann::json cmd;
      cmd["type"] = "get_model_info";
      ws_.sendText(cmd.dump());

      // Request robot description for joint and sensor mapping
      nlohmann::json desc_cmd;
      desc_cmd["type"] = "get_description";
      ws_.sendText(desc_cmd.dump());
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
    std::vector<double> &sensordata, double &time, OdometryData &odom_data)
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
    odom_data = cached_odom_data_;

    return true;
  }

  bool isConnected() const { return connected_; }

  const RobotDescription& getRobotDescription() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return robot_description_;
  }

  bool hasRobotDescription() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return description_received_;
  }

  void shutdown()
  {
    if (connected_)
    {
      ws_.stop();
      connected_ = false;
    }

    // Cleanup odometry, camera, and tracked body publishers
    if (ros_initialized_)
    {
      if (executor_)
      {
        executor_->cancel();
      }
      if (spin_thread_.joinable())
      {
        spin_thread_.join();
      }
      ros_initialized_ = false;
    }
  }

  // Centralized odometry publishing with actual data
  void publishOdometry(const rclcpp::Time &time, const OdometryData &odom_data)
  {
    if (!ros_initialized_)
    {
      initializeROS();
    }

    if (odom_publisher_ && !odom_data.position.empty() && !odom_data.orientation.empty())
    {
      auto odom = nav_msgs::msg::Odometry();

      // Set header
      odom.header.stamp = builtin_interfaces::msg::Time();
      odom.header.stamp.sec = static_cast<int32_t>(time.seconds());
      odom.header.stamp.nanosec =
        static_cast<uint32_t>((time.seconds() - odom.header.stamp.sec) * 1e9);
      odom.header.frame_id = odom_data.frame_id.empty() ? "odom_mujoco" : odom_data.frame_id;
      odom.child_frame_id =
        odom_data.child_frame_id.empty() ? "base_link" : odom_data.child_frame_id;

      // Set position
      if (odom_data.position.size() >= 3)
      {
        odom.pose.pose.position.x = odom_data.position[0];
        odom.pose.pose.position.y = odom_data.position[1];
        odom.pose.pose.position.z = odom_data.position[2];
      }

      // Set orientation
      if (odom_data.orientation.size() >= 4)
      {
        odom.pose.pose.orientation.w = odom_data.orientation[0];
        odom.pose.pose.orientation.x = odom_data.orientation[1];
        odom.pose.pose.orientation.y = odom_data.orientation[2];
        odom.pose.pose.orientation.z = odom_data.orientation[3];
      }

      // Set linear velocity
      if (odom_data.linear_velocity.size() >= 3)
      {
        odom.twist.twist.linear.x = odom_data.linear_velocity[0];
        odom.twist.twist.linear.y = odom_data.linear_velocity[1];
        odom.twist.twist.linear.z = odom_data.linear_velocity[2];
      }

      // Set angular velocity
      if (odom_data.angular_velocity.size() >= 3)
      {
        odom.twist.twist.angular.x = odom_data.angular_velocity[0];
        odom.twist.twist.angular.y = odom_data.angular_velocity[1];
        odom.twist.twist.angular.z = odom_data.angular_velocity[2];
      }

      // Set covariance matrices (simple diagonal covariance)
      std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
      std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);

      // Set some reasonable covariance values (you can tune these)
      odom.pose.covariance[0] = 0.001;   // x position variance
      odom.pose.covariance[7] = 0.001;   // y position variance
      odom.pose.covariance[14] = 0.001;  // z position variance
      odom.pose.covariance[21] = 0.001;  // roll variance
      odom.pose.covariance[28] = 0.001;  // pitch variance
      odom.pose.covariance[35] = 0.001;  // yaw variance

      odom.twist.covariance[0] = 0.001;   // vx variance
      odom.twist.covariance[7] = 0.001;   // vy variance
      odom.twist.covariance[14] = 0.001;  // vz variance
      odom.twist.covariance[21] = 0.001;  // wx variance
      odom.twist.covariance[28] = 0.001;  // wy variance
      odom.twist.covariance[35] = 0.001;  // wz variance

      odom_publisher_->publish(odom);
    }
  }

private:
  WebSocketManager() { ix::initNetSystem(); }

  ~WebSocketManager() { shutdown(); }

  void initializeROS()
  {
    if (ros_initialized_) return;

    node_ = rclcpp::Node::make_shared("mujoco_websocket_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom_mujoco", 10);
    ros_initialized_ = true;

    std::cout << "ROS2 node initialized (WebSocket mode)." << std::endl;
  }

  void initializeCameraPublisher(const nlohmann::json &camera_info)
  {
    if (!ros_initialized_)
    {
      initializeROS();
    }

    std::string cam_name = camera_info.value("name", "");

    // Check if already initialized
    if (camera_publishers_.find(cam_name) != camera_publishers_.end())
    {
      return;
    }

    CameraPublisher cam_pub;
    cam_pub.name = cam_name;
    cam_pub.width = camera_info.value("width", 320);
    cam_pub.height = camera_info.value("height", 240);
    cam_pub.fovy = camera_info.value("fovy", 45.0);
    cam_pub.frame_name = camera_info.value("frame_name", cam_name + "_optical_frame");
    cam_pub.has_depth = camera_info.value("has_depth", false);

    // Get topic paths from camera info
    auto topic_paths = camera_info.value("topic_paths", nlohmann::json::object());

    // Create publishers based on topic paths
    if (topic_paths.contains("image"))
    {
      std::string image_topic = "/" + topic_paths["image"].get<std::string>();
      cam_pub.image_pub =
        node_->create_publisher<sensor_msgs::msg::CompressedImage>(image_topic, 10);
    }

    if (topic_paths.contains("camera_info"))
    {
      std::string info_topic = "/" + topic_paths["camera_info"].get<std::string>();
      cam_pub.camera_info_pub =
        node_->create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);
    }

    if (cam_pub.has_depth)
    {
      if (topic_paths.contains("depth_image"))
      {
        std::string depth_topic = "/" + topic_paths["depth_image"].get<std::string>();
        cam_pub.depth_pub = node_->create_publisher<sensor_msgs::msg::Image>(depth_topic, 10);
      }

      if (topic_paths.contains("depth_camera_info"))
      {
        std::string depth_info_topic = "/" + topic_paths["depth_camera_info"].get<std::string>();
        cam_pub.depth_info_pub =
          node_->create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic, 10);
      }
    }

    camera_publishers_[cam_name] = cam_pub;
    std::cout << "Initialized camera publisher for: " << cam_name << std::endl;
  }

  void initializeTrackedBodyPublisher(
    const std::string &body_name, int body_id, const std::string &frame_id)
  {
    if (!ros_initialized_)
    {
      initializeROS();
    }

    // Check if already initialized
    if (tracked_body_publishers_.find(body_name) != tracked_body_publishers_.end())
    {
      return;
    }

    TrackedBodyPublisher body_pub;
    body_pub.name = body_name;
    body_pub.body_id = body_id;
    body_pub.frame_id = frame_id;

    // Create pose publisher using the same topic pattern as your original C++ code
    std::string pose_topic = "/mujoco/item_position";
    body_pub.pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    tracked_body_publishers_[body_name] = body_pub;
    std::cout << "Initialized tracked body publisher for: " << body_name << " (ID: " << body_id
              << ")" << std::endl;
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

          // Extract odometry data if available
          if (state.contains("odometry") && state["odometry"].is_object())
          {
            auto odom = state["odometry"];
            cached_odom_data_.frame_id = odom.value("frame_id", "odom_mujoco");
            cached_odom_data_.child_frame_id = odom.value("child_frame_id", "base_link");

            if (odom.contains("position") && odom["position"].is_array())
            {
              cached_odom_data_.position = odom["position"].get<std::vector<double>>();
            }

            if (odom.contains("orientation") && odom["orientation"].is_array())
            {
              cached_odom_data_.orientation = odom["orientation"].get<std::vector<double>>();
            }

            if (odom.contains("linear_velocity") && odom["linear_velocity"].is_array())
            {
              cached_odom_data_.linear_velocity =
                odom["linear_velocity"].get<std::vector<double>>();
            }

            if (odom.contains("angular_velocity") && odom["angular_velocity"].is_array())
            {
              cached_odom_data_.angular_velocity =
                odom["angular_velocity"].get<std::vector<double>>();
            }
          }
          else
          {
            // Clear odometry data if not available
            cached_odom_data_.position.clear();
            cached_odom_data_.orientation.clear();
            cached_odom_data_.linear_velocity.clear();
            cached_odom_data_.angular_velocity.clear();
          }

          state_received_ = true;
        }
        else if (type == "model_info")
        {
          // Store model info and initialize camera publishers if needed
          std::cout << "Received model info from server" << std::endl;

          if (j.contains("cameras") && j["cameras"].is_array())
          {
            for (const auto &cam_info : j["cameras"])
            {
              initializeCameraPublisher(cam_info);
            }
          }
        }
        else if (type == "description")
        {
          // Handle robot description response
          processRobotDescription(j);
        }
        else if (type == "tracked_bodies")
        {
          // Handle tracked bodies data
          processTrackedBodies(j);
        }
        else if (type == "camera_frame")
        {
          // Store camera frame header for processing with binary data
          pending_camera_frame_ = j;
          expecting_camera_binary_ = true;
          binary_frames_expected_ = 1;  // RGB data
          if (j.value("has_depth", false))
          {
            binary_frames_expected_ = 2;  // RGB + depth data
          }
          binary_frames_received_ = 0;
        }
      }
      catch (const std::exception &e)
      {
        std::cerr << "WebSocket JSON parse error: " << e.what() << std::endl;
      }
    }
    else
    {
      // Handle binary camera data
      if (expecting_camera_binary_ && binary_frames_received_ < binary_frames_expected_)
      {
        if (binary_frames_received_ == 0)
        {
          // First binary frame is RGB JPEG data
          processCameraRGBData(msg->str);
          binary_frames_received_++;
        }
        else if (binary_frames_received_ == 1)
        {
          // Second binary frame is depth data (if present)
          processCameraDepthData(msg->str);
          binary_frames_received_++;
        }

        if (binary_frames_received_ >= binary_frames_expected_)
        {
          expecting_camera_binary_ = false;
          pending_camera_frame_.clear();
        }
      }
    }
  }

  void processRobotDescription(const nlohmann::json &description_msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    std::cout << "Received robot description from server" << std::endl;

    try
    {
      // Parse model dimensions
      if (description_msg.contains("dimensions"))
      {
        auto dims = description_msg["dimensions"];
        robot_description_.dimensions.nq = dims.value("nq", 0);
        robot_description_.dimensions.nv = dims.value("nv", 0);
      }

      // Parse joint information
      if (description_msg.contains("joints") && description_msg["joints"].is_array())
      {
        robot_description_.joints.clear();
        for (const auto &joint_data : description_msg["joints"])
        {
          RobotDescription::JointInfo joint_info;
          joint_info.name = joint_data.value("name", "");
          joint_info.joint_type = joint_data.value("joint_type", 0);
          joint_info.pos_adr = joint_data.value("pos_adr", -1);
          joint_info.vel_adr = joint_data.value("vel_adr", -1);
          joint_info.act_adr = joint_data.value("act_adr", -1);

          if (!joint_info.name.empty())
          {
            robot_description_.joints.push_back(joint_info);
          }
        }
      }

      // Parse sensor information
      if (description_msg.contains("sensors") && description_msg["sensors"].is_array())
      {
        robot_description_.sensors.clear();
        for (const auto &sensor_data : description_msg["sensors"])
        {
          RobotDescription::SensorInfo sensor_info;
          sensor_info.name = sensor_data.value("name", "");
          sensor_info.sensor_adr = sensor_data.value("sensor_adr", -1);

          if (!sensor_info.name.empty())
          {
            robot_description_.sensors.push_back(sensor_info);
          }
        }
      }

      description_received_ = true;
      std::cout << "Robot description parsed successfully: "
                << robot_description_.joints.size() << " joints, "
                << robot_description_.sensors.size() << " sensors" << std::endl;
    }
    catch (const std::exception &e)
    {
      std::cerr << "Error parsing robot description: " << e.what() << std::endl;
    }
  }

  void processTrackedBodies(const nlohmann::json &tracked_bodies_msg)
  {
    if (!ros_initialized_)
    {
      initializeROS();
    }

    if (!tracked_bodies_msg.contains("bodies") || !tracked_bodies_msg["bodies"].is_array())
    {
      return;
    }

    // Get simulation time from the message
    double sim_time = tracked_bodies_msg.value("time", 0.0);

    // Convert to ROS time
    auto ros_time = rclcpp::Time(static_cast<int64_t>(sim_time * 1e9), RCL_ROS_TIME);

    for (const auto &body_data : tracked_bodies_msg["bodies"])
    {
      if (!body_data.is_object()) continue;

      std::string body_name = body_data.value("name", "");
      int body_id = body_data.value("body_id", -1);
      std::string frame_id = body_data.value("frame_id", body_name);

      if (body_name.empty()) continue;

      // Initialize publisher if not exists
      if (tracked_body_publishers_.find(body_name) == tracked_body_publishers_.end())
      {
        initializeTrackedBodyPublisher(body_name, body_id, frame_id);
      }

      auto &body_pub = tracked_body_publishers_[body_name];

      // Extract position and orientation arrays
      if (!body_data.contains("position") || !body_data.contains("orientation")) continue;

      auto position = body_data["position"];
      auto orientation = body_data["orientation"];

      if (
        !position.is_array() || position.size() != 3 || !orientation.is_array() ||
        orientation.size() != 4)
      {
        continue;
      }

      // Create and publish PoseStamped message
      auto pose_msg = geometry_msgs::msg::PoseStamped();
      pose_msg.header.stamp = ros_time;
      pose_msg.header.frame_id = frame_id;  // Use frame_id from the message

      // Set position (x, y, z)
      pose_msg.pose.position.x = position[0].get<double>();
      pose_msg.pose.position.y = position[1].get<double>();
      pose_msg.pose.position.z = position[2].get<double>();

      // Set orientation (w, x, y, z - MuJoCo format)
      pose_msg.pose.orientation.w = orientation[0].get<double>();
      pose_msg.pose.orientation.x = orientation[1].get<double>();
      pose_msg.pose.orientation.y = orientation[2].get<double>();
      pose_msg.pose.orientation.z = orientation[3].get<double>();

      body_pub.pose_pub->publish(pose_msg);

      // Small delay to let publish do its work (matching your original C++ implementation)
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
  }

  void processCameraRGBData(const std::string &jpeg_data)
  {
    if (!ros_initialized_ || pending_camera_frame_.empty()) return;

    std::string cam_name = pending_camera_frame_.value("camera_name", "");
    auto it = camera_publishers_.find(cam_name);
    if (it == camera_publishers_.end()) return;

    auto &cam_pub = it->second;

    // Get current time
    auto now = node_->get_clock()->now();

    // Publish compressed image
    if (cam_pub.image_pub)
    {
      auto compressed_img = sensor_msgs::msg::CompressedImage();
      compressed_img.header.stamp = now;
      compressed_img.header.frame_id = cam_pub.frame_name;
      compressed_img.format = "jpeg";
      compressed_img.data.assign(jpeg_data.begin(), jpeg_data.end());
      cam_pub.image_pub->publish(compressed_img);
    }

    // Publish camera info
    if (cam_pub.camera_info_pub)
    {
      auto cam_info = sensor_msgs::msg::CameraInfo();
      cam_info.header.stamp = now;
      cam_info.header.frame_id = cam_pub.frame_name;
      cam_info.height = cam_pub.height;
      cam_info.width = cam_pub.width;

      // Simple pinhole camera model
      double fx = cam_pub.width / (2.0 * tan(cam_pub.fovy * M_PI / 360.0));
      double fy = fx;
      double cx = cam_pub.width / 2.0;
      double cy = cam_pub.height / 2.0;

      cam_info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
      cam_info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
      cam_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

      cam_pub.camera_info_pub->publish(cam_info);
    }
  }

  void processCameraDepthData(const std::string &depth_data)
  {
    if (!ros_initialized_ || pending_camera_frame_.empty()) return;

    std::string cam_name = pending_camera_frame_.value("camera_name", "");
    auto it = camera_publishers_.find(cam_name);
    if (it == camera_publishers_.end()) return;

    auto &cam_pub = it->second;

    if (!cam_pub.has_depth || !cam_pub.depth_pub) return;

    // Get current time
    auto now = node_->get_clock()->now();

    // Publish depth image
    auto depth_img = sensor_msgs::msg::Image();
    depth_img.header.stamp = now;
    depth_img.header.frame_id = cam_pub.frame_name;
    depth_img.height = cam_pub.height;
    depth_img.width = cam_pub.width;
    depth_img.encoding = pending_camera_frame_.value("depth_encoding", "32FC1");
    depth_img.is_bigendian = false;
    depth_img.step = cam_pub.width * sizeof(float);
    depth_img.data.assign(depth_data.begin(), depth_data.end());
    cam_pub.depth_pub->publish(depth_img);

    // Publish depth camera info
    if (cam_pub.depth_info_pub)
    {
      auto cam_info = sensor_msgs::msg::CameraInfo();
      cam_info.header.stamp = now;
      cam_info.header.frame_id = cam_pub.frame_name;
      cam_info.height = cam_pub.height;
      cam_info.width = cam_pub.width;

      // Simple pinhole camera model
      double fx = cam_pub.width / (2.0 * tan(cam_pub.fovy * M_PI / 360.0));
      double fy = fx;
      double cx = cam_pub.width / 2.0;
      double cy = cam_pub.height / 2.0;

      cam_info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
      cam_info.p = {fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0};
      cam_info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

      cam_pub.depth_info_pub->publish(cam_info);
    }
  }

  ix::WebSocket ws_;
  mutable std::mutex state_mutex_;
  bool connected_ = false;
  bool state_received_ = false;
  bool description_received_ = false;
  RobotDescription robot_description_;

  // Cached state
  std::vector<double> cached_qpos_;
  std::vector<double> cached_qvel_;
  std::vector<double> cached_qfrc_applied_;
  std::vector<double> cached_sensor_data_;
  double cached_time_ = 0.0;
  OdometryData cached_odom_data_;  // New: cached odometry data

  // Camera frame processing
  nlohmann::json pending_camera_frame_;
  bool expecting_camera_binary_ = false;
  int binary_frames_expected_ = 0;
  int binary_frames_received_ = 0;

  // Centralized ROS members
  bool ros_initialized_ = false;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  std::map<std::string, CameraPublisher> camera_publishers_;
  std::map<std::string, TrackedBodyPublisher>
    tracked_body_publishers_;  // New: tracked body publishers
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
  OdometryData odom_data;

  if (!wsManager.getLatestState(qpos, qvel, qfrc_applied, sensordata, sim_time, odom_data))
  {
    // No fresh state, but that's OK - we'll use previous values
    return hardware_interface::return_type::OK;
  }

  // Update joint states from WebSocket data
  for (size_t i = 0; i < joint_states_.size(); ++i)
  {
    if (static_cast<size_t>(joint_states_[i].mj_pos_adr) < qpos.size())
    {
      joint_states_[i].position = qpos[joint_states_[i].mj_pos_adr];
    }

    if (static_cast<size_t>(joint_states_[i].mj_vel_adr) < qvel.size())
    {
      joint_states_[i].velocity = qvel[joint_states_[i].mj_vel_adr];
    }

    if (static_cast<size_t>(joint_states_[i].mj_vel_adr) < qfrc_applied.size())
    {
      joint_states_[i].effort = qfrc_applied[joint_states_[i].mj_vel_adr];
    }
  }

  // Update FT sensor data
  for (auto &data : ft_sensor_data_)
  {
    if (static_cast<size_t>(data.force.mj_sensor_index + 2) < sensordata.size())
    {
      data.force.data.x() = -sensordata[data.force.mj_sensor_index];
      data.force.data.y() = -sensordata[data.force.mj_sensor_index + 1];
      data.force.data.z() = -sensordata[data.force.mj_sensor_index + 2];
    }

    if (static_cast<size_t>(data.torque.mj_sensor_index + 2) < sensordata.size())
    {
      data.torque.data.x() = -sensordata[data.torque.mj_sensor_index];
      data.torque.data.y() = -sensordata[data.torque.mj_sensor_index + 1];
      data.torque.data.z() = -sensordata[data.torque.mj_sensor_index + 2];
    }
  }

  // Use centralized odometry publishing with actual data
  wsManager.publishOdometry(time, odom_data);

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
  // Store for compatibility but will be removed later
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

  RCLCPP_INFO(logger_, "Connected to WebSocket server");

  // Wait for robot description
  while (!wsManager.hasRobotDescription())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RCLCPP_INFO(logger_, "Robot description received");

  register_joints(urdf_model, hardware_info);
  register_sensors(urdf_model, hardware_info);

  // Get robot description for reset command
  const auto &robot_desc = wsManager.getRobotDescription();

  // Send initial reset command with initial joint positions
  nlohmann::json reset_cmd;
  reset_cmd["type"] = "reset";
  reset_cmd["state"]["qpos"] = std::vector<double>(robot_desc.dimensions.nq, 0.0);
  reset_cmd["state"]["qvel"] = std::vector<double>(robot_desc.dimensions.nv, 0.0);

  // Set initial positions from joint_states_
  for (const auto &joint_state : joint_states_)
  {
    if (joint_state.mj_pos_adr < robot_desc.dimensions.nq && joint_state.mj_pos_adr >= 0)
    {
      reset_cmd["state"]["qpos"][joint_state.mj_pos_adr] = joint_state.position;
    }
  }

  wsManager.sendCommand(reset_cmd);

  RCLCPP_INFO(
    logger_,
    "MujocoSystem initialization complete (WebSocket mode with camera, tracked bodies, and "
    "odometry support)");

  return true;
}

void MujocoSystem::register_joints(
  const urdf::Model &urdf_model, const hardware_interface::HardwareInfo &hardware_info)
{
  joint_states_.resize(hardware_info.joints.size());

  // Get robot description from WebSocket manager
  auto &wsManager = WebSocketManager::getInstance();
  const auto &robot_desc = wsManager.getRobotDescription();

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    int joint_desc_index = robot_desc.findJointIndex(joint.name);
    if (joint_desc_index == -1)
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find joint in robot description, joint name: " << joint.name);
      continue;
    }

    const auto &joint_info = robot_desc.joints[joint_desc_index];

    // save information in joint_states_ variable
    JointState joint_state;
    joint_state.name = joint.name;
    joint_state.mj_joint_type = joint_info.joint_type;
    joint_state.mj_pos_adr = joint_info.pos_adr;
    joint_state.mj_vel_adr = joint_info.vel_adr;
    joint_state.mj_act_adr = joint_info.act_adr;
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

  // Get robot description from WebSocket manager
  auto &wsManager = WebSocketManager::getInstance();
  const auto &robot_desc = wsManager.getRobotDescription();

  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);

    FTSensorData sensor_data;
    sensor_data.name = sensor.name;
    sensor_data.force.name = sensor.name + "_force";
    sensor_data.torque.name = sensor.name + "_torque";

    int force_sensor_index = robot_desc.findSensorIndex(sensor_data.force.name);
    int torque_sensor_index = robot_desc.findSensorIndex(sensor_data.torque.name);

    if (force_sensor_index == -1 || torque_sensor_index == -1)
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Failed to find sensor in robot description, sensor name: " << sensor.name);
      continue;
    }

    sensor_data.force.mj_sensor_index = robot_desc.sensors[force_sensor_index].sensor_adr;
    sensor_data.torque.mj_sensor_index = robot_desc.sensors[torque_sensor_index].sensor_adr;

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