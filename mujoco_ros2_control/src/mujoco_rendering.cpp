#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace mujoco_ros2_control
{
MujocoRendering *MujocoRendering::instance_ = nullptr;

MujocoRendering *MujocoRendering::get_instance()
{
  if (instance_ == nullptr)
  {
    instance_ = new MujocoRendering();
  }
  return instance_;
}

MujocoRendering::MujocoRendering() : mj_model_(nullptr), mj_data_(nullptr) {}

void MujocoRendering::init(
  rclcpp::Node::SharedPtr &node, mjModel *mujoco_model, mjData *mujoco_data)
{
  node_ = node;
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;
  image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>("/mujoco_camera/image_raw", 10);

  // Initialize GLFW window for scene rendering
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }

  for (int i = 0; i < mj_model_->ncam; i++)
  {
    std::string cam_name = std::string(mj_model_->names + mj_model_->name_camadr[i]);
    std::string topic_name = "/mujoco_camera/" + cam_name + "/image_raw";

    auto pub = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    camera_publishers_.push_back(pub);

    // same for depth
    std::string depth_topic_name = "/mujoco_camera/" + cam_name + "/depth";
    auto depth_pub = node_->create_publisher<sensor_msgs::msg::Image>(depth_topic_name, 10);
    depth_publishers_.push_back(depth_pub);

    std::cout << "Created publisher for camera: " << cam_name << " -> Topic: " << topic_name
              << std::endl;
  }

  std::cout << "Number of cameras: " << camera_publishers_.size() << std::endl;

  window_ = glfwCreateWindow(1200, 900, "MuJoCo Scene View", NULL, NULL);
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  // Initialize MuJoCo visualization components
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);
  // Install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);
}

void MujocoRendering::mouse_button_callback(GLFWwindow *window, int button, int act, int mods)
{
  get_instance()->mouse_button_callback_impl(window, button, act, mods);
}

void MujocoRendering::mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
{
  get_instance()->mouse_move_callback_impl(window, xpos, ypos);
}

void MujocoRendering::scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
{
  get_instance()->scroll_callback_impl(window, xoffset, yoffset);
}
void MujocoRendering::keyboard_callback(
  GLFWwindow *window, int key, int scancode, int act, int mods)
{
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
}

void MujocoRendering::keyboard_callback_impl(
  GLFWwindow *window, int key, int scancode, int act, int mods)
{
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }
}
void MujocoRendering::mouse_button_callback_impl(GLFWwindow *window, int button, int act, int mods)
{
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &lastx_, &lasty_);

  if (button_left_ && act == GLFW_PRESS)
  {
    // Get window size
    int width, height;
    glfwGetWindowSize(window_, &width, &height);
    mjtNum aspectratio = (mjtNum)width / (mjtNum)height;

    // Convert cursor position to normalized screen coordinates
    mjtNum relx = lastx_ / (mjtNum)width;
    mjtNum rely = 1.0 - lasty_ / (mjtNum)height;  // Invert y-axis

    // Variables to store selection results
    mjtNum selpos[3];
    int geomid = -1, skinid = -1, bodyid = -1;

    // Perform selection
    mjv_select(
      mj_model_, mj_data_, &mjv_opt_, aspectratio, relx, rely, &mjv_scn_, selpos, &geomid, &skinid,
      &bodyid);

    if (geomid >= 0 && bodyid > 0)  // Ensure it's a dynamic object
    {
      selected_body_ = bodyid;
      selected_pos_[0] = selpos[0];
      selected_pos_[1] = selpos[1];
      selected_pos_[2] = selpos[2];
    }
  }

  if (button_left_ && act == GLFW_RELEASE)
  {
    selected_body_ = -1;
  }
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos)
{
  if (!button_left_ && !button_middle_ && !button_right_) return;

  double dx = xpos - lastx_;
  double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

  int width, height;
  glfwGetWindowSize(window_, &width, &height);
  bool mod_shift =
    (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  if (selected_body_ >= 0)
  {
    // Move selected object by modifying qpos
    mjtNum dpos[3] = {dx / width, -dy / height, 0};

    int qposadr = mj_model_->jnt_qposadr[mj_model_->body_jntadr[selected_body_]];
    mj_data_->qpos[qposadr] += dpos[0];
    mj_data_->qpos[qposadr + 1] += dpos[1];
    mj_data_->qpos[qposadr + 2] += dpos[2];

    mj_forward(mj_model_, mj_data_);
  }
  else
  {
    // Normal camera movement
    mjtMouse action;
    if (button_right_)
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if (button_left_)
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
      action = mjMOUSE_ZOOM;

    mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
  }
}

void MujocoRendering::scroll_callback_impl(GLFWwindow *window, double xoffset, double yoffset)
{
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::capture_and_publish_image()
{
  if (!mj_model_ || !mj_data_) return;

  mjrRect viewport = {0, 0, 640, 480};
  std::vector<unsigned char> rgb(viewport.width * viewport.height * 3);

  // Capture only the scene rendering, without an embedded camera view
  mjr_readPixels(rgb.data(), nullptr, viewport, &mjr_con_);

  cv::Mat img(viewport.height, viewport.width, CV_8UC3, rgb.data());
  cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

  sensor_msgs::msg::Image ros_img;
  ros_img.header.stamp = node_->now();
  ros_img.header.frame_id = "mujoco_camera";
  ros_img.height = viewport.height;
  ros_img.width = viewport.width;
  ros_img.encoding = "bgr8";
  ros_img.is_bigendian = false;
  ros_img.step = 3 * viewport.width;
  ros_img.data.assign(img.data, img.data + img.total() * img.elemSize());

  image_pub_->publish(ros_img);
}

void MujocoRendering::capture_and_publish_cameras()
{
  for (int i = 3; i < mj_model_->ncam; i++)
  {
    std::string cam_name = std::string(mj_model_->names + mj_model_->name_camadr[i]);
    std::cout << "Camera " << i << " Name: " << cam_name << std::endl;
    std::cout << "--------------------------------------" << std::endl;

    std::cout << "  cam_mode: " << mj_model_->cam_mode[i] << std::endl;
    std::cout << "  cam_bodyid: " << mj_model_->cam_bodyid[i] << std::endl;
    // get body id position
    std::cout << "  cam_bodyid position: [" << mj_model_->body_pos[3 * mj_model_->cam_bodyid[i]]
              << ", " << mj_model_->body_pos[3 * mj_model_->cam_bodyid[i] + 1] << ", "
              << mj_model_->body_pos[3 * mj_model_->cam_bodyid[i] + 2] << "]" << std::endl;

    std::cout << "  cam_targetbodyid: " << mj_model_->cam_targetbodyid[i] << std::endl;

    std::cout << "  cam_pos: [" << mj_model_->cam_pos[3 * i] << ", "
              << mj_model_->cam_pos[3 * i + 1] << ", " << mj_model_->cam_pos[3 * i + 2] << "]"
              << std::endl;

    std::cout << "  cam_quat: [" << mj_model_->cam_quat[4 * i] << ", "
              << mj_model_->cam_quat[4 * i + 1] << ", " << mj_model_->cam_quat[4 * i + 2] << ", "
              << mj_model_->cam_quat[4 * i + 3] << "]" << std::endl;

    std::cout << "  cam_poscom0: [" << mj_model_->cam_poscom0[3 * i] << ", "
              << mj_model_->cam_poscom0[3 * i + 1] << ", " << mj_model_->cam_poscom0[3 * i + 2]
              << "]" << std::endl;

    std::cout << "  cam_pos0: [" << mj_model_->cam_pos0[3 * i] << ", "
              << mj_model_->cam_pos0[3 * i + 1] << ", " << mj_model_->cam_pos0[3 * i + 2] << "]"
              << std::endl;

    std::cout << "  cam_mat0: [";
    for (int j = 0; j < 9; j++)
    {
      std::cout << mj_model_->cam_mat0[9 * i + j] << (j < 8 ? ", " : "");
    }
    std::cout << "]" << std::endl;

    std::cout << "  cam_orthographic: " << mj_model_->cam_orthographic[i] << std::endl;
    std::cout << "  cam_fovy: " << mj_model_->cam_fovy[i] << std::endl;
    std::cout << "  cam_ipd: " << mj_model_->cam_ipd[i] << std::endl;

    std::cout << "  cam_resolution: [" << mj_model_->cam_resolution[2 * i] << ", "
              << mj_model_->cam_resolution[2 * i + 1] << "]" << std::endl;

    std::cout << "  cam_sensorsize: [" << mj_model_->cam_sensorsize[2 * i] << ", "
              << mj_model_->cam_sensorsize[2 * i + 1] << "]" << std::endl;

    std::cout << "  cam_intrinsic: [" << mj_model_->cam_intrinsic[4 * i] << ", "
              << mj_model_->cam_intrinsic[4 * i + 1] << ", " << mj_model_->cam_intrinsic[4 * i + 2]
              << ", " << mj_model_->cam_intrinsic[4 * i + 3] << "]" << std::endl;

    std::cout << "  cam_user: [";
    for (int j = 0; j < mj_model_->nuser_cam; j++)
    {
      std::cout << mj_model_->cam_user[mj_model_->nuser_cam * i + j]
                << (j < mj_model_->nuser_cam - 1 ? ", " : "");
    }
    std::cout << "]" << std::endl;

    // Get the absolute position of the camera
    mjtNum *cam_pos = mj_data_->xpos + 3 * mj_model_->cam_bodyid[i];

    // Get the absolute orientation (quaternion)
    mjtNum *cam_quat = mj_data_->xquat + 4 * mj_model_->cam_bodyid[i];

    std::cout << "Camera " << i << " Position: [" << cam_pos[0] << ", " << cam_pos[1] << ", "
              << cam_pos[2] << "]" << std::endl;

    std::cout << "Camera " << i << " Orientation (Quat): [" << cam_quat[0] << ", " << cam_quat[1]
              << ", " << cam_quat[2] << ", " << cam_quat[3] << "]" << std::endl;

    std::cout << "======================================" << std::endl;
  }

  // Force crash or exit after printing
  // std::cerr << "Debug print complete. Exiting program." << std::endl;
  // exit(1);

  mjrRect viewport = {0, 0, 640, 480};
  std::vector<unsigned char> rgb(viewport.width * viewport.height * 3);
  std::vector<float> depth(viewport.width * viewport.height);

  for (int i = 0; i < mj_model_->ncam; i++)
  {
    // Set up a temporary MuJoCo camera
    mjvCamera temp_cam;
    mjv_defaultCamera(&temp_cam);

    // Get camera position
    temp_cam.lookat[0] = mj_model_->cam_pos[3 * i];
    temp_cam.lookat[1] = mj_model_->cam_pos[3 * i + 1];
    temp_cam.lookat[2] = mj_model_->cam_pos[3 * i + 2];

    // print camera position
    // std::cout << "Camera position: " << temp_cam.lookat[0] << " " << temp_cam.lookat[1] << " "
    //           << temp_cam.lookat[2] << std::endl;

    // Extract the camera's orientation (forward vector) from cam_mat0
    mjtNum forward_x = mj_model_->cam_mat0[9 * i + 6];
    mjtNum forward_y = mj_model_->cam_mat0[9 * i + 7];
    mjtNum forward_z = mj_model_->cam_mat0[9 * i + 8];

    // print the forwards
    // std::cout << "Camera forward: " << forward_x << " " << forward_y << " " << forward_z
    //           << std::endl;

    // Set camera distance (adjust as needed)
    temp_cam.distance = 1.0;

    // Compute the final look-at position by moving along the camera's forward vector
    // temp_cam.lookat[0] -= temp_cam.distance * forward_x;
    // temp_cam.lookat[1] -= temp_cam.distance * forward_y;
    // temp_cam.lookat[2] -= temp_cam.distance * forward_z;
    // temp_cam.lookat[0] -= temp_cam.distance ;
    // temp_cam.lookat[1] -= temp_cam.distance ;
    // temp_cam.lookat[2] -= temp_cam.distance ;

    // new
    //

    // // // // // // // // // // // // // // Get the absolute position of the camera
    // // // // // // // // // // // // // mjtNum *cam_pos = mj_data_->xpos + 3 *
    // mj_model_->cam_bodyid[i];

    // // // // // // // // // // // // // // Get the absolute orientation (quaternion)
    // // // // // // // // // // // // // mjtNum *cam_quat = mj_data_->xquat + 4 *
    // mj_model_->cam_bodyid[i];
    // // // // // // // // // // // // // // mjvCamera temp_cam;
    // // // // // // // // // // // // // // mjv_defaultCamera(&temp_cam);

    // // // // // // // // // // // // // // Set camera position in world frame
    // // // // // // // // // // // // // temp_cam.pos[0] = cam_pos[0];
    // // // // // // // // // // // // // temp_cam.pos[1] = cam_pos[1];
    // // // // // // // // // // // // // temp_cam.pos[2] = cam_pos[2];

    // // // // // // // // // // // // // // Compute forward direction (-Z in local frame rotated
    // to world)
    // // // // // // // // // // // // // mjtNum forward_local[3] = {0, 0, -1};  // MuJoCo's
    // default camera direction
    // // // // // // // // // // // // // mjtNum forward_world[3];
    // // // // // // // // // // // // // mju_rotVecQuat(forward_world, forward_local, cam_quat);

    // // // // // // // // // // // // // // Compute look-at position
    // // // // // // // // // // // // // temp_cam.lookat[0] = cam_pos[0] + forward_world[0];
    // // // // // // // // // // // // // temp_cam.lookat[1] = cam_pos[1] + forward_world[1];
    // // // // // // // // // // // // // temp_cam.lookat[2] = cam_pos[2] + forward_world[2];

    // // // // // // // // // // // // // // std::cout << "Camera " << i << " Position: [" <<
    // cam_pos[0] << ", " << cam_pos[1] << ", "
    // // // // // // // // // // // // // //           << cam_pos[2] << "]" << std::endl;

    // // // // // // // // // // // // // // std::cout << "Camera " << i << " Look-at Position: ["
    // << temp_cam.lookat[0] << ", "
    // // // // // // // // // // // // // //           << temp_cam.lookat[1] << ", " <<
    // temp_cam.lookat[2] << "]" << std::endl;

    // renew

    // Retrieve camera position
    mjtNum *cam_pos = mj_model_->cam_pos + 3 * i;

    // Compute camera "look-at" position using cam_mat0
    mjtNum lookat[3] = {
      cam_pos[0] + mj_model_->cam_mat0[9 * i + 6],  // X forward
      cam_pos[1] + mj_model_->cam_mat0[9 * i + 7],  // Y forward
      cam_pos[2] + mj_model_->cam_mat0[9 * i + 8]   // Z forward
    };

    // Set up a MuJoCo visualization camera
    // mjvCamera temp_cam;
    // mjv_defaultCamera(&temp_cam);

    temp_cam.type = mjCAMERA_FIXED;  // Use the pre-defined MuJoCo camera
    temp_cam.fixedcamid = i;         // Set camera ID
    temp_cam.lookat[0] = lookat[0];
    temp_cam.lookat[1] = lookat[1];
    temp_cam.lookat[2] = lookat[2];
    temp_cam.distance = 0.1;  // Ensure the camera is properly positioned
    temp_cam.azimuth = 0;     // Align azimuth
    temp_cam.elevation = 0;   // Align elevation

    // renew
    // Render from this camera
    mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, nullptr, &temp_cam, mjCAT_ALL, &mjv_scn_);
    mjr_render(viewport, &mjv_scn_, &mjr_con_);
    mjr_readPixels(rgb.data(), depth.data(), viewport, &mjr_con_);

    // Convert to OpenCV image
    cv::Mat img(viewport.height, viewport.width, CV_8UC3, rgb.data());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::flip(img, img, 0);  // Fix framebuffer flip

    // Convert to ROS2 Image message
    sensor_msgs::msg::Image ros_img;
    ros_img.header.stamp = node_->now();
    ros_img.header.frame_id = "mujoco_camera_" + std::to_string(i);
    ros_img.height = viewport.height;
    ros_img.width = viewport.width;
    ros_img.encoding = "bgr8";
    ros_img.is_bigendian = false;
    ros_img.step = 3 * viewport.width;
    ros_img.data.assign(img.data, img.data + img.total() * img.elemSize());

    // Publish the camera image
    camera_publishers_[i]->publish(ros_img);

    //////////////////////DEPTH

    // Define depth range for visualization
    const float depth_min = 0.0f;   // Closest point (pure white)
    const float depth_max = 10.0f;  // Anything >=10m is black

    // ✅ Compute Depth Image in Meters
    cv::Mat depth_img(viewport.height, viewport.width, CV_32FC1);
    for (int y = 0; y < viewport.height; y++)
    {
      for (int x = 0; x < viewport.width; x++)
      {
        int index = y * viewport.width + x;
        float depth_val = depth[index];  // Raw MuJoCo inverse depth

        // Convert inverse MuJoCo depth to real-world meters (Avoid divide by zero)
        float real_depth = 1.0f / std::max(depth_val, 1e-6f);

        // Clip to max visualization range (e.g., 10m)
        depth_img.at<float>(y, x) = std::min(real_depth, 10.0f);
      }
    }

    // ✅ Flip image vertically for correct orientation
    cv::flip(depth_img, depth_img, 0);

    // ✅ **Fix RViz Error: Ensure Correct Data Size in `ros_depth`**
    sensor_msgs::msg::Image ros_depth;
    ros_depth.header.stamp = node_->now();
    ros_depth.header.frame_id = "mujoco_camera_" + std::to_string(i) + "_depth";
    ros_depth.height = depth_img.rows;
    ros_depth.width = depth_img.cols;
    ros_depth.encoding = "32FC1";  // ✅ Correct for RViz real depth visualization
    ros_depth.is_bigendian = false;
    ros_depth.step = depth_img.cols * sizeof(float);           // ✅ Fix: Ensure correct `step` size
    ros_depth.data.resize(ros_depth.step * ros_depth.height);  // ✅ Fix: Ensure correct total size
    memcpy(
      ros_depth.data.data(), depth_img.data,
      ros_depth.step * ros_depth.height);  // ✅ Copy raw bytes correctly

    // Publish real-world depth (in meters)
    depth_publishers_[i]->publish(ros_depth);
  }
}

void MujocoRendering::update()
{
  // Render the MuJoCo scene
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, nullptr, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);
  glfwSwapBuffers(window_);
  glfwPollEvents();

  // Publish camera image without embedding the camera view inside the MuJoCo scene
  // capture_and_publish_image();
  capture_and_publish_cameras();
}

bool MujocoRendering::is_close_flag_raised() { return glfwWindowShouldClose(window_); }

void MujocoRendering::close()
{
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  glfwDestroyWindow(window_);
  glfwTerminate();
}
}  // namespace mujoco_ros2_control
