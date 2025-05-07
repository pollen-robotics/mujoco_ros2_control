// Copyright (c) 2025 Sangtaek Lee
// SPDX-License-Identifier: MIT

#include "mujoco_ros2_control/mujoco_rendering.hpp"
#include "sensor_msgs/image_encodings.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <arpa/inet.h>  // For htonl
#include <uwebsockets/App.h>
#include <atomic>
#include <cstring>
#include <nlohmann/json.hpp>
#include <thread>
#include <vector>

namespace mujoco_ros2_control
{

// Static member definitions
MujocoRendering *MujocoRendering::instance_ = nullptr;
uWS::WebSocket<false, true, PerSocketData> *MujocoRendering::ws_client_ = nullptr;
std::atomic<bool> MujocoRendering::ws_connected_ = false;

MujocoRendering *MujocoRendering::get_instance()
{
  if (instance_ == nullptr)
  {
    instance_ = new MujocoRendering();
  }
  return instance_;
}

MujocoRendering::MujocoRendering()
    : mj_model_(nullptr),
      mj_data_(nullptr),
      button_left_(false),
      button_middle_(false),
      button_right_(false),
      lastx_(0.0),
      lasty_(0.0)
{
  // Launch WebSocket server thread
  std::thread(
    []()
    {
      uWS::App::WebSocketBehavior<PerSocketData> behavior;

      behavior.open = [](auto *ws)
      {
        MujocoRendering::ws_client_ = ws;
        MujocoRendering::ws_connected_ = true;
        std::cout << "[WS] Client connected\n";
      };

      behavior.close = [](auto *, int, std::string_view)
      {
        MujocoRendering::ws_connected_ = false;
        std::cout << "[WS] Client disconnected\n";
      };

      behavior.message = [](auto *ws, std::string_view message, uWS::OpCode opCode)
      {
        if (opCode == uWS::OpCode::TEXT)
        {
          try
          {
            auto j = nlohmann::json::parse(message);

            auto *viewer = MujocoRendering::get_instance();
            if (j["type"] == "mouse_move")
            {
              double dx = j["dx"];
              double dy = j["dy"];
              viewer->inject_mouse_move(dx, dy);
            }
            else if (j["type"] == "mouse_button")
            {
              viewer->inject_mouse_buttons(j["left"], j["middle"], j["right"]);
            }
            else if (j["type"] == "scroll")
            {
              double dy = j["dy"];
              viewer->inject_scroll(dy);
            }
          }
          catch (...)
          {
            std::cerr << "Failed to parse control message\n";
          }
        }
      };
      uWS::App()
        .ws<PerSocketData>("/*", std::move(behavior))  // ✅ fix here
        .listen(
          9002,
          [](auto *token)
          {
            if (token)
            {
              std::cout << "[WS] Listening on port 9002\n";
            }
          })
        .run();
    })
    .detach();
}

void MujocoRendering::init(mjModel *mujoco_model, mjData *mujoco_data)
{
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);

  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);
  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.distance = 8.;

  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);

  glfwSwapInterval(0);
}

bool MujocoRendering::is_close_flag_raised() { return glfwWindowShouldClose(window_); }

void MujocoRendering::update()
{
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  glfwMakeContextCurrent(window_);
  mjr_setBuffer(mjFB_WINDOW, &mjr_con_);

  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, NULL, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);
  glfwSwapBuffers(window_);
  glFinish();

  glfwPollEvents();

  static uint32_t frame_id = 0;

  auto raw = read_pixels(viewport.width, viewport.height);
  auto jpeg = encode_jpeg(raw, viewport.width, viewport.height);

  uint32_t net_id = htonl(frame_id++);  // Convert to network byte order
  std::vector<unsigned char> framed;
  framed.reserve(4 + jpeg.size());
  framed.insert(
    framed.end(), reinterpret_cast<unsigned char *>(&net_id),
    reinterpret_cast<unsigned char *>(&net_id) + 4);
  framed.insert(framed.end(), jpeg.begin(), jpeg.end());

  if (ws_connected_ && ws_client_)
  {
    ws_client_->send(
      std::string_view(reinterpret_cast<char *>(framed.data()), framed.size()),
      uWS::OpCode::BINARY);
  }
}

std::vector<unsigned char> MujocoRendering::read_pixels(int width, int height)
{
  std::vector<unsigned char> pixels(3 * width * height);
  glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  std::vector<unsigned char> flipped(3 * width * height);
  for (int y = 0; y < height; ++y)
  {
    std::memcpy(&flipped[y * width * 3], &pixels[(height - 1 - y) * width * 3], width * 3);
  }
  return flipped;
}

std::vector<unsigned char> MujocoRendering::encode_jpeg(
  const std::vector<unsigned char> &rgb, int width, int height)
{
  std::vector<unsigned char> jpeg_buf;
  stbi_write_func *func = [](void *context, void *data, int size)
  {
    auto *out = static_cast<std::vector<unsigned char> *>(context);
    out->insert(out->end(), (unsigned char *)data, (unsigned char *)data + size);
  };
  stbi_write_jpg_to_func(func, &jpeg_buf, width, height, 3, rgb.data(), 90);
  return jpeg_buf;
}

void MujocoRendering::close()
{
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  glfwDestroyWindow(window_);

#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void MujocoRendering::keyboard_callback(
  GLFWwindow *window, int key, int scancode, int act, int mods)
{
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
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

void MujocoRendering::keyboard_callback_impl(GLFWwindow *, int key, int, int act, int)
{
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }
}

void MujocoRendering::mouse_button_callback_impl(GLFWwindow *window, int, int, int)
{
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &lastx_, &lasty_);
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow *window, double xpos, double ypos)
{
  if (!button_left_ && !button_middle_ && !button_right_) return;

  double dx = xpos - lastx_;
  double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift =
    (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;
  if (button_right_)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (button_left_)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::scroll_callback_impl(GLFWwindow *, double, double yoffset)
{
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::inject_mouse_move(double dx, double dy)
{
  int width, height;
  glfwGetWindowSize(window_, &width, &height);

  mjtMouse action;
  bool mod_shift = false;  // You could add support for Shift key too

  if (button_right_)
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  else if (button_left_)
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  else
    action = mjMOUSE_ZOOM;

  mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
}

void MujocoRendering::inject_mouse_buttons(bool left, bool middle, bool right)
{
  button_left_ = left;
  button_middle_ = middle;
  button_right_ = right;
}

void MujocoRendering::inject_scroll(double dy)
{
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.0005 * dy, &mjv_scn_, &mjv_cam_);
}

}  // namespace mujoco_ros2_control
