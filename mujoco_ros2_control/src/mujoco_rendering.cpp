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
              // viewer->inject_mouse_move(dx, dy);
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

  egl_display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  if (!eglInitialize(egl_display_, nullptr, nullptr))
  {
    std::cerr << "EGL initialization failed\n";
    std::exit(1);
  }

  const EGLint config_attribs[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE,
                                   EGL_OPENGL_BIT,   EGL_DEPTH_SIZE,  24,  // ✅ ensure depth
                                   EGL_NONE};
  EGLConfig egl_config;
  EGLint num_configs;
  if (!eglChooseConfig(egl_display_, config_attribs, &egl_config, 1, &num_configs))
  {
    std::cerr << "EGL config selection failed\n";
    std::exit(1);
  }

  const EGLint pbuffer_attribs[] = {EGL_WIDTH, fb_width_, EGL_HEIGHT, fb_height_, EGL_NONE};
  egl_surface_ = eglCreatePbufferSurface(egl_display_, egl_config, pbuffer_attribs);
  eglBindAPI(EGL_OPENGL_API);
  const EGLint ctx_attribs[] = {
    EGL_CONTEXT_CLIENT_VERSION, 2,  // or omit
    EGL_NONE};
  egl_context_ = eglCreateContext(egl_display_, egl_config, EGL_NO_CONTEXT, ctx_attribs);
  eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_);
  glEnable(GL_DEPTH_TEST);

  // Force OpenGL to be ready
  glClearColor(0.2f, 0.4f, 0.6f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glFinish();  // Ensure all commands have been processed
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.lookat[0] = 0;
  mjv_cam_.lookat[1] = 0;
  mjv_cam_.lookat[2] = 1.0;
  mjv_cam_.distance = 3.0;

  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);
  mjr_resizeOffscreen(fb_width_, fb_height_, &mjr_con_);
}

bool MujocoRendering::is_close_flag_raised() { return false; }
void MujocoRendering::update()
{
  static uint32_t frame_id = 0;

  eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_);
  glViewport(0, 0, fb_width_, fb_height_);
  glEnable(GL_DEPTH_TEST);

  // Set the offscreen buffer and render
  mjr_setBuffer(mjFB_OFFSCREEN, &mjr_con_);
  mj_step(mj_model_, mj_data_);
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, nullptr, &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render({0, 0, fb_width_, fb_height_}, &mjv_scn_, &mjr_con_);
  // glFinish();
  glFlush();

  // Allocate RGB buffer and read pixels using MuJoCo
  std::vector<unsigned char> rgb(3 * fb_width_ * fb_height_);
  mjr_readPixels(rgb.data(), nullptr, {0, 0, fb_width_, fb_height_}, &mjr_con_);

  // Flip vertically (OpenGL origin is bottom-left)
  std::vector<unsigned char> flipped(3 * fb_width_ * fb_height_);
  for (int y = 0; y < fb_height_; ++y)
  {
    std::memcpy(
      &flipped[y * fb_width_ * 3], &rgb[(fb_height_ - 1 - y) * fb_width_ * 3], fb_width_ * 3);
  }

  // Compress and send
  auto jpeg = encode_jpeg(flipped, fb_width_, fb_height_);
  uint32_t net_id = htonl(frame_id++);
  std::vector<unsigned char> framed;
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

void MujocoRendering::close()
{
  // Free MuJoCo visualization resources
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);

  // Destroy EGL surface and context
  if (egl_surface_ != EGL_NO_SURFACE)
  {
    eglDestroySurface(egl_display_, egl_surface_);
    egl_surface_ = EGL_NO_SURFACE;
  }

  if (egl_context_ != EGL_NO_CONTEXT)
  {
    eglDestroyContext(egl_display_, egl_context_);
    egl_context_ = EGL_NO_CONTEXT;
  }

  if (egl_display_ != EGL_NO_DISPLAY)
  {
    eglTerminate(egl_display_);
    egl_display_ = EGL_NO_DISPLAY;
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

// // void MujocoRendering::inject_mouse_move(double dx, double dy)
// // {
// //   int width, height;
// //   glfwGetWindowSize(window_, &width, &height);

// //   mjtMouse action;
// //   bool mod_shift = false;  // You could add support for Shift key too

// //   if (button_right_)
// //     action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
// //   else if (button_left_)
// //     action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
// //   else
// //     action = mjMOUSE_ZOOM;

// //   mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
// // }

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
