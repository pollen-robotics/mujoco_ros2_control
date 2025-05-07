#pragma once

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GL/gl.h>
#include <mujoco/mujoco.h>
#include <atomic>
#include <vector>

// Forward-declare uWS
namespace uWS
{
template <bool, bool, typename>
class WebSocket;
}

namespace mujoco_ros2_control
{

struct PerSocketData
{
};

class MujocoRendering
{
public:
  static MujocoRendering *get_instance();
  MujocoRendering();

  void init(mjModel *mujoco_model, mjData *mujoco_data);
  void update();
  void close();
  bool is_close_flag_raised();

  void inject_mouse_move(double dx, double dy);
  void inject_mouse_buttons(bool left, bool middle, bool right);
  void inject_scroll(double dy);

private:
  static MujocoRendering *instance_;

  // WebSocket
  static uWS::WebSocket<false, true, PerSocketData> *ws_client_;
  static std::atomic<bool> ws_connected_;

  std::vector<unsigned char> read_pixels(int width, int height);
  std::vector<unsigned char> encode_jpeg(
    const std::vector<unsigned char> &rgb, int width, int height);

  // GLFW/MuJoCo camera state
  mjModel *mj_model_;
  mjData *mj_data_;
  mjvCamera mjv_cam_;
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  bool button_left_;
  bool button_middle_;
  bool button_right_;
  double lastx_;
  double lasty_;
  EGLDisplay egl_display_ = EGL_NO_DISPLAY;
  EGLContext egl_context_ = EGL_NO_CONTEXT;
  EGLSurface egl_surface_ = EGL_NO_SURFACE;
  int fb_width_ = 1200;
  int fb_height_ = 900;
};

}  // namespace mujoco_ros2_control
