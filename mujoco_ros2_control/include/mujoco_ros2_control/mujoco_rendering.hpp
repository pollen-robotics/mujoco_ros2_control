#pragma once

#include <GLFW/glfw3.h>
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
  GLFWwindow *window_;

  bool button_left_;
  bool button_middle_;
  bool button_right_;
  double lastx_;
  double lasty_;

  // Input callbacks
  static void keyboard_callback(GLFWwindow *, int, int, int, int);
  static void mouse_button_callback(GLFWwindow *, int, int, int);
  static void mouse_move_callback(GLFWwindow *, double, double);
  static void scroll_callback(GLFWwindow *, double, double);

  void keyboard_callback_impl(GLFWwindow *, int, int, int, int);
  void mouse_button_callback_impl(GLFWwindow *, int, int, int);
  void mouse_move_callback_impl(GLFWwindow *, double, double);
  void scroll_callback_impl(GLFWwindow *, double, double);
};

}  // namespace mujoco_ros2_control
