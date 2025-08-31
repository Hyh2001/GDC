#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <SDL.h>

using namespace std::chrono_literals;

class KeyboardNode : public rclcpp::Node
{
public:
  KeyboardNode() : Node("keyboard_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    timer = this->create_wall_timer(20ms, std::bind(&KeyboardNode::timer_callback, this));

  }

private:
  void timer_callback()
  {

    // --- Axis state ---
    static float axis_x = 0.0f; // LEFTX
    static float axis_y = 0.0f; // LEFTY
    static float right_x = 0.0f; // RIGHTX
    static float right_y = 0.0f; // RIGHTY
    static float trigger_left = 0.0f; // TRIGGERLEFT
    static float trigger_right = 0.0f; // TRIGGERRIGHT
    const float step = 0.1f;
    const float max_val = 1.0f;
    const float min_val = -1.0f;

    // --- Button state ---
    static bool button[21] = {false};
    // Key press state
    static bool left_pressed = false, right_pressed = false, up_pressed = false, down_pressed = false;
    static bool w_pressed = false, a_pressed = false, s_pressed = false, d_pressed = false;
    static bool i_pressed = false, j_pressed = false, k_pressed = false, l_pressed = false;
    static bool z_pressed = false, x_pressed = false;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch(event.type) {
        case SDL_KEYDOWN:
          switch (event.key.keysym.sym) {
            case SDLK_LEFT: left_pressed = true; break;
            case SDLK_RIGHT: right_pressed = true; break;
            case SDLK_UP: up_pressed = true; break;
            case SDLK_DOWN: down_pressed = true; break;
            case SDLK_w: w_pressed = true; break;
            case SDLK_a: a_pressed = true; break;
            case SDLK_s: s_pressed = true; break;
            case SDLK_d: d_pressed = true; break;
            case SDLK_i: i_pressed = true; break;
            case SDLK_j: j_pressed = true; break;
            case SDLK_k: k_pressed = true; break;
            case SDLK_l: l_pressed = true; break;
            case SDLK_z: z_pressed = true; break;
            case SDLK_x: x_pressed = true; break;
            case SDLK_SPACE: button[0] = true; break; // A
            case SDLK_LCTRL: button[1] = true; break; // B
            case SDLK_LALT: button[2] = true; break; // X
            case SDLK_LSHIFT: button[3] = true; break; // Y
            case SDLK_TAB: button[4] = true; break; // BACK
            case SDLK_F1: button[5] = true; break; // GUIDE
            case SDLK_RETURN: button[6] = true; break; // START
            case SDLK_c: button[7] = true; break; // LEFTSTICK
            case SDLK_v: button[8] = true; break; // RIGHTSTICK
            case SDLK_q: button[9] = true; break; // LEFTSHOULDER
            case SDLK_e: button[10] = true; break; // RIGHTSHOULDER
            case SDLK_t: button[11] = true; break; // DPAD_UP
            case SDLK_g: button[12] = true; break; // DPAD_DOWN
            case SDLK_f: button[13] = true; break; // DPAD_LEFT
            case SDLK_h: button[14] = true; break; // DPAD_RIGHT
            case SDLK_m: button[15] = true; break; // MISC1
            case SDLK_1: button[16] = true; break; // PADDLE1
            case SDLK_2: button[17] = true; break; // PADDLE2
            case SDLK_3: button[18] = true; break; // PADDLE3
            case SDLK_4: button[19] = true; break; // PADDLE4
            case SDLK_5: button[20] = true; break; // TOUCHPAD
            default: break;
          }
          break;
        case SDL_KEYUP:
          switch (event.key.keysym.sym) {
            case SDLK_LEFT: left_pressed = false; break;
            case SDLK_RIGHT: right_pressed = false; break;
            case SDLK_UP: up_pressed = false; break;
            case SDLK_DOWN: down_pressed = false; break;
            case SDLK_w: w_pressed = false; break;
            case SDLK_a: a_pressed = false; break;
            case SDLK_s: s_pressed = false; break;
            case SDLK_d: d_pressed = false; break;
            case SDLK_i: i_pressed = false; break;
            case SDLK_j: j_pressed = false; break;
            case SDLK_k: k_pressed = false; break;
            case SDLK_l: l_pressed = false; break;
            case SDLK_z: z_pressed = false; break;
            case SDLK_x: x_pressed = false; break;
            case SDLK_SPACE: button[0] = false; break;
            case SDLK_LCTRL: button[1] = false; break;
            case SDLK_LALT: button[2] = false; break;
            case SDLK_LSHIFT: button[3] = false; break;
            case SDLK_TAB: button[4] = false; break;
            case SDLK_F1: button[5] = false; break;
            case SDLK_RETURN: button[6] = false; break;
            case SDLK_c: button[7] = false; break;
            case SDLK_v: button[8] = false; break;
            case SDLK_q: button[9] = false; break;
            case SDLK_e: button[10] = false; break;
            case SDLK_t: button[11] = false; break;
            case SDLK_g: button[12] = false; break;
            case SDLK_f: button[13] = false; break;
            case SDLK_h: button[14] = false; break;
            case SDLK_m: button[15] = false; break;
            case SDLK_1: button[16] = false; break;
            case SDLK_2: button[17] = false; break;
            case SDLK_3: button[18] = false; break;
            case SDLK_4: button[19] = false; break;
            case SDLK_5: button[20] = false; break;
            default: break;
          }
          break;
        case SDL_QUIT:
          SDL_FreeSurface(window);
          SDL_Quit();
          timer->cancel();
          break;
      }
    }

    // LEFTX/LEFTY (arrow keys)
    if (left_pressed && !right_pressed) {
      axis_x -= step;
    } else if (right_pressed && !left_pressed) {
      axis_x += step;
    } else if (!left_pressed && !right_pressed) {
      // Gradually return to zero
      if (axis_x > 0) axis_x -= step;
      else if (axis_x < 0) axis_x += step;
      if (std::abs(axis_x) < step) axis_x = 0.0f;
    }
    axis_x = std::max(std::min(axis_x, max_val), min_val);

    if (up_pressed && !down_pressed) {
      axis_y += step;
    } else if (down_pressed && !up_pressed) {
      axis_y -= step;
    } else if (!up_pressed && !down_pressed) {
      if (axis_y > 0) axis_y -= step;
      else if (axis_y < 0) axis_y += step;
      if (std::abs(axis_y) < step) axis_y = 0.0f;
    }
    axis_y = std::max(std::min(axis_y, max_val), min_val);

    // RIGHTX/RIGHTY (IJKL control)
    if (j_pressed && !l_pressed) {
      right_x -= step;
    } else if (l_pressed && !j_pressed) {
      right_x += step;
    } else if (!j_pressed && !l_pressed) {
      // Gradually return to zero
      if (right_x > 0) right_x -= step;
      else if (right_x < 0) right_x += step;
      if (std::abs(right_x) < step) right_x = 0.0f;
    }
    right_x = std::max(std::min(right_x, max_val), min_val);

    if (i_pressed && !k_pressed) {
      right_y += step;
    } else if (k_pressed && !i_pressed) {
      right_y -= step;
    } else if (!i_pressed && !k_pressed) {
      if (right_y > 0) right_y -= step;
      else if (right_y < 0) right_y += step;
      if (std::abs(right_y) < step) right_y = 0.0f;
    }
    right_y = std::max(std::min(right_y, max_val), min_val);

    // TRIGGERLEFT (Z key)
    if (z_pressed) {
      trigger_left = max_val;
    } else {
      trigger_left = 0.0f;
    }
    // TRIGGERRIGHT (X key)
    if (x_pressed) {
      trigger_right = max_val;
    } else {
      trigger_right = 0.0f;
    }

    // Publish Joy message
    auto joy_msg = sensor_msgs::msg::Joy();
    joy_msg.header.stamp = this->get_clock()->now();
    joy_msg.axes.resize(6);
    joy_msg.axes[0] = axis_x; // LEFTX
    joy_msg.axes[1] = axis_y; // LEFTY
    joy_msg.axes[2] = right_x; // RIGHTX
    joy_msg.axes[3] = right_y; // RIGHTY
    joy_msg.axes[4] = trigger_left; // TRIGGERLEFT
    joy_msg.axes[5] = trigger_right; // TRIGGERRIGHT
    joy_msg.buttons.resize(21);
    for (int i = 0; i < 21; ++i) joy_msg.buttons[i] = button[i] ? 1 : 0;
    pub_->publish(joy_msg);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  SDL_Surface* window;
};