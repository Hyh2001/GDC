#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

// Helper macros for bit testing
#define test_bit(bit, array) (array[bit / (8*sizeof(unsigned long))] & (1UL << (bit % (8*sizeof(unsigned long)))))
#define NLONGS(x) (((x) + 8*sizeof(unsigned long) - 1) / (8*sizeof(unsigned long)))

class KeyboardNode : public rclcpp::Node
{
public:
  KeyboardNode() : Node("keyboard_node")
  {
    pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    timer = this->create_wall_timer(20ms, std::bind(&KeyboardNode::timer_callback, this));

    // Auto-detect keyboard device
    std::string keyboard_path = findKeyboardDevice();
    if (!keyboard_path.empty()) {
      keyboard_fd_ = open(keyboard_path.c_str(), O_RDONLY | O_NONBLOCK);
      if (keyboard_fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open keyboard device %s. Run with sudo or add user to input group.", keyboard_path.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "No keyboard device found!");
    }

    // states
    axis_x_ = 0.0f;
    axis_y_ = 0.0f;
    right_x_ = 0.0f;
    right_y_ = 0.0f;
    trigger_left_ = 0.0f;
    trigger_right_ = 0.0f;
    for (int i = 0; i < 21; ++i) button_[i] = false;
    left_pressed_ = false;
    right_pressed_ = false;
    up_pressed_ = false;
    down_pressed_ = false;
    w_pressed_ = false;
    a_pressed_ = false;
    s_pressed_ = false;
    d_pressed_ = false;
    i_pressed_ = false;
    j_pressed_ = false;
    k_pressed_ = false;
    l_pressed_ = false;
    z_pressed_ = false;
    x_pressed_ = false;
  }

  ~KeyboardNode() {
    if (keyboard_fd_ != -1) {
      close(keyboard_fd_);
    }
  }

private:
  // Function to automatically find keyboard device
  std::string findKeyboardDevice() {
    DIR *dir = opendir("/dev/input");
    if (!dir) {
      RCLCPP_ERROR(this->get_logger(), "Cannot open /dev/input directory");
      return "";
    }

    struct dirent *entry;
    std::vector<std::pair<std::string, int>> candidates; // device_path, capability_score
    
    while ((entry = readdir(dir)) != nullptr) {
      if (strncmp(entry->d_name, "event", 5) == 0) {
        std::string device_path = "/dev/input/" + std::string(entry->d_name);
        
        int fd = open(device_path.c_str(), O_RDONLY);
        if (fd < 0) continue;

        char name[256] = "Unknown";
        ioctl(fd, EVIOCGNAME(sizeof(name)), name);
        
        // Check if this device has keyboard capabilities
        unsigned long evbit[NLONGS(EV_MAX)] = {0};
        unsigned long keybit[NLONGS(KEY_MAX)] = {0};
        
        if (ioctl(fd, EVIOCGBIT(0, EV_MAX), evbit) >= 0 &&
            ioctl(fd, EVIOCGBIT(EV_KEY, KEY_MAX), keybit) >= 0) {
          
          // Check if device supports key events
          if (test_bit(EV_KEY, evbit)) {
            // Check if it has full keyboard layout (not just media keys or mouse buttons)
            bool has_letters = test_bit(KEY_A, keybit) && test_bit(KEY_S, keybit) && 
                              test_bit(KEY_D, keybit) && test_bit(KEY_F, keybit);
            bool has_numbers = test_bit(KEY_1, keybit) && test_bit(KEY_2, keybit);
            bool has_space = test_bit(KEY_SPACE, keybit);
            bool has_arrows = test_bit(KEY_LEFT, keybit) && test_bit(KEY_RIGHT, keybit) &&
                             test_bit(KEY_UP, keybit) && test_bit(KEY_DOWN, keybit);
            
            // Avoid mouse devices and touchpads
            bool has_mouse_buttons = test_bit(BTN_LEFT, keybit) || test_bit(BTN_RIGHT, keybit);
            
            if (has_letters && has_numbers && has_space && has_arrows && !has_mouse_buttons) {
              // Calculate capability score to prefer more complete keyboards
              int score = 0;
              
              // Check for more keyboard keys to determine the "main" keyboard
              if (test_bit(KEY_ESC, keybit)) score += 10;
              if (test_bit(KEY_TAB, keybit)) score += 10;
              if (test_bit(KEY_ENTER, keybit)) score += 10;
              if (test_bit(KEY_LEFTSHIFT, keybit)) score += 10;
              if (test_bit(KEY_LEFTCTRL, keybit)) score += 10;
              if (test_bit(KEY_LEFTALT, keybit)) score += 10;
              
              // Check for function keys (main keyboards have these)
              if (test_bit(KEY_F1, keybit) && test_bit(KEY_F12, keybit)) score += 20;
              
              // Check for numeric keypad (main keyboards often have these)
              if (test_bit(KEY_KP0, keybit) && test_bit(KEY_KP9, keybit)) score += 15;
              
              RCLCPP_INFO(this->get_logger(), "Found keyboard candidate: %s - %s (score: %d)", 
                         device_path.c_str(), name, score);
              candidates.push_back({device_path, score});
            } else {
              RCLCPP_DEBUG(this->get_logger(), "Skipping device: %s - %s (not a full keyboard)", 
                          device_path.c_str(), name);
            }
          }
        }
        close(fd);
      }
    }
    closedir(dir);
    
    // Sort candidates by score (highest first) and return the best one
    if (!candidates.empty()) {
      std::sort(candidates.begin(), candidates.end(), 
                [](const auto& a, const auto& b) { return a.second > b.second; });
      
      RCLCPP_INFO(this->get_logger(), "Selected keyboard device: %s (best score: %d)", 
                  candidates[0].first.c_str(), candidates[0].second);
      return candidates[0].first;
    }
    
    return "";
  }

private:
    int keyboard_fd_ = -1;
    // --- Axis state ---
    float axis_x_; // LEFTX
    float axis_y_; // LEFTY
    float right_x_; // RIGHTX
    float right_y_; // RIGHTY
    float trigger_left_; // TRIGGERLEFT
    float trigger_right_; // TRIGGERRIGHT
    const float step_ = 0.1f;
    const float max_val_ = 1.0f;
    const float min_val_ = -1.0f;

    // --- Button state ---
    bool button_[21];
    // Key press state
    bool left_pressed_, right_pressed_, up_pressed_, down_pressed_;
    bool w_pressed_, a_pressed_, s_pressed_, d_pressed_;
    bool i_pressed_, j_pressed_, k_pressed_, l_pressed_;
    bool z_pressed_, x_pressed_;

  void timer_callback()
  {
    if (keyboard_fd_ == -1) return;

    struct input_event ev;
    while (read(keyboard_fd_, &ev, sizeof(ev)) == sizeof(ev)) {
      if (ev.type == EV_KEY) {
        bool pressed = (ev.value == 1);
        bool released = (ev.value == 0);
        
        switch (ev.code) {
          case KEY_LEFT: if (pressed) left_pressed_ = true; if (released) left_pressed_ = false; break;
          case KEY_RIGHT: if (pressed) right_pressed_ = true; if (released) right_pressed_ = false; break;
          case KEY_UP: if (pressed) up_pressed_ = true; if (released) up_pressed_ = false; break;
          case KEY_DOWN: if (pressed) down_pressed_ = true; if (released) down_pressed_ = false; break;
          case KEY_W: if (pressed) w_pressed_ = true; if (released) w_pressed_ = false; break;
          case KEY_A: if (pressed) a_pressed_ = true; if (released) a_pressed_ = false; break;
          case KEY_S: if (pressed) s_pressed_ = true; if (released) s_pressed_ = false; break;
          case KEY_D: if (pressed) d_pressed_ = true; if (released) d_pressed_ = false; break;
          case KEY_I: if (pressed) i_pressed_ = true; if (released) i_pressed_ = false; break;
          case KEY_J: if (pressed) j_pressed_ = true; if (released) j_pressed_ = false; break;
          case KEY_K: if (pressed) k_pressed_ = true; if (released) k_pressed_ = false; break;
          case KEY_L: if (pressed) l_pressed_ = true; if (released) l_pressed_ = false; break;
          case KEY_Q: if (pressed) z_pressed_ = true; if (released) z_pressed_ = false; break;  // Left Trigger
          case KEY_E: if (pressed) x_pressed_ = true; if (released) x_pressed_ = false; break;  // Right Trigger
          case KEY_SPACE: if (pressed) button_[0] = true; if (released) button_[0] = false; break;  // A Button
          case KEY_X: if (pressed) button_[1] = true; if (released) button_[1] = false; break;     // B Button  
          case KEY_Z: if (pressed) button_[2] = true; if (released) button_[2] = false; break;     // X Button
          case KEY_C: if (pressed) button_[3] = true; if (released) button_[3] = false; break;     // Y Button
          case KEY_TAB: if (pressed) button_[4] = true; if (released) button_[4] = false; break;   // Left Bumper
          case KEY_R: if (pressed) button_[5] = true; if (released) button_[5] = false; break;     // Right Bumper
          case KEY_LEFTSHIFT: if (pressed) button_[6] = true; if (released) button_[6] = false; break; // Back/Select
          case KEY_ENTER: if (pressed) button_[7] = true; if (released) button_[7] = false; break; // Start
          case KEY_F: if (pressed) button_[8] = true; if (released) button_[8] = false; break;     // Left Stick Click
          case KEY_V: if (pressed) button_[9] = true; if (released) button_[9] = false; break;     // Right Stick Click
          case KEY_T: if (pressed) button_[10] = true; if (released) button_[10] = false; break;   // Extra Button
          case KEY_G: if (pressed) button_[11] = true; if (released) button_[11] = false; break;   // Extra Button
          case KEY_H: if (pressed) button_[12] = true; if (released) button_[12] = false; break;   // Extra Button
          case KEY_M: if (pressed) button_[13] = true; if (released) button_[13] = false; break;   // Extra Button
          case KEY_1: if (pressed) button_[14] = true; if (released) button_[14] = false; break;   // D-Pad Up
          case KEY_2: if (pressed) button_[15] = true; if (released) button_[15] = false; break;   // D-Pad Down
          case KEY_3: if (pressed) button_[16] = true; if (released) button_[16] = false; break;   // D-Pad Left
          case KEY_4: if (pressed) button_[17] = true; if (released) button_[17] = false; break;   // D-Pad Right
          case KEY_LEFTCTRL: if (pressed) button_[18] = true; if (released) button_[18] = false; break; // Extra
          case KEY_LEFTALT: if (pressed) button_[19] = true; if (released) button_[19] = false; break;  // Extra
          case KEY_5: if (pressed) button_[20] = true; if (released) button_[20] = false; break;   // Extra
        }
      }
    }

    // LEFTX/LEFTY (arrow keys or WASD)
    if ((left_pressed_ || a_pressed_) && !(right_pressed_ || d_pressed_)) {
      axis_x_ -= step_;
    } else if ((right_pressed_ || d_pressed_) && !(left_pressed_ || a_pressed_)) {
      axis_x_ += step_;
    } else if (!(left_pressed_ || a_pressed_) && !(right_pressed_ || d_pressed_)) {
      // Gradually return to zero
      if (axis_x_ > 0) axis_x_ -= step_;
      else if (axis_x_ < 0) axis_x_ += step_;
      if (std::abs(axis_x_) < step_) axis_x_ = 0.0f;
    }
    axis_x_ = std::max(std::min(axis_x_, max_val_), min_val_);

    if ((up_pressed_ || w_pressed_) && !(down_pressed_ || s_pressed_)) {
      axis_y_ += step_;
    } else if ((down_pressed_ || s_pressed_) && !(up_pressed_ || w_pressed_)) {
      axis_y_ -= step_;
    } else if (!(up_pressed_ || w_pressed_) && !(down_pressed_ || s_pressed_)) {
      if (axis_y_ > 0) axis_y_ -= step_;
      else if (axis_y_ < 0) axis_y_ += step_;
      if (std::abs(axis_y_) < step_) axis_y_ = 0.0f;
    }
    axis_y_ = std::max(std::min(axis_y_, max_val_), min_val_);

    // RIGHTX/RIGHTY (IJKL control)
    if (j_pressed_ && !l_pressed_) {
      right_x_ -= step_;
    } else if (l_pressed_ && !j_pressed_) {
      right_x_ += step_;
    } else if (!j_pressed_ && !l_pressed_) {
      // Gradually return to zero
      if (right_x_ > 0) right_x_ -= step_;
      else if (right_x_ < 0) right_x_ += step_;
      if (std::abs(right_x_) < step_) right_x_ = 0.0f;
    }
    right_x_ = std::max(std::min(right_x_, max_val_), min_val_);

    if (i_pressed_ && !k_pressed_) {
      right_y_ += step_;
    } else if (k_pressed_ && !i_pressed_) {
      right_y_ -= step_;
    } else if (!i_pressed_ && !k_pressed_) {
      if (right_y_ > 0) right_y_ -= step_;
      else if (right_y_ < 0) right_y_ += step_;
      if (std::abs(right_y_) < step_) right_y_ = 0.0f;
    }
    right_y_ = std::max(std::min(right_y_, max_val_), min_val_);

    // TRIGGERLEFT (Z key)
    if (z_pressed_) {
      trigger_left_ = max_val_;
    } else {
      trigger_left_ = 0.0f;
    }
    // TRIGGERRIGHT (X key)
    if (x_pressed_) {
      trigger_right_ = max_val_;
    } else {
      trigger_right_ = 0.0f;
    }

    // Publish Joy message
    auto joy_msg = sensor_msgs::msg::Joy();
    joy_msg.header.stamp = this->get_clock()->now();
    joy_msg.axes.resize(6);
    joy_msg.axes[0] = axis_x_; // LEFTX
    joy_msg.axes[1] = axis_y_; // LEFTY
    joy_msg.axes[2] = right_x_; // RIGHTX
    joy_msg.axes[3] = right_y_; // RIGHTY
    joy_msg.axes[4] = trigger_left_; // TRIGGERLEFT
    joy_msg.axes[5] = trigger_right_; // TRIGGERRIGHT
    joy_msg.buttons.resize(21);
    for (int i = 0; i < 21; ++i) joy_msg.buttons[i] = button_[i] ? 1 : 0;
    pub_->publish(joy_msg);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
};