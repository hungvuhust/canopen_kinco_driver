#ifndef MOTOR_HPP
#define MOTOR_HPP

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_kinco_driver/base.hpp"
#include "canopen_kinco_driver/default_homing_mode.hpp"
#include "canopen_kinco_driver/mode_forward_helper.hpp"
#include "canopen_kinco_driver/profiled_position_mode.hpp"
#include "canopen_kinco_driver/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

namespace ros2_canopen {

typedef ModeForwardHelper<MotorBase::Velocity_Mode, int32_t, 0x60FF, 0, 0>
    ProfiledVelocityMode;

class CANOPEN_KINCO_DRIVER_EXPORT MotorKinco : public MotorBase {
public:
  MotorKinco(
      std::shared_ptr<LelyDriverBridge>     driver,
      ros2_canopen::State402::InternalState switching_state)
      : MotorBase(), switching_state_(switching_state), monitor_mode_(true),
        state_switch_timeout_(5) {
    this->driver= driver;
  }

  virtual bool setTarget(double val);
  virtual bool enterModeAndWait(uint16_t mode);
  virtual bool isModeSupported(uint16_t mode);

  virtual uint16_t getMode();

  bool readState();

  void handleDiag();

  bool handleInit();

  void handleRead();

  void handleWrite();

  bool handleHomming();

  bool handleShutdown();

  bool handleHalt();

  bool handleRecover();

  template <typename T, typename... Args>
  bool registerMode(uint16_t mode, Args &&...args) {
    return mode_allocators_
        .insert(std::make_pair(
            mode,
            [args..., mode, this]() {
              registerMode(mode, ModeSharedPtr(new T(args...)));
            }))
        .second;
  }

  virtual void registerDefaultModes() {
    registerMode<ProfiledVelocityMode>(MotorBase::Velocity_Mode, driver);
    registerMode<ProfiledPositionMode>(MotorBase::Profiled_Position, driver);
    registerMode<DefaultHomingMode>(MotorBase::Homing, driver);
  }

  double get_speed() const {
    return (double)this->driver->universal_get_value<int32_t>(0x606C, 0);
  }

  double get_position() const {
    return (double)this->driver->universal_get_value<int32_t>(0x6064, 0);
  }

  void set_default_mode(uint16_t mode) { default_mode_id_= mode; }

  void registerMode(uint16_t id, const ModeSharedPtr &m);

  ModeSharedPtr allocMode(uint16_t mode);

  bool switchMode(uint16_t mode);

  bool switchState(const State402::InternalState &target);

private:
  std::atomic<uint16_t>                status_word_;
  uint16_t                             control_word_;
  std::mutex                           cw_mutex_;
  std::atomic<bool>                    start_fault_reset_;
  std::atomic<State402::InternalState> target_state_;

  State402 state_handler_;

  std::mutex                                  map_mutex_;
  std::unordered_map<uint16_t, ModeSharedPtr> modes_;

  typedef std::function<void()> AllocFuncType;

  std::unordered_map<uint16_t, AllocFuncType> mode_allocators_;

  ModeSharedPtr                 selected_mode_;
  uint16_t                      mode_id_;
  uint16_t                      default_mode_id_{MotorBase::No_Mode};
  std::condition_variable       mode_cond_;
  std::mutex                    mode_mutex_;
  const State402::InternalState switching_state_;
  const bool                    monitor_mode_{true};
  const std::chrono::seconds    state_switch_timeout_;

  bool log_error_= false;

  std::shared_ptr<LelyDriverBridge> driver;
  const uint16_t                    status_word_entry_index = 0x6041;
  const uint16_t                    control_word_entry_index= 0x6040;
  const uint16_t                    op_mode_display_index   = 0x6061;
  const uint16_t                    op_mode_index           = 0x6060;
};

} // namespace ros2_canopen

#endif
