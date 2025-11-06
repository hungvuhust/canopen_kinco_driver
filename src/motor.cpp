#include "canopen_kinco_driver/motor.hpp"
#include "canopen_kinco_driver/base.hpp"
#include "canopen_kinco_driver/state.hpp"
#include <rclcpp/logger.hpp>
using namespace ros2_canopen;

bool MotorKinco::setTarget(double val) {
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ && selected_mode_->setTarget(val);
  }
  return false;
}
bool MotorKinco::isModeSupported(uint16_t mode) {
  return mode != MotorBase::Homing && allocMode(mode);
}

bool MotorKinco::enterModeAndWait(uint16_t mode) {
  bool okay = mode != MotorBase::Homing && switchMode(mode);
  return okay;
}

uint16_t MotorKinco::getMode() {
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_ ? selected_mode_->mode_id_
                        : (uint16_t)MotorBase::No_Mode;
}

void MotorKinco::registerMode(uint16_t id, const ModeSharedPtr &m) {
  std::scoped_lock map_lock(map_mutex_);
  if (m && m->mode_id_ == id)
    modes_.insert(std::make_pair(id, m));
}

ModeSharedPtr MotorKinco::allocMode(uint16_t mode) {
  ModeSharedPtr res;
  {
    std::scoped_lock map_lock(map_mutex_);

    std::unordered_map<uint16_t, ModeSharedPtr>::iterator it =
        modes_.find(mode);
    if (it != modes_.end()) {
      res = it->second;
    }
  }
  return res;
}

bool MotorKinco::switchMode(uint16_t mode) {
  if (mode == MotorBase::No_Mode) {
    std::scoped_lock lock(mode_mutex_);
    selected_mode_.reset();
    try { // try to set mode
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);
    } catch (...) {
    }
    return true;
  }
  // HUNGVU
  RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
              "Switching to mode %d", mode);
  // HUNGVU
  ModeSharedPtr next_mode = allocMode(mode);
  if (!next_mode) {
    RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                "Mode is not supported.");
    return false;
  }

  if (!next_mode->start()) {
    RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                "Could not  start mode.");
    return false;
  }

  { // disable mode handler
    std::scoped_lock lock(mode_mutex_);

    if (mode_id_ == mode && selected_mode_ &&
        selected_mode_->mode_id_ == mode) {
      // nothing to do
      return true;
    }

    selected_mode_.reset();
  }

  if (!switchState(switching_state_)) {
    return false;
  }

  driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);

  bool okay = false;

  { // wait for switch
    std::unique_lock lock(mode_mutex_);

    std::chrono::steady_clock::time_point abstime =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    if (monitor_mode_) {
      while (mode_id_ != mode && mode_cond_.wait_until(lock, abstime) ==
                                     std::cv_status::no_timeout) {
      }
    } else {
      while (mode_id_ != mode && std::chrono::steady_clock::now() < abstime) {
        lock.unlock(); // unlock inside loop
        driver->universal_get_value<int8_t>(op_mode_display_index, 0x0); // poll
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100)); // wait some time
        lock.lock();
      }
    }

    if (mode_id_ == mode) {
      selected_mode_ = next_mode;
      okay           = true;
    } else {
      RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                  "Mode switch timed out. Because mode_id: %d, mode: %d",
                  mode_id_, mode);
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode_id_);
    }
  }

  if (!switchState(State402::Operation_Enable))
    return false;

  return okay;
}

bool MotorKinco::switchState(const State402::InternalState &target) {
  std::chrono::steady_clock::time_point abstime =
      std::chrono::steady_clock::now() + state_switch_timeout_;
  State402::InternalState state = state_handler_.getState();
  target_state_                 = target;
  while (state != target_state_) {
    std::unique_lock        lock(cw_mutex_);
    State402::InternalState next = State402::Unknown;
    bool                    success =
        Command402::setTransition(control_word_, state, target_state_, &next);
    if (!success) {
      RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                  "Could not set transition.");
      return false;
    }
    lock.unlock();
    if (state != next && !state_handler_.waitForNewState(abstime, state)) {
      RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                  "Transition timed out.");

      return false;
    }
  }
  return state == target;
}

bool MotorKinco::readState() {
  uint16_t old_sw,
      sw = driver->universal_get_value<uint16_t>(
          status_word_entry_index, 0x0); // TODO: added error handling
  old_sw = status_word_.exchange(sw);

  state_handler_.read(sw);

  std::unique_lock lock(mode_mutex_);
  uint16_t         new_mode;

  new_mode = driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);

  if (selected_mode_ /*&& selected_mode_->mode_id_ == new_mode*/) {
    if (!selected_mode_->read(sw)) {
      RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                  "Mode handler has error.");
    }
  }

  if (new_mode != mode_id_) {
    mode_id_ = new_mode;
    mode_cond_.notify_all();
  }

  // if (selected_mode_ && selected_mode_->mode_id_ != new_mode &&
  //     state_handler_.getState() != State402::Switch_On_Disabled) {
  //   RCLCPP_INFO(
  //       rclcpp::get_logger("canopen_kinco_driver"),
  //       "Mode does not match: %d, selected mode: %d., state: %d", new_mode,
  //       selected_mode_->mode_id_, state_handler_.getState());
  // }

  // if (!(sw & (1 << State402::SW_Quick_stop))) {
  //   if (old_sw & (1 << State402::SW_Quick_stop)) {
  //     RCLCPP_INFO(
  //         rclcpp::get_logger("canopen_kinco_driver"), "Quick stop active");
  //   } else {
  //     RCLCPP_INFO(
  //         rclcpp::get_logger("canopen_kinco_driver"), "Quick stop inactive");
  //   }
  // }

  if (sw & (1 << State402::SW_Internal_limit)) {
    if (old_sw & (1 << State402::SW_Internal_limit)) {
      RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                  "Internal limit active");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
                  "Internal limit active");
    }
  }

  return true;
}

void MotorKinco::handleRead() { readState(); }
void MotorKinco::handleWrite() {
  std::scoped_lock lock(cw_mutex_);
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable) {
    std::scoped_lock     lock(mode_mutex_);
    Mode::OpModeAccesser cwa(control_word_);
    bool                 okay = false;
    if (selected_mode_ && selected_mode_->mode_id_ == mode_id_) {
      okay = selected_mode_->write(cwa);
    } else {
      cwa = 0;
    }
    if (okay) {
      control_word_ &= ~(1 << Command402::CW_Halt);
    }
  }
  if (start_fault_reset_.exchange(false)) {
    this->driver->universal_set_value<uint16_t>(
        control_word_entry_index, 0,
        control_word_ & ~(1 << Command402::CW_Fault_Reset));
  } else {
    this->driver->universal_set_value<uint16_t>(control_word_entry_index, 0,
                                                control_word_);
  }
}

bool MotorKinco::handleInit() {
  for (std::unordered_map<uint16_t, AllocFuncType>::iterator it =
           mode_allocators_.begin();
       it != mode_allocators_.end(); ++it) {
    (it->second)();
  }

  uint16_t status_word_data_1 =
      driver->universal_get_value<uint16_t>(status_word_entry_index, 0x0);
  if ((status_word_data_1 & 0x0007) == 0x0007) {
    // already enabled
    return true;
  }
  if (!readState()) {
    std::cout << "Could not read motor state" << std::endl;
    return false;
  }

  {
    std::scoped_lock lock(cw_mutex_);
    control_word_      = 0;
    start_fault_reset_ = true;
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"), "Init: Enable");
  if (!switchState(State402::Operation_Enable)) {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Switch no mode");
  if (!switchMode(MotorBase::No_Mode)) {
    std::cout << "Could not enter no mode" << std::endl;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_kinco_driver"),
              "Init: Switch to default mode");
  if (!switchMode(default_mode_id_)) {
    std::cout << "Could not enter default mode" << std::endl;
    return false;
  }

  return true;
}

bool MotorKinco::handleHomming() {
  {
    std::scoped_lock lock(cw_mutex_);
    control_word_      = 0x10f;
    start_fault_reset_ = true;
  }

  ModeSharedPtr m = allocMode(MotorBase::Homing);
  if (!m) {
    std::cout << "Homeing mode not supported" << std::endl;
    return true; // homing not supported
  }

  HomingMode *homing = dynamic_cast<HomingMode *>(m.get());

  if (!homing) {
    std::cout << "Homing mode has incorrect handler" << std::endl;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"),
              "Homming: Switch to homing");

  if (!switchMode(MotorBase::Homing)) {
    std::cout << "Could not enter homing mode" << std::endl;
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"),
              "Homming: Execute homing");
  if (!homing->executeHoming()) {
    std::cout << "Homing failed" << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"),
                "Homming: Switch no mode");
    if (!switchMode(MotorBase::No_Mode)) {
      std::cout << "Could not enter no mode" << std::endl;
      return false;
    }

    if (!switchMode(MotorBase::Profiled_Position)) {
      std::cout << "Could not enter default mode" << std::endl;
      return false;
    }
    return false;
  }

  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"),
              "Homming: Switch no mode");
  if (!switchMode(MotorBase::No_Mode)) {
    std::cout << "Could not enter no mode" << std::endl;
    return false;
  }

  if (!switchMode(MotorBase::Profiled_Position)) {
    std::cout << "Could not enter default mode" << std::endl;
    return false;
  }

  return true;
}

bool MotorKinco::handleShutdown() {
  switchMode(MotorBase::No_Mode);
  return switchState(State402::Switch_On_Disabled);
}

bool MotorKinco::handleHalt() {
  State402::InternalState state = state_handler_.getState();
  std::scoped_lock        lock(cw_mutex_);

  // do not demand quickstop in case of fault
  if (state == State402::Fault_Reaction_Active || state == State402::Fault)
    return false;

  if (state != State402::Operation_Enable) {
    target_state_ = state;
  } else {
    target_state_ = State402::Quick_Stop_Active;
    if (!Command402::setTransition(control_word_, state,
                                   State402::Quick_Stop_Active, 0)) {
      std::cout << "Could not quick stop" << std::endl;
      return false;
    }
  }
  return true;
}

bool MotorKinco::handleRecover() {
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    if (selected_mode_ && !selected_mode_->start()) {
      std::cout << "Could not restart mode." << std::endl;
      return false;
    }
  }
  if (!switchState(State402::Operation_Enable)) {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }
  return true;
}
