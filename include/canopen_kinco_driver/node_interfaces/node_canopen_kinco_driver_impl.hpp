#ifndef NODE_CANOPEN_KINCO_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_KINCO_DRIVER_IMPL_HPP_

#include "canopen_core/driver_error.hpp"
#include "canopen_kinco_driver/node_interfaces/node_canopen_kinco_driver.hpp"

#include <iostream>
#include <optional>

using namespace std::placeholders;

namespace ros2_canopen {
namespace node_interfaces {

NodeCanopenKincoDriver::NodeCanopenKincoDriver(rclcpp::Node *node)
    : NodeCanopenBaseDriver(node) {
  RCLCPP_INFO(this->node_->get_logger(), "NodeCanopenKincoDriver");
}

void NodeCanopenKincoDriver::init(bool called_from_base) {
  NodeCanopenBaseDriver::init(false);

  handle_init_service= this->node_->create_service<std_srvs::srv::Trigger>(
      std::string(this->node_->get_name()).append("/init").c_str(),
      std::bind(
          &NodeCanopenKincoDriver::handle_init, this, std::placeholders::_1,
          std::placeholders::_2));

  handle_set_mode_velocity_service=
      this->node_->create_service<std_srvs::srv::Trigger>(
          std::string(this->node_->get_name()).append("/velocity_mode").c_str(),
          std::bind(
              &NodeCanopenKincoDriver::handle_set_mode_velocity, this,
              std::placeholders::_1, std::placeholders::_2));
  handle_set_mode_position_service=
      this->node_->create_service<std_srvs::srv::Trigger>(
          std::string(this->node_->get_name()).append("/position_mode").c_str(),
          std::bind(
              &NodeCanopenKincoDriver::handle_set_mode_position, this,
              std::placeholders::_1, std::placeholders::_2));

  handle_homming_mode_service=
      this->node_->create_service<std_srvs::srv::Trigger>(
          std::string(this->node_->get_name()).append("/homming_mode").c_str(),
          std::bind(
              &NodeCanopenKincoDriver::handle_homming_mode, this,
              std::placeholders::_1, std::placeholders::_2));
}

void NodeCanopenKincoDriver::configure(bool called_from_base) {
  NodeCanopenBaseDriver::configure(false);
  std::optional<double> scale_pos_to_dev;
  std::optional<double> scale_pos_from_dev;
  std::optional<double> scale_vel_to_dev;
  std::optional<double> scale_vel_from_dev;
  std::optional<int>    switching_state;
  try {
    scale_pos_to_dev=
        std::optional(this->config_["scale_pos_to_dev"].as<double>());
  } catch (...) {}
  try {
    scale_pos_from_dev=
        std::optional(this->config_["scale_pos_from_dev"].as<double>());
  } catch (...) {}
  try {
    scale_vel_to_dev=
        std::optional(this->config_["scale_vel_to_dev"].as<double>());
  } catch (...) {}
  try {
    scale_vel_from_dev=
        std::optional(this->config_["scale_vel_from_dev"].as<double>());
  } catch (...) {}
  try {
    switching_state= std::optional(this->config_["switching_state"].as<int>());
  } catch (...) {}

  scale_pos_to_dev_  = scale_pos_to_dev.value_or(1);
  scale_pos_from_dev_= scale_pos_from_dev.value_or(1);
  scale_vel_to_dev_  = scale_vel_to_dev.value_or(1);
  scale_vel_from_dev_= scale_vel_from_dev.value_or(1);
  switching_state_=
      (ros2_canopen::State402::InternalState)switching_state.value_or(
          (int)ros2_canopen::State402::InternalState::Operation_Enable);
  RCLCPP_INFO(
      this->node_->get_logger(),
      "scale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ "
      "%f\nscale_vel_from_dev_ %f\n",
      scale_pos_to_dev_, scale_pos_from_dev_, scale_vel_to_dev_,
      scale_vel_from_dev_);
}

void NodeCanopenKincoDriver::activate(bool called_from_base) {
  NodeCanopenBaseDriver::activate(false);
  motor_->registerDefaultModes();
}

void NodeCanopenKincoDriver::deactivate(bool called_from_base) {
  NodeCanopenBaseDriver::deactivate(false);
}

void NodeCanopenKincoDriver::poll_timer_callback() {
  NodeCanopenBaseDriver::poll_timer_callback();
  motor_->handleRead();
  motor_->handleWrite();
}

void NodeCanopenKincoDriver::add_to_master() {
  NodeCanopenBaseDriver::add_to_master();
  motor_= std::make_shared<MotorKinco>(this->lely_driver_, switching_state_);
}

bool NodeCanopenKincoDriver::init_motor() {
  if (this->activated_.load()) {
    bool temp= motor_->handleInit();
    return temp;
  } else {
    RCLCPP_INFO(this->node_->get_logger(), "Initialisation failed.");
    return false;
  }
}

bool NodeCanopenKincoDriver::halt_motor() {
  if (this->activated_.load()) {
    return motor_->handleHalt();
  } else {
    return false;
  }
}

bool NodeCanopenKincoDriver::set_operation_mode(uint16_t mode) {
  if (this->activated_.load()) {
    return motor_->enterModeAndWait(mode);
  }
  return false;
}

bool NodeCanopenKincoDriver::set_mode_velocity() {
  if (this->activated_.load()) {
    if (motor_->getMode() != MotorBase::Velocity_Mode) {
      return motor_->enterModeAndWait(MotorBase::Velocity_Mode);
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool NodeCanopenKincoDriver::set_mode_position() {
  if (this->activated_.load()) {
    if (motor_->getMode() != MotorBase::Profiled_Position) {
      return motor_->enterModeAndWait(MotorBase::Profiled_Position);
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool NodeCanopenKincoDriver::set_target(double target) {
  if (this->activated_.load()) {
    auto   mode= motor_->getMode();
    double scaled_target;
    if ((mode == MotorBase::Profiled_Position) or
        (mode == MotorBase::Cyclic_Synchronous_Position) or
        (mode == MotorBase::Interpolated_Position)) {
      scaled_target= target * scale_pos_to_dev_;
    } else if (
        (mode == MotorBase::Velocity) or
        (mode == MotorBase::Profiled_Velocity) or
        (mode == MotorBase::Velocity_Mode) or
        (mode == MotorBase::Cyclic_Synchronous_Velocity)) {
      scaled_target= target * scale_vel_to_dev_;
    } else {
      scaled_target= target;
    }
    return motor_->setTarget(scaled_target);
  } else {
    return false;
  }
}

void NodeCanopenKincoDriver::handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr      response) {
  if (this->activated_.load()) {
    bool temp        = motor_->handleInit();
    response->success= temp;
  }
}

void NodeCanopenKincoDriver::handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr      response) {
  response->success= set_mode_velocity();
}

void NodeCanopenKincoDriver::handle_set_mode_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr      response) {
  response->success= set_mode_position();
}

void NodeCanopenKincoDriver::handle_homming_mode(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr      response) {
  if (this->activated_.load()) {
    response->success= motor_->handleHomming();
  }
}

} // namespace node_interfaces
} // namespace ros2_canopen

#endif
