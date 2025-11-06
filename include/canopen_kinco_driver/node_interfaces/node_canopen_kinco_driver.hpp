#ifndef NODE_CANOPEN_KINCO_DRIVER
#define NODE_CANOPEN_KINCO_DRIVER

#include "canopen_base_driver/lely_driver_bridge.hpp"
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_kinco_driver/motor.hpp"
#include "canopen_kinco_driver/visibility_control.h"

namespace ros2_canopen {
namespace node_interfaces {

class CANOPEN_KINCO_DRIVER_EXPORT NodeCanopenKincoDriver
    : public NodeCanopenBaseDriver {

protected:
  std::shared_ptr<MotorKinco>                        motor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      handle_set_mode_velocity_service,
      handle_set_mode_position_service, handle_homming_mode_service;
  double                                scale_pos_to_dev_;
  double                                scale_pos_from_dev_;
  double                                scale_vel_to_dev_;
  double                                scale_vel_from_dev_;
  ros2_canopen::State402::InternalState switching_state_;

  virtual void poll_timer_callback() override;

public:
  NodeCanopenKincoDriver(rclcpp::Node *node);
  // ================== Init ==================
  virtual void init(bool called_from_base) override;
  virtual void configure(bool called_from_base) override;
  virtual void activate(bool called_from_base) override;
  virtual void deactivate(bool called_from_base) override;
  virtual void add_to_master() override;
  virtual bool init_motor();
  bool         halt_motor();

  // ================== Setters ==================
  virtual bool set_operation_mode(uint16_t mode);
  virtual bool set_mode_velocity();
  virtual bool set_mode_position();
  virtual bool set_target(double target);
  virtual void set_default_mode(uint16_t mode) {
    motor_->set_default_mode(mode);
  }
  // ================== Getters ==================
  virtual double get_speed() {
    return motor_->get_speed() * scale_vel_from_dev_;
  }
  virtual double get_position() {
    return motor_->get_position() * scale_pos_from_dev_;
  }
  uint16_t get_mode() { return motor_->getMode(); }
  // ================== Service Handlers ==================
  void handle_init(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr      response);
  void handle_set_mode_velocity(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr      response);
  void handle_set_mode_position(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr      response);
  void handle_homming_mode(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr      response);
};
} // namespace node_interfaces
} // namespace ros2_canopen

#endif
