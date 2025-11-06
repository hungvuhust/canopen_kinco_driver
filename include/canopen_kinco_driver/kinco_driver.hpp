#ifndef CANOPEN_KINCO_DRIVER__KINCO_DRIVER_HPP_
#define CANOPEN_KINCO_DRIVER__KINCO_DRIVER_HPP_
#include "canopen_core/driver_node.hpp"
#include "canopen_kinco_driver/node_interfaces/node_canopen_kinco_driver.hpp"
#include "canopen_kinco_driver/visibility_control.h"

namespace ros2_canopen {
/**
 * @brief Abstract Class for a CANopen Device Node
 *
 * This class provides the base functionality for creating a
 * CANopen device node. It provides callbacks for nmt and rpdo.
 */
class CANOPEN_KINCO_DRIVER_EXPORT KincoDriver
    : public ros2_canopen::CanopenDriver {
  std::shared_ptr<node_interfaces::NodeCanopenKincoDriver>
      node_canopen_kinco_driver_;

public:
  KincoDriver(rclcpp::NodeOptions node_options= rclcpp::NodeOptions());

  virtual bool reset_node_nmt_command() {
    return node_canopen_kinco_driver_->reset_node_nmt_command();
  }

  virtual bool start_node_nmt_command() {
    return node_canopen_kinco_driver_->start_node_nmt_command();
  }

  virtual bool tpdo_transmit(ros2_canopen::COData &data) {
    return node_canopen_kinco_driver_->tpdo_transmit(data);
  }

  virtual void register_nmt_state_cb(
      std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb) {
    node_canopen_kinco_driver_->register_nmt_state_cb(nmt_state_cb);
  }

  virtual void register_rpdo_cb(std::function<void(COData, uint8_t)> rpdo_cb) {
    node_canopen_kinco_driver_->register_rpdo_cb(rpdo_cb);
  }

  virtual double get_speed() { return node_canopen_kinco_driver_->get_speed(); }

  virtual double get_position() {
    return node_canopen_kinco_driver_->get_position();
  }

  virtual bool set_target(double target) {
    return node_canopen_kinco_driver_->set_target(target);
  }

  virtual bool init_motor() { return node_canopen_kinco_driver_->init_motor(); }

  virtual bool set_mode_velocity() {
    return node_canopen_kinco_driver_->set_mode_velocity();
  }

  virtual bool halt_motor() { return node_canopen_kinco_driver_->halt_motor(); }

  virtual bool set_mode_position() {
    return node_canopen_kinco_driver_->set_mode_position();
  }

  uint16_t get_mode() { return node_canopen_kinco_driver_->get_mode(); }

  bool set_operation_mode(uint16_t mode) {
    return node_canopen_kinco_driver_->set_operation_mode(mode);
  }

  void set_default_mode(uint16_t mode) {
    node_canopen_kinco_driver_->set_default_mode(mode);
  }
};
} // namespace ros2_canopen

#endif // CANOPEN_402_DRIVER__CANOPEN_402_DRIVER_HPP_
