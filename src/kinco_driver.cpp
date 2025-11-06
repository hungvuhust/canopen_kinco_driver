#include "canopen_kinco_driver/kinco_driver.hpp"

using namespace ros2_canopen;

KincoDriver::KincoDriver(rclcpp::NodeOptions node_options)
    : CanopenDriver(node_options) {
  node_canopen_kinco_driver_=
      std::make_shared<node_interfaces::NodeCanopenKincoDriver>(this);
  node_canopen_driver_=
      std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(
          node_canopen_kinco_driver_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::KincoDriver)