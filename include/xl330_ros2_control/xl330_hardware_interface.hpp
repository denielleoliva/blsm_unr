#ifndef XL330_HARDWARE_INTERFACE_HPP_
#define XL330_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "visibility_control.h"

#include <memory>
#include <vector>

namespace dynamixel {
  class PortHandler;
  class PacketHandler;
}

namespace xl330_ros2_control {

class XL330HardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XL330HardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::unique_ptr<dynamixel::PortHandler> port_;
  std::unique_ptr<dynamixel::PacketHandler> packet_;

  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_commands_;
};

} // namespace xl330_ros2_control

#endif  // XL330_HARDWARE_INTERFACE_HPP_