#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>

namespace blossom_dxl_hw {

class BlossomSystem : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BlossomSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & prev_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & prev_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // params
  std::string port_;
  int baud_{57600};
  std::vector<int> ids_;      // order matches joints in URDF

  // state/command (radians)
  std::vector<double> pos_state_;
  std::vector<double> pos_cmd_;

  // SDK impl (opaque here)
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace blossom_dxl_hw
