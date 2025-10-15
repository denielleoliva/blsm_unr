#include "xl330_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <dynamixel_sdk/dynamixel_sdk.h>

namespace xl330_ros2_control {

hardware_interface::CallbackReturn XL330HardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  size_t num_joints = info_.joints.size();
  joint_positions_.resize(num_joints, 0.0);
  joint_velocities_.resize(num_joints, 0.0);
  joint_commands_.resize(num_joints, 0.0);

  port_ = std::make_unique<dynamixel::PortHandler>(info.hardware_parameters.at("port_name"));
  packet_ = std::make_unique<dynamixel::PacketHandler>(2.0);

  if (!port_->openPort() || !port_->setBaudRate(std::stoi(info.hardware_parameters.at("baud_rate")))) {
    RCLCPP_ERROR(rclcpp::get_logger("XL330HardwareInterface"), "Failed to open port or set baud rate");
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < num_joints; ++i) {
    int id = std::stoi(info_.joints[i].name);
    uint8_t dxl_error = 0;
    int dxl_comm_result = packet_->write1ByteTxRx(port_.get(), id, 64, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("XL330HardwareInterface"), "Failed to enable torque for motor ID %d", id);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> XL330HardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position", &joint_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity", &joint_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XL330HardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_commands_.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position", &joint_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type XL330HardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    uint32_t pos = 0;
    uint8_t dxl_error = 0;
    packet_->read4ByteTxRx(port_.get(), std::stoi(info_.joints[i].name), 132, &pos, &dxl_error);
    joint_positions_[i] = static_cast<double>(pos) * (2.0 * M_PI / 4096.0);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XL330HardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
  for (size_t i = 0; i < joint_commands_.size(); ++i) {
    uint32_t pos = static_cast<uint32_t>(joint_commands_[i] * 4096.0 / (2.0 * M_PI));
    uint8_t dxl_error = 0;
    packet_->write4ByteTxRx(port_.get(), std::stoi(info_.joints[i].name), 116, pos, &dxl_error);
  }
  return hardware_interface::return_type::OK;
}

} // namespace xl330_ros2_control