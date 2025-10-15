#include "blossom_ros2_driver/blossom_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <dynamixel_sdk/dynamixel_sdk.h>


namespace blossom_ros2_driver {


hardware_interface::CallbackReturn BlossomHardwareInterface::on_init(const hardware_interface::HardwareInfo & info) {
if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
return hardware_interface::CallbackReturn::ERROR;
}


joint_positions_.resize(info_.joints.size(), 0.0);
joint_velocities_.resize(info_.joints.size(), 0.0);
joint_commands_.resize(info_.joints.size(), 0.0);


port_ = std::make_unique<dynamixel::PortHandler>(info_.hardware_parameters.at("port_name"));
packet_ = std::make_unique<dynamixel::PacketHandler>(2.0);


if (!port_->openPort() || !port_->setBaudRate(std::stoi(info_.hardware_parameters.at("baud_rate")))) {
RCLCPP_ERROR(rclcpp::get_logger("BlossomHardwareInterface"), "Failed to open port or set baud rate");
return hardware_interface::CallbackReturn::ERROR;
}


for (const auto& joint : info_.joints) {
int id = std::stoi(joint.name); // assumes joint.name is the ID, or map manually
uint8_t dxl_error = 0;
int result = packet_->write1ByteTxRx(port_.get(), id, 64, 1, &dxl_error); // Enable torque
if (result != COMM_SUCCESS || dxl_error != 0) {
RCLCPP_ERROR(rclcpp::get_logger("BlossomHardwareInterface"), "Failed to enable torque for ID %d", id);
return hardware_interface::CallbackReturn::ERROR;
}
}


return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BlossomHardwareInterface::export_state_interfaces() {
std::vector<hardware_interface::StateInterface> states;
for (size_t i = 0; i < info_.joints.size(); ++i) {
states.emplace_back(info_.joints[i].name, "position", &joint_positions_[i]);
states.emplace_back(info_.joints[i].name, "velocity", &joint_velocities_[i]);
}
return states;
}


std::vector<hardware_interface::CommandInterface> BlossomHardwareInterface::export_command_interfaces() {
std::vector<hardware_interface::CommandInterface> cmds;
for (size_t i = 0; i < info_.joints.size(); ++i) {
cmds.emplace_back(info_.joints[i].name, "position", &joint_commands_[i]);
}
return cmds;
}


hardware_interface::return_type BlossomHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) {
for (size_t i = 0; i < info_.joints.size(); ++i) {
uint32_t pos_raw = 0;
uint8_t dxl_error = 0;
packet_->read4ByteTxRx(port_.get(), std::stoi(info_.joints[i].name), 132, &pos_raw, &dxl_error);
joint_positions_[i] = static_cast<double>(pos_raw) * (2.0 * M_PI / 4096.0);
}
return hardware_interface::return_type::OK;
}


hardware_interface::return_type BlossomHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) {
for (size_t i = 0; i < info_.joints.size(); ++i) {
uint32_t pos_raw = static_cast<uint32_t>(joint_commands_[i] * 4096.0 / (2.0 * M_PI));
uint8_t dxl_error = 0;
packet_->write4ByteTxRx(port_.get(), std::stoi(info_.joints[i].name), 116, pos_raw, &dxl_error);
}
return hardware_interface::return_type::OK;
}


} // namespace blossom_ros2_driver