#ifndef BLOSSOM_HARDWARE_INTERFACE_HPP_
#define BLOSSOM_HARDWARE_INTERFACE_HPP_


#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "blossom_ros2_driver/visibility_control.hpp"


#include <memory>
#include <vector>


namespace dynamixel {
class PortHandler;
class PacketHandler;
}


namespace blossom_ros2_driver {


class BlossomHardwareInterface : public hardware_interface::SystemInterface {
public:
RCLCPP_SHARED_PTR_DEFINITIONS(BlossomHardwareInterface)


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


} // namespace blossom_ros2_driver


#endif // BLOSSOM_HARDWARE_INTERFACE_HPP_