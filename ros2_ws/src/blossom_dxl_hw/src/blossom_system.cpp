#include "blossom_dxl_hw/blossom_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <cmath>
#include <sstream>

namespace blossom_dxl_hw {

// XL-330 helpers (0..4095 -> 0..360 deg)
static inline int   deg_to_pos_xl330(double deg){ if(deg<0)deg=0; if(deg>360)deg=360; return int(std::round(deg*4095.0/360.0)); }
static inline double pos_to_deg_xl330(int pos){ return pos*360.0/4095.0; }

static inline double rad2deg(double r){ return r*180.0/M_PI; }
static inline double deg2rad(double d){ return d*M_PI/180.0; }

struct BlossomSystem::Impl {
  dynamixel::PortHandler* port{nullptr};
  dynamixel::PacketHandler* packet{nullptr};
  bool opened{false};
};

hardware_interface::CallbackReturn BlossomSystem::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // Params
  port_ = info.hardware_parameters.at("port");
  baud_ = std::stoi(info.hardware_parameters.at("baud"));
  {
    // ids as CSV
    auto s = info.hardware_parameters.at("ids");
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) ids_.push_back(std::stoi(tok));
  }

  // joints count must match ids
  if (info_.joints.size() != ids_.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("blossom_hw"), "URDF joints (%zu) != ids (%zu)", info_.joints.size(), ids_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  pos_state_.assign(ids_.size(), 0.0);
  pos_cmd_.assign(ids_.size(), 0.0);

  impl_ = std::make_unique<Impl>();
  impl_->port = dynamixel::PortHandler::getPortHandler(port_.c_str());
  impl_->packet = dynamixel::PacketHandler::getPacketHandler(2.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BlossomSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i=0;i<info_.joints.size();++i)
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[i]);
  return out;
}

std::vector<hardware_interface::CommandInterface> BlossomSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i=0;i<info_.joints.size();++i)
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_cmd_[i]);
  return out;
}

hardware_interface::CallbackReturn BlossomSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (!impl_->port->openPort()) {
    RCLCPP_ERROR(rclcpp::get_logger("blossom_hw"), "openPort(%s) failed", port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!impl_->port->setBaudRate(baud_)) {
    RCLCPP_ERROR(rclcpp::get_logger("blossom_hw"), "setBaudRate(%d) failed", baud_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  impl_->opened = true;

  // Torque ON (XL-330: ADDR_TORQUE_ENABLE=64)
  for (auto id : ids_) {
    uint8_t err=0;
    (void)impl_->packet->write1ByteTxRx(impl_->port, id, 64, 1, &err);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BlossomSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  // Torque OFF
  for (auto id : ids_) {
    uint8_t err=0;
    (void)impl_->packet->write1ByteTxRx(impl_->port, id, 64, 0, &err);
  }
  if (impl_->opened) {
    impl_->port->closePort();
    impl_->opened = false;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BlossomSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
  // XL-330: PRESENT_POSITION=132 (4B)
  for (size_t i=0;i<ids_.size();++i) {
    uint8_t err=0;
    uint32_t raw=0;
    (void)impl_->packet->read4ByteTxRx(impl_->port, ids_[i], 132, &raw, &err);
    pos_state_[i] = deg2rad(pos_to_deg_xl330(int(raw)));
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BlossomSystem::write(const rclcpp::Time &, const rclcpp::Duration &) {
  // XL-330: GOAL_POSITION=116 (4B)
  for (size_t i=0;i<ids_.size();++i) {
    uint32_t ticks = (uint32_t)deg_to_pos_xl330(rad2deg(pos_cmd_[i]));
    uint8_t err=0;
    (void)impl_->packet->write4ByteTxRx(impl_->port, ids_[i], 116, ticks, &err);
  }
  return hardware_interface::return_type::OK;
}

} // namespace blossom_dxl_hw

PLUGINLIB_EXPORT_CLASS(blossom_dxl_hw::BlossomSystem, hardware_interface::SystemInterface)
