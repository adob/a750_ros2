#pragma once

#include <string>
#include <vector>

#include "a750_control/a750.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace a750_hardware
{

class A750System : public hardware_interface::SystemInterface
{
public:
//   RCLCPP_SHARED_PTR_DEFINITIONS(A750System)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool read_joints();
  void set_state(a750pb::RobotState const &state);

  // Example “deployment” params
  // std::string robot_ip_;
  // int robot_port_{0};
  std::string device_path_;

  // Interfaces: one per joint
  std::vector<double> pos_, vel_, eff_;
  std::vector<double> cmd_pos_, cmd_vel_, cmd_eff_;

  a750_control::Robot robot_;
};

}  // n