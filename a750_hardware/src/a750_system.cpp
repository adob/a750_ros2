#include "a750_hardware/a750_system.hpp"
#include "a750_control/robot.pb_msg.h"
#include "lib/error.h"
#include "lib/fmt/fmt.h"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
//#include "a750_control/a750.h"

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("a750_hardware.A750System");

const std::array<float, 7> DefaultPositionGain = {
  //20.0f, 20.0f, 20.0f,
  240.f, 240.f, 240.f,
  24.0f,  31.0f,  25.0f, 25.0f};
const std::array<float, 7> DefaultVelGain = {3, 3, 3,
                                          0.2f, 0.2f, 0.2f, 0.2f};


struct ErrorLogger : lib::ErrorReporter {
  void handle(lib::Error &error) override {
    lib::String msg = lib::fmt::stringify(error);

    RCLCPP_ERROR(LOGGER, "error: %s", msg.c_str());
  }
} ;

namespace a750_hardware
{

hardware_interface::CallbackReturn A750System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  A750System &a = *this;
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read params from the <ros2_control><hardware><param .../>...</hardware> section
  a.device_path_ = a.info_.hardware_parameters.find("device_path")->second;
  if (a.device_path_.empty()) {
    RCLCPP_ERROR(LOGGER, "Parameter 'device_path' is not set");
    return CallbackReturn::ERROR;
  }
  
  const size_t n = info_.joints.size();
  pos_.assign(n, 0.0);
  vel_.assign(n, 0.0);
  eff_.assign(n, 0.0);
  cmd_pos_.assign(n, 0.0);
  cmd_vel_.assign(n, 0.0);
  cmd_eff_.assign(n, 0.0);

  RCLCPP_INFO(LOGGER, "A750: using device_path=%s; joints size %ld", a.device_path_.c_str(), n);

  if (n != 7) {
    RCLCPP_ERROR(LOGGER, "A750: unexepcted number of joints");
    return CallbackReturn::ERROR;
  }
  
  ErrorLogger err;
  a.robot_.connect(a.device_path_, err);
  if (err) {
    RCLCPP_INFO(LOGGER, "A750: connect error");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "A750: connected");
  return CallbackReturn::SUCCESS;
}

// hardware_interface::CallbackReturn A750System::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
//   // A750System &s = *this;
//   // s.joint_state_interfaces_;

//   RCLCPP_INFO(LOGGER, "A750System::on_configure");
//   set_state("foo", 1);

//   return CallbackReturn::SUCCESS;
// }

std::vector<hardware_interface::StateInterface> A750System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  si.reserve(info_.joints.size() * 3);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    si.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &eff_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface> A750System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  ci.reserve(info_.joints.size() * 3);

  // Pick ONE command mode to start (common: position)
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
    ci.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &cmd_eff_[i]);
  }
  return ci;
}

const float GripperMPerRad = float(0.15268  / (M_PI / 180.0) / 1000.0);

static float gripper_pos_rad_to_m(float pos_rad) {
    return 0.06f + pos_rad * GripperMPerRad;
}

static float gripper_vel_rads_to_ms(float pos_rad) {
    return pos_rad * GripperMPerRad;
}

static float gripper_pos_m_to_rad(float pos_m) {
    return (pos_m - 0.06f) / GripperMPerRad;
}

static float gripper_vel_ms_to_rads(float vel_ms) {
    return vel_ms / GripperMPerRad;
}

void A750System::set_state(a750pb::RobotState const &state) {
  A750System &s = *this;
  
  s.pos_[0] = state.joint1.pos_rad;
  s.pos_[1] = state.joint2.pos_rad;
  s.pos_[2] = state.joint3.pos_rad;
  s.pos_[3] = state.joint4.pos_rad;
  s.pos_[4] = state.joint5.pos_rad;
  s.pos_[5] = state.joint6.pos_rad;
  s.pos_[6] = gripper_pos_rad_to_m(state.gripper.pos_rad);

  s.vel_[0] = state.joint1.vel_rads;
  s.vel_[1] = state.joint2.vel_rads;
  s.vel_[2] = state.joint3.vel_rads;
  s.vel_[3] = state.joint4.vel_rads;
  s.vel_[4] = state.joint5.vel_rads;
  s.vel_[5] = state.joint6.vel_rads;
  s.vel_[6] = gripper_vel_rads_to_ms(state.gripper.vel_rads);

  s.eff_[0] = state.joint1.torque_nm;
  s.eff_[1] = state.joint2.torque_nm;
  s.eff_[2] = state.joint3.torque_nm;
  s.eff_[3] = state.joint4.torque_nm;
  s.eff_[4] = state.joint5.torque_nm;
  s.eff_[5] = state.joint6.torque_nm;
}

bool A750System::read_joints() {
  A750System &s = *this;
  //RCLCPP_INFO(LOGGER, "A750System::read_joints()");

  ErrorLogger err;
  a750pb::RobotState resp = s.robot_.client.robot_service.read_state(err);
  if (err) {
    return false;
  }

  s.set_state(resp);
  return true;
}

hardware_interface::CallbackReturn A750System::on_activate(const rclcpp_lifecycle::State &)
{
  A750System &s = *this;
  RCLCPP_INFO(LOGGER, "activating hardware interface...");

  ErrorLogger err;
  a750pb::RobotState state = s.robot_.client.robot_service.start_realtime_control(err);
  if (err) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  s.set_state(state);
  
  cmd_pos_ = pos_;
  RCLCPP_INFO(LOGGER, "activating hardware interface... done");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn A750System::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  A750System &s = *this;
  RCLCPP_INFO(LOGGER, "deactivating hardware interface");
  ErrorLogger err;
  
  s.robot_.client.robot_service.stop_realtime_control(err);
  if (err) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type A750System::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  A750System &s = *this;

  // if (!s.read_joints()) {
  //   return hardware_interface::return_type::ERROR;
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type A750System::write(
  const rclcpp::Time &, const rclcpp::Duration &/*duration*/)
{
  // RCLCPP_INFO(LOGGER, "A750System::write()");
  // for (int i = 0; i < (int) info_.joints.size(); i++) {
  //   RCLCPP_INFO(LOGGER, "joint %d pos %f", i+1, cmd_pos_[i]);
  // }

  // RCLCPP_INFO(LOGGER, "WRITE pos %f %f %f %f %f %f; vel %f %f %f %f %f %f; eff %f %f %f %f %f %f; dur %f", 
  //     cmd_pos_[0]*(180.0/M_PI), cmd_pos_[1]*(180.0/M_PI), cmd_pos_[2]*(180.0/M_PI), cmd_pos_[3]*(180.0/M_PI), cmd_pos_[4]*(180.0/M_PI), cmd_pos_[5]*(180.0/M_PI),
  //     cmd_vel_[0]*(180.0/M_PI), cmd_vel_[1]*(180.0/M_PI), cmd_vel_[2]*(180.0/M_PI), cmd_vel_[3]*(180.0/M_PI), cmd_vel_[4]*(180.0/M_PI), cmd_vel_[5]*(180.0/M_PI),
  //     cmd_eff_[0], cmd_eff_[1], cmd_eff_[2], cmd_eff_[3], cmd_eff_[4], cmd_eff_[5],
  //     duration.seconds() * 1000);

  // RCLCPP_INFO(LOGGER, "READ  pos %f %f %f %f %f %f; vel %f %f %f %f %f %f; eff %f %f %f %f %f %f", 
  //     pos_[0]*(180.0/M_PI), pos_[1]*(180.0/M_PI), pos_[2]*(180.0/M_PI), pos_[3]*(180.0/M_PI), pos_[4]*(180.0/M_PI), pos_[5]*(180.0/M_PI),
  //     vel_[0]*(180.0/M_PI), vel_[1]*(180.0/M_PI), vel_[2]*(180.0/M_PI), vel_[3]*(180.0/M_PI), vel_[4]*(180.0/M_PI), vel_[5]*(180.0/M_PI),
  //     eff_[0], eff_[1], eff_[2], eff_[3], eff_[4], eff_[5]);

  A750System &s = *this;
  a750pb::CommandJointsRequest req;
  req.joint1.pos_setpoint_rad = float(cmd_pos_[0]);
  req.joint2.pos_setpoint_rad = float(cmd_pos_[1]);
  req.joint3.pos_setpoint_rad = float(cmd_pos_[2]);
  req.joint4.pos_setpoint_rad = float(cmd_pos_[3]);
  req.joint5.pos_setpoint_rad = float(cmd_pos_[4]);
  req.joint6.pos_setpoint_rad = float(cmd_pos_[5]);
  req.gripper.pos_setpoint_rad = gripper_pos_m_to_rad(float(cmd_pos_[6]));

  req.joint1.pos_gain_nmrad = DefaultPositionGain[0];
  req.joint2.pos_gain_nmrad = DefaultPositionGain[1];
  req.joint3.pos_gain_nmrad = DefaultPositionGain[2];
  req.joint4.pos_gain_nmrad = DefaultPositionGain[3];
  req.joint5.pos_gain_nmrad = DefaultPositionGain[4];
  req.joint6.pos_gain_nmrad = DefaultPositionGain[5];
  req.gripper.pos_gain_nmrad = DefaultPositionGain[6];

  req.joint1.vel_setpoint_rads = float(cmd_vel_[0]);
  req.joint2.vel_setpoint_rads = float(cmd_vel_[1]);
  req.joint3.vel_setpoint_rads = float(cmd_vel_[2]);
  req.joint4.vel_setpoint_rads = float(cmd_vel_[3]);
  req.joint5.vel_setpoint_rads = float(cmd_vel_[4]);
  req.joint6.vel_setpoint_rads = float(cmd_vel_[5]);
  req.gripper.vel_setpoint_rads = float(gripper_vel_ms_to_rads(float(cmd_vel_[6])));

  req.joint1.vel_gain_nms_rad = DefaultVelGain[0];
  req.joint2.vel_gain_nms_rad = DefaultVelGain[1];
  req.joint3.vel_gain_nms_rad = DefaultVelGain[2];
  req.joint4.vel_gain_nms_rad = DefaultVelGain[3];
  req.joint5.vel_gain_nms_rad = DefaultVelGain[4];
  req.joint6.vel_gain_nms_rad = DefaultVelGain[5];
  req.gripper.vel_gain_nms_rad = DefaultVelGain[6];

  ErrorLogger err;
  a750pb::RobotState state = s.robot_.client.robot_service.command_joints(req, err);
  if (err) {
    return hardware_interface::return_type::ERROR;
  }

  s.set_state(state);
  return hardware_interface::return_type::OK;
}

}  // namespace a750_hardware

// pluginlib export
PLUGINLIB_EXPORT_CLASS(a750_hardware::A750System, hardware_interface::SystemInterface)