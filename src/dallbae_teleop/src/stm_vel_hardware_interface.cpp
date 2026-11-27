#include "dallbae_teleop/stm_vel_hardware_interface.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dallbae_teleop
{

int StmVelHardwareInterface::clamp_int(int v, int lo, int hi)
{
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

uint32_t StmVelHardwareInterface::next_seq()
{
  const uint32_t cur = seq_;
  seq_ = (seq_ >= 1000000000U) ? 1U : (seq_ + 1U);
  return cur;
}

hardware_interface::CallbackReturn StmVelHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() < 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("StmVelHardwareInterface"),
      "Expected 2 wheel joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_joint_name_ = info_.joints[0].name;
  right_joint_name_ = info_.joints[1].name;

  auto get_param_or = [&](const std::string & key, const std::string & def_val) {
    auto it = info_.hardware_parameters.find(key);
    return (it != info_.hardware_parameters.end()) ? it->second : def_val;
  };

  port_ = get_param_or("port", "/dev/ttyACM0");
  baudrate_ = std::stoi(get_param_or("baudrate", "115200"));
  rpm_scale_ = std::stod(get_param_or("rpm_scale", "1.0"));
  max_rpm_ = std::stoi(get_param_or("max_rpm", "12"));

  RCLCPP_INFO(
    rclcpp::get_logger("StmVelHardwareInterface"),
    "Init: port=%s, baud=%d, rpm_scale=%.4f, max_rpm=%d",
    port_.c_str(), baudrate_, rpm_scale_, max_rpm_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
StmVelHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_joint_name_, hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    left_joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_joint_name_, hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    right_joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
StmVelHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    left_joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    right_joint_name_, hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));
  return command_interfaces;
}

hardware_interface::CallbackReturn StmVelHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_positions_ = {0.0, 0.0};
  hw_velocities_ = {0.0, 0.0};
  hw_commands_ = {0.0, 0.0};
  seq_ = 1U;

  if (!open_serial(port_, baudrate_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  send_allow_cmd(true);
  send_vel_rpm_cmd(0, 0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StmVelHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  send_stop_cmd();
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool StmVelHardwareInterface::open_serial(const std::string & port, int baudrate)
{
  serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("StmVelHardwareInterface"),
      "Failed to open %s: %s", port.c_str(), std::strerror(errno));
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    close_serial();
    return false;
  }

  speed_t spd = B115200;
  switch (baudrate) {
    case 9600: spd = B9600; break;
    case 57600: spd = B57600; break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    case 460800: spd = B460800; break;
    case 921600: spd = B921600; break;
    default: spd = B115200; break;
  }

  cfsetospeed(&tty, spd);
  cfsetispeed(&tty, spd);
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    close_serial();
    return false;
  }
  return true;
}

void StmVelHardwareInterface::close_serial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

void StmVelHardwareInterface::send_line(const char * line)
{
  if (serial_fd_ < 0 || line == nullptr) {
    return;
  }

  const size_t total = std::strlen(line);
  size_t sent = 0;
  while (sent < total) {
    const ssize_t n = ::write(serial_fd_, line + sent, total - sent);
    if (n <= 0) {
      break;
    }
    sent += static_cast<size_t>(n);
  }
}

void StmVelHardwareInterface::send_allow_cmd(bool allow)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "A,%u\n", allow ? 1U : 0U);
  send_line(buf);
}

void StmVelHardwareInterface::send_vel_rpm_cmd(int left_rpm, int right_rpm)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "V,%u,%d,%d\n", next_seq(), left_rpm, right_rpm);
  send_line(buf);
}

void StmVelHardwareInterface::send_stop_cmd()
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "S,%u\n", next_seq());
  send_line(buf);
}

hardware_interface::return_type StmVelHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 피드백 미수신 구성이므로 상태값 유지
  return hardware_interface::return_type::SUCCESS;
}

hardware_interface::return_type StmVelHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // rad/s -> rpm: (rad/s) * (60 / (2 * pi)) * rpm_scale
  constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);

  const double raw_left_rpm = hw_commands_[0] * kRadPerSecToRpm * rpm_scale_;
  const double raw_right_rpm = hw_commands_[1] * kRadPerSecToRpm * rpm_scale_;

  const int left_rpm = clamp_int(
    static_cast<int>(std::llround(raw_left_rpm)), -max_rpm_, max_rpm_);
  const int right_rpm = clamp_int(
    static_cast<int>(std::llround(raw_right_rpm)), -max_rpm_, max_rpm_);

  send_vel_rpm_cmd(left_rpm, right_rpm);
  return hardware_interface::return_type::SUCCESS;
}

}  // namespace dallbae_teleop

PLUGINLIB_EXPORT_CLASS(
  dallbae_teleop::StmVelHardwareInterface,
  hardware_interface::SystemInterface)