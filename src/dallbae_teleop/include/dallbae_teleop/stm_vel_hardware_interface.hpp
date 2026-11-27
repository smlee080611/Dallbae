#ifndef DALLBAE_TELEOP__STM_VEL_HARDWARE_INTERFACE_HPP_
#define DALLBAE_TELEOP__STM_VEL_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace dallbae_teleop
{

class StmVelHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StmVelHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

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
  bool open_serial(const std::string & port, int baudrate);
  void close_serial();
  void send_line(const char * line);
  void send_allow_cmd(bool allow);
  void send_vel_rpm_cmd(int left_rpm, int right_rpm);
  void send_stop_cmd();
  uint32_t next_seq();

  static int clamp_int(int v, int lo, int hi);

  std::string port_{"/dev/ttyACM0"};
  int baudrate_{115200};
  double rpm_scale_{1.0};
  int max_rpm_{12};  // 초기 시험용 상한 (±12 rpm)

  std::string left_joint_name_{"left_wheel_joint"};
  std::string right_joint_name_{"right_wheel_joint"};

  // [0] = left (모터 1), [1] = right (모터 2)
  std::vector<double> hw_positions_{0.0, 0.0};
  std::vector<double> hw_velocities_{0.0, 0.0};
  std::vector<double> hw_commands_{0.0, 0.0};

  int serial_fd_{-1};
  uint32_t seq_{1};
};

}  // namespace dallbae_teleop

#endif  // DALLBAE_TELEOP__STM_VEL_HARDWARE_INTERFACE_HPP_