/**
 * @author: Dharan Kumar Nallagatla 
*/

#ifndef RPI_ROS2_CONTROL_H
#define RPI_ROS2_CONTROL_H

#include <cstring>
#include "rclcpp/rclcpp.hpp"

#include "rpi_diff_hw_control/rpi_differential_driver.hpp"

//#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <unistd.h>

#include <limits>
#include <vector>

namespace rpi_diff_hw_control
{
    class RpiController : public hardware_interface::SystemInterface
    {
        struct Params
        {
            std::string left_wheel_name = "left_wheel";
            std::string right_wheel_name = "right_wheel";
            float loop_rate = 30;
            std::string device = "/dev/ttyUSB0";
            int baud_rate = 57600;
            int timeout = 1000;
            int enc_counts_per_rev = 3436;
        };

        private:
            
        RpiDriveController m_rpiDriveObj;
        Params m_Params;
            
        public:
            RpiController();

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


            rclcpp::Logger logger_;

            std::chrono::time_point<std::chrono::system_clock> time_;

            std::vector<double> hw_commands_;
            std::vector<double> hw_positions_;
            std::vector<double> hw_velocities_;
    };
}
#endif
