/**
 * @author: Dharan Kumar Nallagatla 
*/

#ifndef RPI_DIFFERENTIAL_DRIVER_H
#define RPI_DIFFERENTIAL_DRIVER_H


#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <pigpiod_if2.h> 

class RpiDriveController : public rclcpp::Node
{
    private:
        
        /* Constants */ 
        const unsigned int m_leftEn = 13;  // GPIO pin number
        const unsigned int m_rightEn = 12; // GPIO pin number
        const unsigned int m_leftBackward = 6; // GPIO pin number
        const unsigned int m_leftForward = 16; // GPIO pin number
        const unsigned int m_rightForward = 16; // GPIO pin number
        const unsigned int m_rightBackward = 20; // GPIO pin number

        const unsigned int m_motorRpm = 96;              // max rpm of motor on full voltage 
        const double m_wheelDiameter = 0.1;      // in meters
        const double m_wheelSeparation = 0.3;     // in meters
        const int m_maxPwmval = 100;           // 100 for Raspberry Pi, 255 for Arduino
        const int m_minPwmVal = 0;             // Minimum PWM value that is needed for the robot to move

        const double m_wheelRadius = m_wheelDiameter / 2;
        const double m_circumference_of_wheel = 2 * M_PI * m_wheelRadius;
        const double m_maxSpeed = (m_circumference_of_wheel * m_motorRpm) / 60;   // m/sec

        /* PWM frequency is 100 Hz */ 
        const unsigned int pwmFrequency = 100;

        int m_pwmL = 0, m_pwmR = 0;

        int m_isConnectionOk = 0; // pigpiod connection check
   
    public:
        
        /**
         * @brief: RpiDriveController Constructor
        */
        RpiDriveController(std::string node_name): Node(node_name)
        {
            RCLCPP_INFO(this->get_logger(), "RpiDriveController Constructor Is Invoked");

            initParameters();
        }

        /**
         * @brief: RpiDriveController Destructor(Returns Non-Zero Value if Connection is Success)
        */
        ~RpiDriveController()
        {
            RCLCPP_INFO(this->get_logger(), "RpiDriveController Destructor Is Invoked");
            pigpio_stop(this->m_isConnectionOk);
        }

        /**
         * @brief: Initializing The Parameters
        */
        void initParameters()
        {
            // Printing initialization parameters
            RCLCPP_INFO(get_logger(), "AMR RpiDriveController Initialized with following Params-");
            RCLCPP_INFO(get_logger(), "Motor Max RPM:\t%d RPM", this->m_motorRpm);
            RCLCPP_INFO(get_logger(), "Wheel Diameter:\t%f m", this->m_wheelDiameter);
            RCLCPP_INFO(get_logger(), "Wheel Separation:\t%f m", this->m_wheelSeparation);
            RCLCPP_INFO(get_logger(), "Robot Max Speed:\t%f m/sec", this->m_maxSpeed);
        }

        /**
         * @brief: Stop The AMR
        */
        void stop()
        {
            if (this->m_isConnectionOk >= 0) 
            {
                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_leftEn, 0);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_leftForward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_leftBackward, PI_HIGH);

                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_rightEn, 0);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_rightForward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_rightBackward, PI_HIGH);
            }
        }

        /**
         * @brief: Move Forward 
        */
        void forward(double left_speed, double right_speed)
        {
            if (this->m_isConnectionOk >= 0) 
            {
                RCLCPP_INFO(get_logger(),"Left speed value = %.2f", left_speed);
                RCLCPP_INFO(get_logger()," Right speed value = %.2f", right_speed);
                RCLCPP_INFO(get_logger()," Max speed value = %.2f", m_maxSpeed);
                RCLCPP_INFO(get_logger()," Max PWM speed value = %d", m_maxPwmval);
                RCLCPP_INFO(get_logger()," MIn PWM speed value = %d", m_minPwmVal);

                double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                
                RCLCPP_INFO(get_logger(),"LPWM value = %.2f", lspeedPWM);
                RCLCPP_INFO(get_logger(),"RightPWM value = %.2f", rspeedPWM);
                
                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_leftEn, lspeedPWM);
                set_PWM_dutycycle(m_isConnectionOk,this->m_rightEn, rspeedPWM);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_leftForward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_rightForward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_leftBackward, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_rightBackward, PI_LOW);
            }   
        }

        /**
         * @brief: Move Backward 
        */
        void backward(double left_speed, double right_speed)
        {
            if (m_isConnectionOk >= 0) 
            {            
                double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
          
                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_leftEn, lspeedPWM);
                set_PWM_dutycycle(m_isConnectionOk,this->m_rightEn, rspeedPWM);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_leftForward, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_rightForward, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_leftBackward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_rightBackward, PI_HIGH);
            }
        }

        /**
         * @brief: Move Left 
        */
        void left(double left_speed, double right_speed)
        {
            if (m_isConnectionOk >= 0) 
            { 
                double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
          
                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_leftEn, lspeedPWM);
                set_PWM_dutycycle(m_isConnectionOk,this->m_rightEn, rspeedPWM);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_leftForward, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_rightForward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_leftBackward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_rightBackward, PI_LOW);
            }
        }

        /**
         * @brief: Move Right 
        */
        void right(double left_speed, double right_speed)
        {
            if (m_isConnectionOk >= 0) 
            {
                double lspeedPWM = std::max(std::min(((left_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
                double rspeedPWM = std::max(std::min(((right_speed / m_maxSpeed) * m_maxPwmval), static_cast<double>(m_maxPwmval)), static_cast<double>(m_minPwmVal));
          
                // Set PWM duty cycle for the GPIO pin
                set_PWM_dutycycle(m_isConnectionOk,this->m_leftEn, lspeedPWM);
                set_PWM_dutycycle(m_isConnectionOk,this->m_rightEn, rspeedPWM);

                /* Writing The Requested Pin Data */
                gpio_write(this->m_isConnectionOk, this->m_leftForward, PI_HIGH);
                gpio_write(this->m_isConnectionOk, this->m_rightForward, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_leftBackward, PI_LOW);
                gpio_write(this->m_isConnectionOk, this->m_rightBackward, PI_HIGH);
            }
        }

        bool startRpi()
        {
            RCLCPP_INFO(get_logger()," StartRpi Function Is Invoked");
            this->m_isConnectionOk = pigpio_start(NULL, NULL); // Checking The Pigpiod Connection (Returns Non-Zero Value if Connection is Success)

            if (this->m_isConnectionOk < 0)
            {
                RCLCPP_ERROR(get_logger()," Pigpio Initialization Failed");
                return false;
            }
            else
            {
                RCLCPP_WARN(get_logger(),"Pigpio Initialization Successful");

                RCLCPP_INFO(this->get_logger(), "Setting All The Pin Mode To Output");
                   
                set_mode(this->m_isConnectionOk, this->m_leftEn, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_rightEn, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_leftBackward, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_leftForward, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_rightForward, PI_OUTPUT);
                set_mode(this->m_isConnectionOk, this->m_rightBackward, PI_OUTPUT);

                /* Set GPIO pin as PWM output and start PWM with specified frequency */
                this->m_pwmL = set_PWM_frequency(this->m_isConnectionOk, this->m_leftEn, this->pwmFrequency);
                set_PWM_dutycycle(m_isConnectionOk,this->m_leftEn, 0);

                /* Set GPIO pin as PWM output and start PWM with specified frequency */
                this->m_pwmR = set_PWM_frequency(this->m_isConnectionOk, this->m_rightEn, this->pwmFrequency);
                set_PWM_dutycycle(m_isConnectionOk,this->m_rightEn, 0);

                return true;
            }
        }

        bool stopRpi()
        {
            RCLCPP_INFO(get_logger()," StopRpi Function Is Invoked");
            pigpio_stop(this->m_isConnectionOk);
            return true;
        }

        int readEncoders(std::string wheel_name)
        {
            RCLCPP_INFO(get_logger()," Read Encoder Function For %s wheel Is Invoked ", wheel_name.c_str());

            int level = 0;

            if(wheel_name == "left" || "LEFT")
            {
                /* Read the Value of the GPIO pin */
                level = gpio_read(this->m_isConnectionOk, this->m_leftEn);
                return level;
            }
            if(wheel_name == "right" || "RIGHT")
            {
                /* Read the Value of the GPIO pin */
                level = gpio_read(this->m_isConnectionOk, this->m_rightEn);
                return level;
            } 
            return 0;
        }
};

#endif