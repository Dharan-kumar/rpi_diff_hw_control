/**
 * @author: Dharan Kumar Nallagatla
*/

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

/**
 * @brief: Node to convert Twist msg to Twist Stamped as the ros2 controller accepting the Twiststamped
*/
class TwistToTwistStampedNode : public rclcpp::Node
{
    public:
        /**
         * @brief: TwistToTwistStampedNode Constructor
        */
        TwistToTwistStampedNode() : Node("twist_to_twiststamped_node")
        {
            RCLCPP_INFO(get_logger(),"TwistToTwistStampedNode Is Online");

            // Subscriber for teleop node
            m_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&TwistToTwistStampedNode::cmdVelCallback, this, std::placeholders::_1));

            // Publisher for TwistStamped on diff_drive_controller/cmd_vel topic
            m_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diffbot_base_controller/cmd_vel", 10);

            // Timer to periodically publish data
            m_timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TwistToTwistStampedNode::publishData, this));
        }

    private:
        /**
         * @brief: Publishing the data for every 100 milliseconds
        */
        void publishData()
        {
            if (m_lastTwistMsg)
            {
                auto stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
                stamped_msg->header.stamp = this->now();
                stamped_msg->header.frame_id = "base_link";
                stamped_msg->twist = *m_lastTwistMsg;
                m_publisher->publish(*stamped_msg);
            }
        }

        /**
         * @brief: Callback for /cmd_vel Teleop node
        */
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            RCLCPP_INFO(get_logger(), "Cmd_vel callback Is Invoked");
            m_lastTwistMsg = msg;
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscriber; // Subscriber
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_publisher; // Publisher

        rclcpp::TimerBase::SharedPtr m_timer;
        geometry_msgs::msg::Twist::SharedPtr m_lastTwistMsg;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwistToTwistStampedNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
