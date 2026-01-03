#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

class LoanMsgSubscriber : public rclcpp::Node
{
public:
    LoanMsgSubscriber()
        : Node("loan_msg_subscriber")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.best_effort();
        qos.durability_volatile();

        sub_ = create_subscription<std_msgs::msg::Int32>(
            "/topic/loan_msg",
            qos,
            std::bind(&LoanMsgSubscriber::loan_msg_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::String>("/topic/transport_msg", qos);
        
        RCLCPP_INFO(get_logger(), "Loan msg subscriber started");
    }

private:
    void loan_msg_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Received loan msg: %d", msg->data);

        auto message = std_msgs::msg::String();
        message.data = "Pub second! " + std::to_string(msg->data);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        pub_->publish(message);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanMsgSubscriber>());
    rclcpp::shutdown();
    return 0;
}