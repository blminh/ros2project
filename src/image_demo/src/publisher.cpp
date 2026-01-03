#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class LoanMsgPublisher : public rclcpp::Node
{
public:
    LoanMsgPublisher()
        : Node("loan_msg_publisher")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.best_effort();
        qos.durability_volatile();

        pub_ = create_publisher<std_msgs::msg::Int32>("/topic/loan_msg", qos);

        timer_ = create_wall_timer(
            100ms, std::bind(&LoanMsgPublisher::publish_loan_msg, this));

        RCLCPP_INFO(get_logger(), "Image publisher started");
    }

private:
    void publish_loan_msg()
    {
        auto loaned_msg = pub_->borrow_loaned_message();
        auto &msg = loaned_msg.get();
        msg.data = counter_++;
        RCLCPP_INFO(get_logger(), "Published int: %d", msg.data);
        pub_->publish(std::move(loaned_msg));
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LoanMsgPublisher>());
    rclcpp::shutdown();
    return 0;
}