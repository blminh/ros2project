#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("image_subscriber")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.best_effort();
        qos.durability_volatile();
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image",
            qos,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Image subscriber started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time receive_time = now();
        rclcpp::Time send_time = msg->header.stamp;
        rclcpp::Duration latency = receive_time - send_time;

        RCLCPP_INFO(
            get_logger(),
            "Received image: %ux%u | encoding=%s | first_byte=%u | Latency: %.3f ms",
            msg->width,
            msg->height,
            msg->encoding.c_str(),
            msg->data.empty() ? 0 : msg->data[0],
            latency.seconds() * 1000.0
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
