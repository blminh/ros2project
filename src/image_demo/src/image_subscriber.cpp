#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("image_subscriber")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/image",
            10,
            std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Image subscriber started");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(
            get_logger(),
            "Received image: %ux%u | encoding=%s | first_byte=%u",
            msg->width,
            msg->height,
            msg->encoding.c_str(),
            msg->data.empty() ? 0 : msg->data[0]);
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
