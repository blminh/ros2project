#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
        : Node("image_publisher")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.best_effort();
        qos.durability_volatile();
        
        pub_ = create_publisher<sensor_msgs::msg::Image>("/image", qos);

        timer_ = create_wall_timer(
            100ms, std::bind(&ImagePublisher::publish_image, this));

        RCLCPP_INFO(get_logger(), "Image publisher started");
    }

private:
    void publish_image()
    {
        const int width = 1920;
        const int height = 1080;

        sensor_msgs::msg::Image img;
        img.header.stamp = now();
        img.header.frame_id = "camera_frame";

        img.width = width;
        img.height = height;
        img.encoding = "rgb8";
        img.is_bigendian = false;
        img.step = width * 3;
        img.data.resize(height * img.step);

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int idx = y * img.step + x * 3;

                if (x < width / 2)
                {
                    // Nửa trái: đen
                    img.data[idx + 0] = 0; // R
                    img.data[idx + 1] = 0; // G
                    img.data[idx + 2] = 0; // B
                }
                else
                {
                    // Nửa phải: xanh dương
                    img.data[idx + 0] = 0;   // R
                    img.data[idx + 1] = 0;   // G
                    img.data[idx + 2] = 255; // B
                }
            }
        }

        pub_->publish(img);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
