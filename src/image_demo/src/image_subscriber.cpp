#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "shm_interfaces/msg/pod_image8m.hpp"

class SHMImageSubscriber : public rclcpp::Node
{
private:
    using PodImage8m = shm_interfaces::msg::PodImage8m;

public:
    SHMImageSubscriber()
        : Node("shm_image_subscriber")
    {
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

        sub_ = create_subscription<PodImage8m>(
            "/shm_image",
            sensor_qos,
            std::bind(&SHMImageSubscriber::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Image subscriber started");

        pub_ = create_publisher<sensor_msgs::msg::Image>("/topic/sensor_image", sensor_qos);
    }

private:
    void image_callback(const PodImage8m::SharedPtr msg)
    {

        const rclcpp::Time now = this->get_clock()->now();
        const rclcpp::Time pub_time(msg->header.stamp);
        const int64_t latency_ns = (now - pub_time).nanoseconds();
        latency_accumulator_ += latency_ns;
        count_++;

        // 100 frame log 1 lan
        if (count_ % 100 == 0)
        {
            const double avg_latency_us =
                static_cast<double>(latency_accumulator_) /
                count_ / 1e3;

            RCLCPP_INFO(
                get_logger(),
                "Avg latency: %.3f us",
                avg_latency_us);
        }
        
        std::string encoding(reinterpret_cast<const char *>(msg->encoding.data.data()), msg->encoding.size);
        // RCLCPP_INFO(
        //     get_logger(),
        //     "Received image: %ux%u | encoding=%s | first_byte=%u",
        //     msg->width,
        //     msg->height,
        //     encoding.c_str(),
        //     msg->data.empty() ? 0 : msg->data[0]);

        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        image_msg->header.stamp = this->get_clock()->now();
        std::string frame_id(reinterpret_cast<const char *>(msg->header.frame_id.data.data()), msg->header.frame_id.size);
        image_msg->header.frame_id = frame_id;

        image_msg->height = msg->height;
        image_msg->width = msg->width;
        image_msg->step = msg->step;

        image_msg->encoding = encoding.c_str();

        size_t data_size = msg->data.size();
        image_msg->data.resize(data_size);

        std::memcpy(image_msg->data.data(), msg->data.data(), data_size);

        pub_->publish(*image_msg);
    }

    uint64_t count_ = 0;
    int64_t latency_accumulator_ = 0;
    rclcpp::Subscription<PodImage8m>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SHMImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
