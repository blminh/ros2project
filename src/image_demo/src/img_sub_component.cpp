#include <memory>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace image_demo
{
    class ImageSubscriber : public rclcpp::Node
    {
    public:
        ImageSubscriber(const rclcpp::NodeOptions &options)
            : Node("image_subscriber", rclcpp::NodeOptions(options).use_intra_process_comms(true))
        {
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            qos.best_effort();
            qos.durability_volatile();

            sub_ = create_subscription<sensor_msgs::msg::Image>(
                "/image",
                qos,
                std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

            RCLCPP_INFO(get_logger(), "Image subscriber started");
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::UniquePtr msg)
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

            RCLCPP_INFO(
                get_logger(),
                "Received image: %ux%u | encoding=%s | first_byte=%u",
                msg->width,
                msg->height,
                msg->encoding.c_str(),
                msg->data.empty() ? 0 : msg->data[0]);
        }

        uint64_t count_ = 0;
        int64_t latency_accumulator_ = 0;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_demo::ImageSubscriber)
