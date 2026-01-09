#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "shm_interfaces/msg/pod_image8m.hpp"
#include "shm_msgs/conversions.hpp"

class SHMImageSubscriber : public rclcpp::Node
{
private:
    using PodImage8m = shm_interfaces::msg::PodImage8m;

public:
    SHMImageSubscriber()
        : Node("shm_image_subscriber")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.reliable();
        qos.durability_volatile();

        sub_ = create_subscription<PodImage8m>(
            "/shm_image",
            qos,
            std::bind(&SHMImageSubscriber::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Image subscriber started");

        pub_ = create_publisher<sensor_msgs::msg::Image>("/topic/sensor_image", qos);
    }

private:
    void image_callback(const PodImage8m::ConstSharedPtr &msg)
    {

        const rclcpp::Time now = this->get_clock()->now();
        const rclcpp::Time pub_time(msg->header.stamp);
        const int64_t latency_ns = (now - pub_time).nanoseconds();
        latency_accumulator_ += latency_ns;
        count_++;
        RCLCPP_INFO(get_logger(), ">>>>> SHMImageSubscriber ::: image_callback - count: %ld", count_);

        // 100 frame log 1 lan
        if (count_ % 100 == 0)
        {
            const double avg_latency_us =
                static_cast<double>(latency_accumulator_) /
                count_ / 1e3;

            RCLCPP_INFO(
                get_logger(),
                ">>>>> SHMImageSubscriber ::: image_callback - Avg latency: %.3f us",
                avg_latency_us);
        }

        auto sensorImg = shm_msgs::toSensorImage(*msg);
        pub_->publish(sensorImg);
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
