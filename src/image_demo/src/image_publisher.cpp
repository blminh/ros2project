#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "shm_interfaces/msg/pod_image8m.hpp"

using namespace std::chrono_literals;

class SHMImagePublisher : public rclcpp::Node
{
private:
    using PodImage8m = shm_interfaces::msg::PodImage8m;

public:
    explicit SHMImagePublisher()
        : Node("shm_image_publisher")
    {
        rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

        pub_ = create_publisher<PodImage8m>("/shm_image", sensor_qos);

        std::string pkg_path = ament_index_cpp::get_package_share_directory("image_demo");
        image_ = cv::imread(pkg_path + "/images/img.jpg", cv::IMREAD_COLOR);
        if (image_.empty())
        {
            throw std::runtime_error("Failed to load image");
        }

        RCLCPP_INFO(
            get_logger(),
            "Loaded image: %dx%d",
            image_.cols,
            image_.rows);

        timer_ = create_wall_timer(
            100ms, std::bind(&SHMImagePublisher::publish_image, this));

        RCLCPP_INFO(get_logger(), "Image publisher started");
    }

private:
    void publish_image()
    {
        auto loaned_msg = pub_->borrow_loaned_message();

        loaned_msg.get().header.stamp = this->get_clock()->now();
        loaned_msg.get().header.frame_id.size = 12;
        strncpy((char *)loaned_msg.get().header.frame_id.data.data(), "camera_frame", 255);

        cv::resize(image_, image_, cv::Size(1920, 1080));

        loaned_msg.get().height = image_.rows;
        loaned_msg.get().width = image_.cols;
        loaned_msg.get().step = static_cast<uint32_t>(image_.cols * image_.elemSize());

        loaned_msg.get().encoding.size = 4; // strlen("bgr8")
        strncpy((char *)loaned_msg.get().encoding.data.data(), "bgr8", 255);

        size_t data_size = static_cast<size_t>(image_.total() * image_.elemSize());
        size_t buffer_size = loaned_msg.get().data.size();
        if (data_size > buffer_size)
        {
            RCLCPP_ERROR(get_logger(), "Image too big! data_size=%zu, buffer_size=%zu", data_size, buffer_size);
            std::cout.flush();
            return;
        }

        std::memcpy(
            loaned_msg.get().data.data(),
            image_.data,
            data_size);

            pub_->publish(std::move(loaned_msg));
    }

    rclcpp::Publisher<PodImage8m>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat image_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SHMImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
