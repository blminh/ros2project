#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "shm_interfaces/msg/pod_image8m.hpp"
#include "shm_msgs/conversions.hpp"

using namespace std::chrono_literals;

class SHMImagePublisher : public rclcpp::Node
{
private:
    using PodImage8m = shm_interfaces::msg::PodImage8m;

public:
    explicit SHMImagePublisher()
        : Node("shm_image_publisher")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        qos.reliable();
        qos.durability_volatile();

        pub_ = create_publisher<PodImage8m>("/shm_image", qos);

        std::string pkg_path = ament_index_cpp::get_package_share_directory("image_demo");
        image_ = cv::imread(pkg_path + "/images/img.jpg", cv::IMREAD_COLOR);
        if (image_.empty())
        {
            throw std::runtime_error("Failed to load image");
        }

        m_input_cvimage->image = image_;
        m_input_cvimage->header.frame_id = "camera_link";
        m_input_cvimage->encoding = "bgr8";

        timer_ = create_wall_timer(
            100ms, std::bind(&SHMImagePublisher::publish_image, this));

        RCLCPP_INFO(get_logger(), "Image publisher started");
    }

private:
    void publish_image()
    {
        auto loanedMsg = pub_->borrow_loaned_message();
        auto &getLoanedMsg = loanedMsg.get();

        m_input_cvimage->header.stamp = this->get_clock()->now();
        cv::resize(m_input_cvimage->image, m_input_cvimage->image, cv::Size(1920, 1080));
        m_input_cvimage->toImageMsg(getLoanedMsg);

        pub_->publish(std::move(loanedMsg));
    }

    rclcpp::Publisher<PodImage8m>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat image_;
    std::shared_ptr<shm_msgs::CvImage> m_input_cvimage{std::make_shared<shm_msgs::CvImage>()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SHMImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
