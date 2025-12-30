#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

namespace image_demo
{
    class ImagePublisher : public rclcpp::Node
    {
    public:
        explicit ImagePublisher(const rclcpp::NodeOptions &options)
            : Node("image_publisher", rclcpp::NodeOptions(options).use_intra_process_comms(true))
        {
            rclcpp::QoS qos(rclcpp::KeepLast(1));
            qos.best_effort();
            qos.durability_volatile();

            pub_ = create_publisher<sensor_msgs::msg::Image>("/image", qos);

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
                10ms, std::bind(&ImagePublisher::publish_image, this));

            RCLCPP_INFO(get_logger(), "Image publisher started");
        }

    private:
        void publish_image()
        {
            // const int width = 1920;
            // const int height = 1080;

            auto img = std::make_unique<sensor_msgs::msg::Image>();

            img->height = image_.rows;
            img->width = image_.cols;
            img->encoding = "bgr8";
            // img->encoding = "rgb8";
            img->is_bigendian = false;
            img->step = img->width * 3;
            img->data.resize(img->height * img->step);
            
            // for (int y = 0; y < image_.rows; ++y)
            // {
            //     const cv::Vec3b *src = image_.ptr<cv::Vec3b>(y);
            //     uint8_t *dst = img->data.data() + y * img->step;
            //     for (int x = 0; x < image_.cols; ++x)
            //     {
            //         dst[3 * x + 0] = src[x][2]; // R
            //         dst[3 * x + 1] = src[x][1]; // G
            //         dst[3 * x + 2] = src[x][0]; // B
            //     }
            // }
            std::memcpy(
                img->data.data(),
                image_.data,
                img->data.size());

            img->header.frame_id = "camera_frame";
            img->header.stamp = this->get_clock()->now();

            // for (int y = 0; y < height; ++y)
            // {
            //     for (int x = 0; x < width; ++x)
            //     {
            //         int idx = y * img->step + x * 3;
            //         if (x < width / 2)
            //         {
            //             // Nửa trái: đen
            //             img->data[idx + 0] = 0; // R
            //             img->data[idx + 1] = 0; // G
            //             img->data[idx + 2] = 0; // B
            //         }
            //         else
            //         {
            //             // Nửa phải: xanh dương
            //             img->data[idx + 0] = 0;   // R
            //             img->data[idx + 1] = 0;   // G
            //             img->data[idx + 2] = 255; // B
            //         }
            //     }
            // }

            pub_->publish(std::move(img));
        }

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::Mat image_;
    };
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(image_demo::ImagePublisher)
