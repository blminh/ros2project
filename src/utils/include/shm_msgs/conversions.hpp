#ifndef SHM_INTERFACES__CONVERSIONS_HPP_
#define SHM_INTERFACES__CONVERSIONS_HPP_

#include <ostream>
#include <stdexcept>
#include <string>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/core.hpp>

#include <shm_interfaces/msg/pod_string.hpp>
#include <shm_interfaces/msg/pod_image8m.hpp>
#include <shm_msgs/array_helper.hpp>

namespace shm_msgs
{
    class Exception : public std::runtime_error
    {
    public:
        explicit Exception(const std::string &description)
            : std::runtime_error(description) {}
    };

    class CvImage;
    typedef std::shared_ptr<CvImage> CvImagePtr;
    typedef std::shared_ptr<CvImage const> CvImageConstPtr;

    class CvImage
    {
    public:
        std_msgs::msg::Header header; // !< ROS header
        std::string encoding;         // !< Image encoding ("mono8", "bgr8", etc.)
        cv::Mat image;                // !< Image data for use with OpenCV

        /**
         * \brief Empty constructor.
         */
        CvImage() {}

        /**
         * \brief Constructor.
         */
        CvImage(
            const std_msgs::msg::Header &header, const std::string &encoding,
            const cv::Mat &image = cv::Mat())
            : header(header), encoding(encoding), image(image)
        {
        }

        shm_interfaces::msg::PodImage8m::SharedPtr toImageMsg8m() const;
        void toImageMsg(shm_interfaces::msg::PodImage8m &ros_image) const;

        CvImagePtr cvtColor(
            const CvImageConstPtr &source,
            const std::string &encoding);
    };

    sensor_msgs::msg::Image fromPodImage8m(const shm_interfaces::msg::PodImage8m &podImage8m);
    CvImagePtr toCvShareFromPodImage(const shm_interfaces::msg::PodImage8m &pod);
}

#endif // SHM_INTERFACES__CONVERSIONS_HPP_