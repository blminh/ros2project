#include <iostream>

#include <opencv2/imgproc.hpp>

#include "shm_msgs/conversions.hpp"
#include <shm_msgs/image_encodings.hpp>

namespace enc = shm_msgs::image_encodings;

namespace shm_msgs
{
    shm_interfaces::msg::PodImage8m::SharedPtr CvImage::toImageMsg8m() const
    {
        shm_interfaces::msg::PodImage8m::SharedPtr ptr = std::make_shared<shm_interfaces::msg::PodImage8m>();
        toImageMsg(*ptr);
        return ptr;
    }
    void CvImage::toImageMsg(shm_interfaces::msg::PodImage8m &ros_image) const
    {
        shm_msgs::set_header(ros_image.header, header);
        ros_image.height = image.rows;
        ros_image.width = image.cols;
        shm_msgs::set_str(ros_image.encoding, encoding);
        ros_image.step = image.cols * image.elemSize();
        size_t size = ros_image.step * image.rows;
        if (size > shm_interfaces::msg::PodImage8m::DATA_MAX_SIZE)
        {
            std::stringstream ss;
            ss << "Image is wrongly formed: height * step > size  or  " << ros_image.height << " * " << ros_image.step << " > " << shm_interfaces::msg::PodImage8m::DATA_MAX_SIZE;
            throw Exception(ss.str());
        }

        if (image.isContinuous())
        {
            memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
        }
        else
        {
            uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
            uchar *cv_data_ptr = image.data;
            for (int i = 0; i < image.rows; ++i)
            {
                memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
                ros_data_ptr += ros_image.step;
                cv_data_ptr += image.step;
            }
        }
    }

    CvImagePtr CvImage::cvtColor(
        const CvImageConstPtr &source,
        const std::string &target_encoding)
    {
        if (!source)
        {
            throw shm_msgs::Exception("CvImage::cvtColor: source is null");
        }

        // ===== YUY2 -> RGB8 =====
        if (source->encoding == "yuv422_yuy2" &&
            target_encoding == "rgb8")
        {
            auto out = std::make_shared<CvImage>();
            out->header = source->header;
            out->encoding = "rgb8";

            // source image must be CV_8UC2
            if (source->image.type() != CV_8UC2)
            {
                throw shm_msgs::Exception(
                    "Expected CV_8UC2 for yuv422_yuy2");
            }

            cv::cvtColor(source->image, out->image, cv::COLOR_YUV2RGB_YUY2);
            return out;
        }

        // ===== same encoding =====
        if (source->encoding == target_encoding)
        {
            auto out = std::make_shared<CvImage>();
            out->header = source->header;
            out->encoding = source->encoding;
            out->image = source->image;
            return out;
        }

        throw shm_msgs::Exception(
            "Unsupported encoding conversion: " +
            source->encoding + " -> " + target_encoding);
    }

    CvImagePtr toCvShareFromPodImage(
        const shm_interfaces::msg::PodImage8m &pod)
    {
        auto cv_img = std::make_shared<shm_msgs::CvImage>();

        cv_img->encoding = shm_msgs::get_str(pod.encoding);
        cv_img->image = cv::Mat(
            pod.height,
            pod.width,
            CV_8UC2, // YUY2
            const_cast<uint8_t *>(pod.data.data()),
            pod.step);

        return cv_img;
    }

    sensor_msgs::msg::Image fromPodImage8m(const shm_interfaces::msg::PodImage8m &source)
    {
        sensor_msgs::msg::Image sensorImage;
        sensorImage.header = shm_msgs::get_header(source.header);
        sensorImage.height = source.height;
        sensorImage.width = source.width;
        sensorImage.encoding = shm_msgs::get_str(source.encoding);
        sensorImage.step = source.step;
        const size_t size = source.data.size();
        sensorImage.data.resize(size);

        std::memcpy(sensorImage.data.data(), source.data.data(), size);
        return sensorImage;
    }
}
