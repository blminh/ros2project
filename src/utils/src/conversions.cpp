#include "shm_msgs/conversions.hpp"
#include <iostream>

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

    sensor_msgs::msg::Image toSensorImage(const shm_interfaces::msg::PodImage8m &source)
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
