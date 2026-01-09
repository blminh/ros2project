#ifndef SHM_INTERFACES__ARRAY_HELPER_HPP_
#define SHM_INTERFACES__ARRAY_HELPER_HPP_

#include <string>
#include <memory>
#include <utility>
#include <cstring>

#include "std_msgs/msg/header.hpp"

#include "shm_interfaces/msg/pod_string.hpp"
#include "shm_interfaces/msg/pod_header.hpp"

namespace shm_msgs
{
    static inline void set_str(shm_interfaces::msg::PodString &shm_str, const std::string &std_str)
    {
        std::memcpy(shm_str.data.data(), std_str.data(), std_str.size());
        shm_str.size = std_str.size();
    }
    static inline std::string get_str(const shm_interfaces::msg::PodString &shm_str)
    {
        std::string std_str;
        std_str.resize(shm_str.size);
        std::memcpy((char *)std_str.data(), shm_str.data.data(), shm_str.size);
        return std_str;
    }
    
    static inline std_msgs::msg::Header get_header(const shm_interfaces::msg::PodHeader &shm_header)
    {
        std_msgs::msg::Header std_header;
        std_header.stamp = shm_header.stamp;
        std_header.frame_id = get_str(shm_header.frame_id);
        return std_header;
    }
    static inline void set_header(shm_interfaces::msg::PodHeader &shm_header, const std_msgs::msg::Header &std_header)
    {
        shm_header.stamp = std_header.stamp;
        set_str(shm_header.frame_id, std_header.frame_id);
    }
}

#endif // SHM_INTERFACES__ARRAY_HELPER_HPP_