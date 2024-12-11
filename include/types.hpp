// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
// MIT
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>

namespace base_types
{

    struct imu_type_t
    {
        double time; // unit: s
        uint32_t count;
        Eigen::Vector3d acce;
        Eigen::Vector3d gyro;
        imu_type_t() {}
        imu_type_t(double _time, const Eigen::Vector3d _acce, const Eigen::Vector3d _gyro) : time(_time), acce(_acce), gyro(_gyro) {}
        ~imu_type_t() {}
        imu_type_t interpolate(const imu_type_t &imu, double ratio) const
        {
            return imu_type_t(this->time * (1 - ratio) + imu.time * ratio,
                              this->acce * (1 - ratio) + imu.acce * ratio,
                              this->gyro * (1 - ratio) + imu.gyro * ratio);
        }
    };

    struct gray_image_type_t
    {
        double time;
        double exposure_time; // unit: us
        double gain;          // unit: dB
        int width;
        int height;
        uint64_t count;
        uint8_t *data;
        gray_image_type_t() {}
        gray_image_type_t(int w, int h) : width(w), height(h), data(new uint8_t[w * h]) {}
        ~gray_image_type_t() { delete[] data; }
    };

    struct rgb_image_type_t
    {
        double time;
        double exposure_time; // unit: us
        double gain;          // unit: dB
        int width;
        int height;
        uint64_t count;
        uint8_t *data;
        rgb_image_type_t(int w, int h) : width(w), height(h), data(new uint8_t[w * h * 3]) {}
        ~rgb_image_type_t() { delete[] data; }
    };

    using ImuType = imu_type_t;
    using ImuTypePtr = std::shared_ptr<imu_type_t>;
    using ImageType = gray_image_type_t;
    using ImageTypePtr = std::shared_ptr<gray_image_type_t>;
} // namespace base_type

namespace cereal
{
    using namespace base_types;

    template <class Archive>
    void serialize(Archive &archive, Eigen::Vector3f &v)
    {
        archive(v.x(), v.y(), v.z());
    }

    template <class Archive>
    void serialize(Archive &archive, Eigen::Vector3d &v)
    {
        archive(v.x(), v.y(), v.z());
    }

    template <class Archive>
    void serialize(Archive &archive, base_types::ImuType &v)
    {
        archive(v.time, v.count, v.acce, v.gyro);
    }

    template <class Archive>
    void save(Archive &archive, const base_types::ImageType &v)
    {
        archive(v.time, v.exposure_time, v.gain, v.width, v.height, v.count);
        // if (v.data != nullptr)
        // {
        //     archive(cereal::binary_data(v.data, v.width * v.height));
        // }

        archive(cereal::binary_data(v.data, v.width * v.height));
    }

    template <class Archive>
    void load(Archive &archive, base_types::ImageType &v)
    {
        archive(v.time, v.exposure_time, v.gain, v.width, v.height, v.count);
        // if (v.data != nullptr)
        // {
        //     archive(cereal::binary_data(v.data, v.width * v.height));
        // }

        v.data = new uint8_t[v.width * v.height];

        archive(cereal::binary_data(v.data, v.width * v.height));
    }
}