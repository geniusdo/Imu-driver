// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
// MIT
#pragma once
#include <string>
#include <thread>
#include <chrono>
#include <iostream>
#include <functional>
#include <atomic>
#include <queue>
#include <mutex>

#include "types.hpp"
namespace imu_interface
{
    using namespace base_types;
    /*

    */
    class HighresImu
    {
    public:
        // constructor
        HighresImu() {

        };
        // destructor
        ~HighresImu() { this->Close(); };

        /*
        we will start a thread that keeps collecting data from serial device and pushes them
        into the buffer continously by calling this function.
        */
        bool start_acquisition();

        bool configure_device();

        bool Open(const std::string dev);

        void Close();

        void register_data_callback(std::function<void(double, const ImuType &)> func)
        {
            this->feed_imu = func;
        }

        void register_stamp_callback(std::function<void(int, double)> func)
        {
            this->feed_time_sequence = func;
        }

        void cal_time_offset();

    private:
        void deserialize(uint8_t *buffer);
        void run();
        void enable_device();
        void disable_device();
        void reset_count();
        void terminate_device();
        void sync_device();
        double cal_time_offset_once();
        int handle;
        std::function<void(double, const ImuType &)> feed_imu;
        std::function<void(int, double)> feed_time_sequence;
        std::atomic<bool> running;
        Eigen::Matrix<double, 7, 1> curr_imu; // the first element is timestamp the other 6 are the imu data
        uint8_t imu_flag = 0;
        uint8_t capture_sig_flag = 0;
        uint8_t sync_flag = 0;
        uint32_t capture_sig_count = 0;
        uint32_t imu_count = 0;
        double capture_sig_time = 0;
        double device_time = 0;
        double last_time = 0;
        double host_deivce_time_offset = 0;
        const double LSB_GYRO = 655360.0 * 180 / 3.1415926535;
        const double LSB_ACCE = 5346304.255829;
        const uint8_t bag_head[2] = {0xf0, 0xfd};
        const uint8_t capture_head[2] = {0xf0, 0xfe};
        const uint8_t sync_head[2] = {0xf0, 0xfa};
        const uint8_t bag_end[2] = {0xe0, 0xeb};
    };
} // imu_interface