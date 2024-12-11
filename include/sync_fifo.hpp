// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
// MIT
#pragma once

#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <chrono>
#include <tuple>
#include <iostream>
#include <typeinfo>
#include <unordered_map>
#include <memory>
#include <type_traits>
#include <fstream>
#include "cereal/types/map.hpp"
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/access.hpp>
template <typename T, typename = void>
struct has_interpolate : std::false_type
{
};

template <typename T>
struct has_interpolate<T, std::void_t<decltype(std::declval<T>().interpolate(std::declval<T>(), std::declval<double>()))>> : std::is_same<decltype(std::declval<T>().interpolate(std::declval<T>(), std::declval<double>())), T>
{
};

template <typename measType>
class MeasFifo
{
public:
    // constructor
    MeasFifo(void) {}
    // destructor
    ~MeasFifo(void) {}

    /// @brief return the size of meas
    /// @return size_t
    size_t size(void)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        return meas_map.size();
    }

    /// @brief return true if the meas fifo is empty
    /// @return bool
    bool empty(void)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        return meas_map.empty();
    }

    /// @brief insert a meas to the fifo
    /// @param time
    /// @param meas
    void push(const double time, const measType &meas)
    {
        {
            std::lock_guard<std::mutex> lock(meas_mutex);
            this->meas_map.emplace(std::pair<double, measType>(time, meas));
        }
        valid_sig = true;
        if (this->is_fixed_size)
        {
            if (this->size() > this->max_size)
            {
                this->pop();
            }
        }
    }

    /// @brief pop the oldest meas from the fifo
    void pop(void)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        this->meas_map.erase(meas_map.begin());
    }

    /// @brief auto maintain the size of the fifo
    /// @param max_size
    void enable_fixed_size(const size_t max_size)
    {
        this->max_size = max_size;
        this->is_fixed_size = true;
    }

    /// @brief get the latest timestamp of a measurement in the fifo
    /// @return double
    double get_latest_time(void)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        return std::prev(meas_map.end())->first;
    }

    /// @brief get the oldest timestamp of a measurement in the fifo
    /// @return double
    double get_oldest_time(void)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        return meas_map.begin()->first;
    }

    std::string get_meas_type(void)
    {
        return typeid(measType).name();
    }

    /// @brief find a measurment at the given timestamp, will auto interpolate if the data structure has the interpolate function
    /// @param time_stamp
    /// @return pointer to the measurement or nullptr if not found
    std::shared_ptr<measType> lookup_measurement(const double time_stamp)
    {
        if (time_stamp < get_oldest_time() || time_stamp > get_latest_time())
            return nullptr;

        std::unique_lock<std::mutex> lock(meas_mutex);
        auto lower_bound_it = meas_map.lower_bound(time_stamp);

        if (lower_bound_it != meas_map.begin())
        {
            auto left_element = std::prev(lower_bound_it);
            lock.unlock();

            double ratio = (time_stamp - left_element->first) / (lower_bound_it->first - left_element->first);
            if constexpr (has_interpolate<measType>::value)
            {
                // ratio as the portion of the right element's contribution to the interpolation
                std::shared_ptr<measType> interpolated_meas = std::make_shared<measType>(left_element->second.interpolate(lower_bound_it->second, ratio));
                return interpolated_meas;
            }
            else if constexpr (std::is_same<measType, float>::value || std::is_same<measType, double>::value)
            {
                std::shared_ptr<measType> interpolated_meas = std::make_shared<measType>(left_element->second * (1 - ratio) + lower_bound_it->second * ratio);
                return interpolated_meas;
            }
            else
            {
                if (ratio < 0.5)
                {
                    return std::make_shared<measType>(left_element->second);
                }
                else
                {
                    return std::make_shared<measType>(lower_bound_it->second);
                }
            }
        }
        else
        {
            return std::make_shared<measType>(lower_bound_it->second);
        }

        return nullptr;
    }

    std::vector<std::pair<double, measType>> get_meas_sequence(const double left_time, const double right_time)
    {
        std::vector<std::pair<double, measType>> return_vec;
        if (empty())
            return return_vec;
        std::unique_lock<std::mutex> lock(meas_mutex);
        auto lower_bound_left_it = meas_map.lower_bound(left_time);

        auto lower_bound_right_it = meas_map.lower_bound(right_time);
        if(lower_bound_left_it != meas_map.end()&&lower_bound_right_it != meas_map.end())
        {
            for(auto it = lower_bound_left_it; it != lower_bound_right_it; it++)
            {
                return_vec.push_back(*it);
            }
        }
        return return_vec;
    }

    /// @brief save the fifo to a file
    /// @param file_name
    /// @return true if success
    bool save_to_file(const std::string &file_name)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        std::ofstream os(file_name, std::ios::binary);
        if (!os.is_open())
            return false;
        cereal::BinaryOutputArchive archive(os);
        archive(*this);
        os.close();
        return true;
    }

    /// @brief load the fifo from a file
    /// @param file_name
    /// @return true if success
    bool load_from_file(const std::string &file_name)
    {
        std::lock_guard<std::mutex> lock(meas_mutex);
        std::ifstream is(file_name, std::ios::binary);
        cereal::BinaryInputArchive iarchive(is);
        if (!is.is_open())
            return false;
        is.seekg(0, std::ios::end);
        if (is.tellg() == 0)
        {
            std::cerr << "The file is empty!" << std::endl;
            return false;
        }
        is.seekg(0, std::ios::beg);
        iarchive(*this);
        is.close();
        return true;
    }

    std::atomic<bool> valid_sig = false;

private:
    std::map<double, measType> meas_map;
    std::mutex meas_mutex;
    bool is_fixed_size = false;
    size_t max_size = 200;

    friend class cereal::access;
    template <class Archive>
    void serialize(Archive &archive)
    {
        archive(meas_map, is_fixed_size, max_size);
    }
};
