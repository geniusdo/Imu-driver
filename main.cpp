// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
// MIT
#include <unistd.h>
#include <iostream>
#include <string>
#include "imu.h"
#include "sync_fifo.hpp"

int main(int argc, char **argv)
{
    if (argc < 6)
    {
        std::cout << "Usage: " << argv[0] << " -d <device_name> -o <output_file> -t <seconds>" << std::endl;
        return 1;
    }

    std::string device_name;
    std::string output_name;
    int seconds = 0;
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "-d" && i + 1 < argc)
        {
            device_name = argv[i + 1];
            i++;
        }
        else if (std::string(argv[i]) == "-o" && i + 1 < argc)
        {
            output_name = argv[i + 1];
            i++;
        }
        else if (std::string(argv[i]) == "-t" && i + 1 < argc)
        {
            seconds = std::stoi(argv[i + 1]);
            i++;
        }
    }

    std::cout << "Device Name: " << device_name << std::endl;
    std::cout << "Output Name: " << output_name << std::endl;
    std::cout << "Seconds: " << seconds << std::endl;
    using namespace imu_interface;
    HighresImu imu_parser{};
    MeasFifo<ImuType> imu_fifo;
    imu_parser.register_data_callback(std::function<void(const double, const ImuType &)>(std::bind(static_cast<void (MeasFifo<ImuType>::*)(const double, const ImuType &)>(&MeasFifo<ImuType>::push), &imu_fifo, std::placeholders::_1, std::placeholders::_2)));
    if (!imu_parser.Open(device_name))
    {
        std::cout << "Failed to open device" << std::endl;
        return 1;
    }

    imu_parser.configure_device();
    imu_parser.start_acquisition();
    for (size_t i = 0; i < seconds; i++)
    {
        printf("%d seconds left", seconds - i);
        sleep(1);
    }
    printf("save to file\n");
    imu_fifo.save_to_file(output_name + ".bin");
    imu_parser.Close();
    return 1;
}