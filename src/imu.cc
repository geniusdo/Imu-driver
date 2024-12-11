// Copyright 2024 Liu Chuangye @ chuangyeliu0206@gmail.com
// MIT
#include "imu.h"

#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

namespace imu_interface
{

    bool HighresImu::Open(const std::string dev)
    {
        this->handle = open(dev.c_str(), O_RDWR);
        if (this->handle == -1)
        {
            strerror(errno);
            return false;
        }
        return true;
    }
    void HighresImu::Close()
    {
        disable_device();
        close(this->handle);
        return;
    }

    bool HighresImu::configure_device()
    {
        struct termios tty;
        if (tcgetattr(this->handle, &tty) != 0)
        {
            printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }

        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 1; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 921600
        cfsetispeed(&tty, B921600);
        cfsetospeed(&tty, B921600);

        // Save tty settings, also checking for error
        if (tcsetattr(this->handle, TCSANOW, &tty) != 0)
        {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }
        struct serial_struct serinfo;
        if (ioctl(this->handle, TIOCGSERIAL, &serinfo) == -1)
        {
            perror("TIOCGSERIAL");
            return -1;
        }

        serinfo.xmit_fifo_size = 1024 * 1024;

        if (ioctl(this->handle, TIOCSSERIAL, &serinfo) == -1)
        {
            perror("TIOCSSERIAL");
            return -1;
        }
        return true;
    }

    bool HighresImu::start_acquisition()
    {
        if (this->feed_imu == nullptr)
        {
            printf("\033[31mError: feed_imu is not implemented\033[0m\n");
            printf("\033[31mError: feed_imu is not implemented\033[0m\n");
            printf("\033[31mError: feed_imu is not implemented\033[0m\n");
            printf("\033[31mError: feed_imu is not implemented\033[0m\n");
            printf("\033[31mError: feed_imu is not implemented\033[0m\n");
            return false;
        }
        this->running = true;
        std::thread parsing_thread = std::thread(&HighresImu::run, this);
        parsing_thread.detach();
        reset_count();
        enable_device();
        cal_time_offset();
        //TODO: make data acquisition after cal_time_offset
        return true;
    }

    void HighresImu::run()
    {
        uint8_t read_buf[36];
        while (this->running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            int num_bytes = read(this->handle, &read_buf, sizeof(read_buf));
            if (num_bytes > 0)
            {
                deserialize(read_buf);
                if (imu_flag == 1)
                {
                    // printf("curr id %d imu time diff: %f\n",this->imu_count, curr_imu(0) - last_imu_time);
                    // if(curr_imu(0) - last_imu_time>0.003)
                    //  printf("ERROR\n");
                    // last_imu_time = curr_imu(0);
                    auto imu = imu_type_t(curr_imu(0), curr_imu.segment(4, 3), curr_imu.segment(1, 3));
                    imu.count = this->imu_count;
                    this->feed_imu(imu.time, imu);
                    imu_flag = 0;
                }
                if (capture_sig_flag == 1)
                {
                    this->feed_time_sequence(this->capture_sig_count, this->capture_sig_time);
                    capture_sig_flag = 0;
                }
            }
        }
    }

    void HighresImu::deserialize(uint8_t *buffer)
    {

        auto decodeLittleEndianInt32 = [](const uint8_t *data) -> int32_t
        {
            return (data[0]) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
        };
        auto decodeLittleEndianUInt32 = [](const uint8_t *data) -> uint32_t
        {
            return (data[0]) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
        };

        if (buffer[0] == bag_head[0] && buffer[1] == bag_head[1] &&
            buffer[34] == bag_end[0] && buffer[35] == bag_end[1])
        {
            int offset = 6;
            // we minus 0.001s cuz the imu is actually the average of 5 imu samples from the raw data. raw frequency is 2000Hz
            // and its downsampled to 400Hz
            curr_imu(0) = static_cast<double>(decodeLittleEndianInt32(&buffer[0 + offset])) * 1e-6 + host_deivce_time_offset - 0.001;  
            curr_imu(1) = static_cast<double>(decodeLittleEndianInt32(&buffer[4 + offset])) / LSB_GYRO;
            curr_imu(2) = static_cast<double>(decodeLittleEndianInt32(&buffer[8 + offset])) / LSB_GYRO;
            curr_imu(3) = static_cast<double>(decodeLittleEndianInt32(&buffer[12 + offset])) / LSB_GYRO;
            curr_imu(4) = static_cast<double>(decodeLittleEndianInt32(&buffer[16 + offset])) / LSB_ACCE;
            curr_imu(5) = static_cast<double>(decodeLittleEndianInt32(&buffer[20 + offset])) / LSB_ACCE;
            curr_imu(6) = static_cast<double>(decodeLittleEndianInt32(&buffer[24 + offset])) / LSB_ACCE;
            this->imu_count = (decodeLittleEndianUInt32(&buffer[2])); // SOLVED
            // printf("imu count %d, curr time %f time diff %f wall time %f \n", this->imu_count, curr_imu(0), curr_imu(0) - last_time, static_cast<double>(std::chrono::system_clock::now().time_since_epoch().count()) / 1e9);
            // if (curr_imu(0) - last_time > 0.003)
            //     printf("ERROR\n");
            last_time = curr_imu(0);
            imu_flag = 1;
        }
        if (buffer[0] == capture_head[0] && buffer[1] == capture_head[1] &&
            buffer[34] == bag_end[0] && buffer[35] == bag_end[1])
        {
            int offset = 2;
            capture_sig_count = static_cast<int>(decodeLittleEndianInt32(&buffer[0 + offset]));
            capture_sig_time = static_cast<double>(decodeLittleEndianInt32(&buffer[4 + offset])) * 1e-6 + host_deivce_time_offset;
            capture_sig_flag = 1;
        }
        if (buffer[0] == sync_head[0] && buffer[1] == sync_head[1] &&
            buffer[34] == bag_end[0] && buffer[35] == bag_end[1])
        {
            int offset = 2;
            device_time = static_cast<double>(decodeLittleEndianInt32(&buffer[0 + offset])) * 1e-6;
            sync_flag = 1;
        }
    }

    void HighresImu::enable_device()
    {
        uint8_t cmd[2];
        cmd[0] = 0xab;
        cmd[1] = 0x11;
        write(this->handle, &cmd[0], sizeof(cmd));
        std::cout << "enable IMU device" << std::endl;
    }

    void HighresImu::disable_device()
    {
        uint8_t cmd[2];
        cmd[0] = 0xab;
        cmd[1] = 0x22;
        write(this->handle, &cmd[0], sizeof(cmd));
        std::cout << "disable IMU device" << std::endl;
    }

    void HighresImu::reset_count()
    {
        uint8_t cmd[2];
        cmd[0] = 0xab;
        cmd[1] = 0x33;
        write(this->handle, &cmd[0], sizeof(cmd));
        std::cout << "reset IMU device" << std::endl;
    }

    void HighresImu::sync_device()
    {
        uint8_t cmd[2];
        cmd[0] = 0xab;
        cmd[1] = 0x44;
        write(this->handle, &cmd[0], sizeof(cmd));
        std::cout << "sync IMU device" << std::endl;
    }

    void HighresImu::terminate_device()
    {
        uint8_t cmd[2];
        cmd[0] = 0xab;
        cmd[1] = 0x55;
        write(this->handle, &cmd[0], sizeof(cmd));
        std::cout << "terminate IMU device" << std::endl;
    }

    double HighresImu::cal_time_offset_once(void)
    {
        auto now1 = std::chrono::high_resolution_clock::now();
        this->sync_device();
        while (!sync_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        auto now2 = std::chrono::high_resolution_clock::now();
        auto two_round_time = std::chrono::duration_cast<std::chrono::microseconds>(now2.time_since_epoch()).count() -
                              std::chrono::duration_cast<std::chrono::microseconds>(now1.time_since_epoch()).count();
        double offset = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now1.time_since_epoch()).count()) * 1e-6 - (device_time + static_cast<double>(two_round_time) * 5e-7);
        this->sync_flag = 0;
        return offset;
    }

    void HighresImu::cal_time_offset()
    {
        double tmp_offset = 0;
        tmp_offset += cal_time_offset_once();
        tmp_offset += cal_time_offset_once();
        tmp_offset += cal_time_offset_once();
        tmp_offset /= 3;
        this->host_deivce_time_offset = tmp_offset;
        ;
        printf("device time: %f  time offset %f\n", device_time, host_deivce_time_offset);
    }
} // namespace imu_interface
