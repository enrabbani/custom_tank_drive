#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
#include <mutex>


class I2C {
    public:
        I2C(const std::string& dev = "/dev/i2c-1", int addr = 0x10);
        ~I2C();

        // Write raw bytes to the slave. Returns true on full write.
        bool sendData(const uint8_t* data, size_t len);
        bool sendData(const std::vector<uint8_t>& data);

        // Read exactly len bytes from the slave into out.
        // NOTE: This is a master-initiated read; Teensy must respond via Wire.onRequest().
        bool receiveData(uint8_t* out, size_t len);
        bool receiveData(std::vector<uint8_t>& out, size_t len);

        bool isOpen() const { return fd_ >= 0; }

    private:
        int fd_ = -1;
        int addr_ = 0;
        std::mutex io_mutex_;

        bool setSlaveAddress(int addr);
};
