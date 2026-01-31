
#include "robot_controller/teensy_i2c.hpp"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

I2C::I2C(const std::string& dev, int addr) : addr_(addr) {
  fd_ = ::open(dev.c_str(), O_RDWR);

  if (fd_ < 0) {
    return;
  }

  setSlaveAddress(addr_);
}

I2C::~I2C() {
  if (fd_ >= 0) ::close(fd_);
}

bool I2C::setSlaveAddress(int addr) {
  if (fd_ < 0) return false;

  if (ioctl(fd_, I2C_SLAVE, addr) < 0) return false;

  addr_ = addr;
  return true;
}

bool I2C::sendData(const uint8_t* data, size_t len) {
  if (fd_ < 0 || data == nullptr || len == 0) return false;
  
  std::lock_guard<std::mutex> lock(io_mutex_);

  const ssize_t n = ::write(fd_, data, len);
  return n == static_cast<ssize_t>(len);
}

bool I2C::sendData(const std::vector<uint8_t>& data) {
  return sendData(data.data(), data.size());
}

bool I2C::receiveData(uint8_t* out, size_t len) {
  if (fd_ < 0 || out == nullptr || len == 0) return false;
  std::lock_guard<std::mutex> lock(io_mutex_);

  const ssize_t n = ::read(fd_, out, len);
  return n == static_cast<ssize_t>(len);
}

bool I2C::receiveData(std::vector<uint8_t>& out, size_t len) {
  out.resize(len);
  return receiveData(out.data(), len);
}
