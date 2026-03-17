#include "rm_nav/drivers/serial/serial_port.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace rm_nav::drivers::serial {
namespace {

speed_t ToBaudRate(int baud_rate) {
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    default:
      return 0;
  }
}

}  // namespace

SerialPort::~SerialPort() { Close(); }

common::Status SerialPort::Open(const SerialConfig& config) {
  Close();

  const speed_t baud = ToBaudRate(config.baud_rate);
  if (baud == 0) {
    return common::Status::InvalidArgument("unsupported serial baud rate");
  }

  int flags = O_RDWR | O_NOCTTY;
  if (config.non_blocking) {
    flags |= O_NONBLOCK;
  }

  fd_ = ::open(config.device_path.c_str(), flags);
  if (fd_ < 0) {
    return common::Status::Unavailable("failed to open serial device");
  }

  termios tty {};
  if (tcgetattr(fd_, &tty) != 0) {
    Close();
    return common::Status::InternalError("failed to query serial attributes");
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<cc_t>(config.read_timeout_ms / 100);

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    Close();
    return common::Status::InternalError("failed to apply serial attributes");
  }

  config_ = config;
  return common::Status::Ok();
}

void SerialPort::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::IsOpen() const { return fd_ >= 0; }

common::Status SerialPort::Write(const std::uint8_t* data, std::size_t size,
                                 std::size_t* bytes_written) {
  if (!IsOpen()) {
    return common::Status::NotReady("serial port is not open");
  }
  if (bytes_written != nullptr) {
    *bytes_written = 0;
  }

  std::size_t total_written = 0;
  while (total_written < size) {
    const ssize_t result =
        ::write(fd_, data + total_written, size - total_written);
    if (result < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      return common::Status::InternalError("serial write failed");
    }
    total_written += static_cast<std::size_t>(result);
  }

  if (bytes_written != nullptr) {
    *bytes_written = total_written;
  }
  return common::Status::Ok();
}

common::Status SerialPort::Read(std::uint8_t* buffer, std::size_t capacity,
                                std::size_t* bytes_read) {
  if (!IsOpen()) {
    return common::Status::NotReady("serial port is not open");
  }
  if (bytes_read != nullptr) {
    *bytes_read = 0;
  }

  const ssize_t result = ::read(fd_, buffer, capacity);
  if (result < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      return common::Status::Ok();
    }
    return common::Status::InternalError("serial read failed");
  }

  if (bytes_read != nullptr) {
    *bytes_read = static_cast<std::size_t>(result);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::drivers::serial
