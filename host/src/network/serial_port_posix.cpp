// POSIX (Linux/Mac) serial port backend -- termios against a /dev/tty*
// device. This is the implementation this project's own CI and Docker
// verification actually compile and can exercise (against a real pseudo-
// terminal pair, see tests/test_serial_port_posix.cpp), since there's no
// Windows machine in that environment. serial_port_windows.cpp is the
// real target for an actual USB-connected ESP32 on the user's own
// Windows machine -- see that file's header comment for why it's
// compiled separately.
#include "network/serial_port.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace imu {

struct SerialPort::Impl {
  int fd = -1;
};

SerialPort::SerialPort() : impl_(new Impl()) {}

SerialPort::~SerialPort() {
  close();
  delete impl_;
}

namespace {
speed_t baudToSpeed(uint32_t baudRate) {
  switch (baudRate) {
    case 9600: return B9600;
    case 115200: return B115200;
    case 921600: return B921600;  // config.h's SERIAL_BAUD -- the one this project actually uses
    default: return B115200;
  }
}
}  // namespace

bool SerialPort::open(const std::string& portName, uint32_t baudRate) {
  impl_->fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (impl_->fd < 0) return false;

  termios tty{};
  if (tcgetattr(impl_->fd, &tty) != 0) {
    ::close(impl_->fd);
    impl_->fd = -1;
    return false;
  }

  const speed_t speed = baudToSpeed(baudRate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // Raw mode: we frame bytes ourselves (FrameParser), we don't want the
  // OS doing line-buffering, echo, or signal-generation on our binary data.
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  tty.c_cc[VMIN] = 0;   // read() returns as soon as ANY bytes are available...
  tty.c_cc[VTIME] = 10; // ...or after a 1.0s timeout with none, whichever first

  if (tcsetattr(impl_->fd, TCSANOW, &tty) != 0) {
    ::close(impl_->fd);
    impl_->fd = -1;
    return false;
  }
  return true;
}

void SerialPort::close() {
  if (impl_->fd >= 0) {
    ::close(impl_->fd);
    impl_->fd = -1;
  }
}

bool SerialPort::isOpen() const { return impl_->fd >= 0; }

int SerialPort::read(uint8_t* buffer, size_t bufferSize) {
  if (impl_->fd < 0) return -1;
  ssize_t n = ::read(impl_->fd, buffer, bufferSize);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;  // no data yet, not an error
    return -1;
  }
  return static_cast<int>(n);
}

}  // namespace imu
