// Windows serial port backend -- Win32 CreateFileA/ReadFile against a COM
// port. THIS is the real target for the actual project: an ESP32 plugged
// into a Windows machine over USB shows up as a COM port (check Device
// Manager -> Ports (COM & LPT) for the exact number after plugging it
// in). Cross-compile-checked against real Windows headers via
// mingw-w64 (see hosting/HOSTING_GUIDE.md's verification notes) since no
// native Windows compiler was available in the environment this project
// was built in -- confirm it links and runs on your own machine as the
// very first bring-up step, the same honesty this repo already applies
// to "you must actually flash and power the real board yourself."
#ifdef _WIN32
#include "network/serial_port.hpp"

#include <windows.h>

namespace imu {

struct SerialPort::Impl {
  HANDLE handle = INVALID_HANDLE_VALUE;
};

SerialPort::SerialPort() : impl_(new Impl()) {}

SerialPort::~SerialPort() {
  close();
  delete impl_;
}

bool SerialPort::open(const std::string& portName, uint32_t baudRate) {
  // Windows needs the "\\.\COMx" form to open ports above COM9 -- and
  // it's a harmless no-op prefix for lower-numbered ports too, so it's
  // simplest to always use it.
  const std::string fullName = "\\\\.\\" + portName;
  impl_->handle = CreateFileA(fullName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                               OPEN_EXISTING, 0, nullptr);
  if (impl_->handle == INVALID_HANDLE_VALUE) return false;

  DCB dcb{};
  dcb.DCBlength = sizeof(DCB);
  if (!GetCommState(impl_->handle, &dcb)) {
    CloseHandle(impl_->handle);
    impl_->handle = INVALID_HANDLE_VALUE;
    return false;
  }
  dcb.BaudRate = baudRate;
  dcb.ByteSize = 8;
  dcb.StopBits = ONESTOPBIT;
  dcb.Parity = NOPARITY;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  if (!SetCommState(impl_->handle, &dcb)) {
    CloseHandle(impl_->handle);
    impl_->handle = INVALID_HANDLE_VALUE;
    return false;
  }

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 1000;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  SetCommTimeouts(impl_->handle, &timeouts);

  return true;
}

void SerialPort::close() {
  if (impl_->handle != INVALID_HANDLE_VALUE) {
    CloseHandle(impl_->handle);
    impl_->handle = INVALID_HANDLE_VALUE;
  }
}

bool SerialPort::isOpen() const { return impl_->handle != INVALID_HANDLE_VALUE; }

int SerialPort::read(uint8_t* buffer, size_t bufferSize) {
  if (impl_->handle == INVALID_HANDLE_VALUE) return -1;
  DWORD bytesRead = 0;
  if (!ReadFile(impl_->handle, buffer, static_cast<DWORD>(bufferSize), &bytesRead, nullptr)) {
    return -1;
  }
  return static_cast<int>(bytesRead);
}

}  // namespace imu
#endif  // _WIN32
