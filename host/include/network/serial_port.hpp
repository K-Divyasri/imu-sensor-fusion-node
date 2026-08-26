#pragma once
#include <cstdint>
#include <cstddef>
#include <string>

// The platform-SPECIFIC half of receiving frames -- just "open a named
// serial port, read raw bytes into a buffer, blocking with a timeout."
// No framing, no CRC, no knowledge of WireFrame at all -- that's
// FrameParser's job (network/frame_parser.hpp). This split means
// FrameParser can be fully unit-tested with zero serial port involved,
// while this class stays a thin, boring wrapper around whatever the OS
// actually offers.
//
// Two independent implementations exist (only one is compiled, chosen by
// CMakeLists.txt's WIN32 check): src/network/serial_port_windows.cpp
// (Win32 CreateFileA/ReadFile against a COM port -- the real target
// platform, since that's what a USB-connected ESP32 shows up as on
// Windows) and src/network/serial_port_posix.cpp (POSIX termios against
// a /dev/tty* device -- what this project's own CI/Docker verification
// actually compiles and runs against, since there's no Windows compiler
// available in that environment).
//
// The pimpl (pointer-to-implementation) pattern here is what lets this
// one header stay platform-agnostic -- no #ifdef _WIN32 anywhere in this
// file, since the platform-specific handle type (HANDLE vs int fd) only
// ever appears inside each .cpp's own private Impl struct.

namespace imu {

class SerialPort {
 public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // portName: "COM5" on Windows, "/dev/ttyUSB0" (or similar) on Linux/Mac.
  bool open(const std::string& portName, uint32_t baudRate);
  void close();
  bool isOpen() const;

  // Blocking read of up to bufferSize bytes. Returns the number of bytes
  // actually read (0 on a read timeout with no data -- not an error), or
  // a negative value on a real I/O error.
  int read(uint8_t* buffer, size_t bufferSize);

 private:
  struct Impl;
  Impl* impl_;
};

}  // namespace imu
