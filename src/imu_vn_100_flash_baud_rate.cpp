#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "vn100.h"

using VnErrorCode = VN_ERROR_CODE;

static void VnEnsure(const VnErrorCode& error_code, const std::string & msg)
{
  if (error_code == VNERR_NO_ERROR) return;

  switch (error_code) {
    case VNERR_UNKNOWN_ERROR:
      throw std::runtime_error("VN " + msg + ": Unknown error");
    case VNERR_NOT_IMPLEMENTED:
      throw std::runtime_error("VN " + msg + ": Not implemented");
    case VNERR_TIMEOUT:
      throw std::runtime_error("VN " + msg + ": Operation timed out");
    case VNERR_SENSOR_INVALID_PARAMETER:
      throw std::runtime_error("VN " + msg + ": Sensor invalid paramter");
    case VNERR_INVALID_VALUE:
      throw std::runtime_error("VN " + msg + ": Invalid value");
    case VNERR_FILE_NOT_FOUND:
      throw std::runtime_error("VN " + msg + ": File not found");
    case VNERR_NOT_CONNECTED:
      throw std::runtime_error("VN " + msg + ": not connected");
    case VNERR_PERMISSION_DENIED:
      throw std::runtime_error("VN " + msg + ": Permission denied");
    default:
      throw std::runtime_error("VN " + msg + ": Unhandled error type");
  }
}

int main(int argc, char *argv[])
{
  if (argc != 4) {
    fprintf(stderr, "A utility to permanently flash a new baud rate to the VN100T\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "Usage: %s <port> <connect_baudrate> <baud_rate_to_set>\n", argv[0]);
    return 1;
  }

  std::string port = std::string(argv[1]);
  uint32_t initial_baudrate = std::stoul(argv[2]);
  uint32_t baudrate = std::stoul(argv[3]);

  Vn100 imu;

  fprintf(stderr, "Connecting to device\n");
  VnEnsure(vn100_connect(&imu, port.c_str(), initial_baudrate), "Connect");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  fprintf(stderr, "Connected to device at %s\n", port.c_str());

  uint32_t old_baudrate;
  VnEnsure(vn100_getSerialBaudRate(&imu, &old_baudrate), "getSerialBaudRate");
  fprintf(stderr, "Default serial baudrate: %u\n", old_baudrate);

  if (old_baudrate != baudrate) {
    fprintf(stderr, "Set serial baudrate to %u\n", baudrate);
    VnEnsure(vn100_setSerialBaudRate(&imu, baudrate, true), "setSerialBaudRate");

    fprintf(stderr, "Disconnecting from device\n");
    vn100_disconnect(&imu);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    fprintf(stderr, "Reconnecting to device\n");
    VnEnsure(vn100_connect(&imu, port.c_str(), baudrate), "Reconnect");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    fprintf(stderr, "Connected to device at %s\n", port.c_str());

    fprintf(stderr, "Permanently writing baudrate to device\n");
    VnEnsure(vn100_writeSettings(&imu, true), "writeSettings");
  } else {
    fprintf(stderr, "Baudrate already set at requested %u, skipping\n", baudrate);
  }

  fprintf(stderr, "Disconnecting from the device\n");
  vn100_disconnect(&imu);

  return 0;
}
