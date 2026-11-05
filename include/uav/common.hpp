#pragma once

#include <stdexcept>
#include <string>

#ifndef UAV_VERSION
#define UAV_VERSION "1.0.0"
#endif

namespace uav {

/// Base exception type for every recoverable error raised by the library.
class Error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

[[nodiscard]] inline std::string version() { return UAV_VERSION; }

}  // namespace uav
