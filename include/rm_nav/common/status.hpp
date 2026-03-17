#pragma once

#include <string>
#include <string_view>

namespace rm_nav::common {

enum class StatusCode {
  kOk = 0,
  kNotReady,
  kInvalidArgument,
  kUnavailable,
  kUnimplemented,
  kInternalError,
};

struct Status {
  StatusCode code{StatusCode::kOk};
  std::string message{"ok"};

  bool ok() const { return code == StatusCode::kOk; }

  static Status Ok() { return {}; }
  static Status NotReady(std::string_view msg) {
    return {StatusCode::kNotReady, std::string(msg)};
  }
  static Status InvalidArgument(std::string_view msg) {
    return {StatusCode::kInvalidArgument, std::string(msg)};
  }
  static Status Unavailable(std::string_view msg) {
    return {StatusCode::kUnavailable, std::string(msg)};
  }
  static Status Unimplemented(std::string_view msg) {
    return {StatusCode::kUnimplemented, std::string(msg)};
  }
  static Status InternalError(std::string_view msg) {
    return {StatusCode::kInternalError, std::string(msg)};
  }
};

}  // namespace rm_nav::common
