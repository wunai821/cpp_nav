#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/time.hpp"
#include "rm_nav/debug/mirror_tap.hpp"

namespace rm_nav::debug {

struct FoxgloveChannel {
  std::uint32_t id{0};
  std::string topic{};
  std::string encoding{"json"};
  std::string schema_name{};
  std::string schema_encoding{"jsonschema"};
  std::string schema{};
};

struct FoxgloveServerOptions {
  std::string name{"rm_nav_mock"};
  std::string host{"127.0.0.1"};
  std::uint16_t port{8765};
};

class FoxgloveServer {
 public:
  FoxgloveServer();
  ~FoxgloveServer();

  FoxgloveServer(const FoxgloveServer&) = delete;
  FoxgloveServer& operator=(const FoxgloveServer&) = delete;

  common::Status Start(const FoxgloveServerOptions& options);
  void Stop();

  bool is_running() const;
  std::uint16_t bound_port() const;

  common::Status Advertise(const std::vector<FoxgloveChannel>& channels);
  common::Status PublishJson(std::uint32_t channel_id, std::string_view json,
                             common::TimePoint stamp);

  void Broadcast(const DebugEnvelope& envelope);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm_nav::debug
