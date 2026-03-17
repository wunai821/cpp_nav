#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/chassis_cmd.hpp"
#include "rm_nav/drivers/stm32/stm32_bridge.hpp"
#include "rm_nav/protocol/protocol_codec.hpp"
#include "rm_nav/utils/file_io.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  std::string device{};
  int baud_rate{115200};
  int duration_ms{1200};
  std::string record_path{};
};

struct PtyPair {
  int master_fd{-1};
  std::string slave_path{};
};

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--device" && index + 1 < argc) {
      options->device = argv[++index];
      continue;
    }
    if (arg == "--baud" && index + 1 < argc) {
      options->baud_rate = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--duration-ms" && index + 1 < argc) {
      options->duration_ms = std::atoi(argv[++index]);
      continue;
    }
    if (arg == "--record" && index + 1 < argc) {
      options->record_path = argv[++index];
      continue;
    }
    return rm_nav::common::Status::InvalidArgument("unsupported stm32_bridge_test arg");
  }
  return rm_nav::common::Status::Ok();
}

rm_nav::common::Status CreatePtyPair(PtyPair* pair) {
  if (pair == nullptr) {
    return rm_nav::common::Status::InvalidArgument("pty pair output is null");
  }
  const int master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (master_fd < 0) {
    return rm_nav::common::Status::InternalError("posix_openpt failed");
  }
  if (grantpt(master_fd) != 0 || unlockpt(master_fd) != 0) {
    close(master_fd);
    return rm_nav::common::Status::InternalError("grantpt/unlockpt failed");
  }

  char slave_name[128];
  if (ptsname_r(master_fd, slave_name, sizeof(slave_name)) != 0) {
    close(master_fd);
    return rm_nav::common::Status::InternalError("ptsname_r failed");
  }

  pair->master_fd = master_fd;
  pair->slave_path = slave_name;
  return rm_nav::common::Status::Ok();
}

void ClosePtyPair(PtyPair* pair) {
  if (pair != nullptr && pair->master_fd >= 0) {
    close(pair->master_fd);
    pair->master_fd = -1;
  }
}

class MockStm32Peer {
 public:
  explicit MockStm32Peer(int master_fd) : master_fd_(master_fd) {}
  ~MockStm32Peer() { Stop(); }

  void Start() { worker_ = std::thread([this]() { Run(); }); }
  void Stop() {
    stop_requested_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void Run() {
    rm_nav::protocol::ProtocolCodec codec;
    rm_nav::protocol::ProtocolCodec tx_codec;
    rm_nav::data::ChassisCmd last_cmd;
    rm_nav::data::OdomState odom;
    rm_nav::data::RefereeState referee;
    referee.is_online = true;
    referee.robot_hp = 400;
    referee.ammo = 150;
    referee.game_stage = 2;
    referee.remaining_time_s = 180;

    auto last_publish = rm_nav::common::Now();

    while (!stop_requested_.load()) {
      std::uint8_t buffer[256];
      const ssize_t bytes_read = read(master_fd_, buffer, sizeof(buffer));
      if (bytes_read > 0) {
        codec.PushRxBytes(buffer, static_cast<std::size_t>(bytes_read));
      }

      while (true) {
        auto decoded = codec.TryDecode();
        if (!decoded.has_value()) {
          break;
        }
        if (decoded->type == rm_nav::protocol::PacketType::kNavCommand) {
          rm_nav::protocol::NavCommandPacket command;
          if (codec.DecodeNavCommand(*decoded, &command).ok()) {
            last_cmd = command.cmd;
          }
        }
      }

      const auto now = rm_nav::common::Now();
      const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now - last_publish)
                                  .count();
      if (elapsed_ms >= 50) {
        const float dt = static_cast<float>(elapsed_ms) / 1000.0F;
        odom.stamp = now;
        odom.sequence += 1;
        odom.vx_mps = last_cmd.vx_mps;
        odom.vy_mps = last_cmd.vy_mps;
        odom.wz_radps = last_cmd.wz_radps;
        odom.x_m += odom.vx_mps * dt;
        odom.y_m += odom.vy_mps * dt;
        odom.yaw_rad += odom.wz_radps * dt;

        referee.stamp = now;
        referee.sequence += 1;
        const int remaining_time =
            std::max(0, 180 - static_cast<int>(referee.sequence));
        referee.remaining_time_s = static_cast<std::uint16_t>(remaining_time);

        const auto odom_bytes = tx_codec.EncodeOdomFeedback({odom});
        const auto referee_bytes = tx_codec.EncodeRefereeState({referee});
        write(master_fd_, odom_bytes.data(), odom_bytes.size());
        write(master_fd_, referee_bytes.data(), referee_bytes.size());
        last_publish = now;
      }

      rm_nav::common::SleepFor(std::chrono::milliseconds(10));
    }
  }

  int master_fd_{-1};
  std::atomic_bool stop_requested_{false};
  std::thread worker_{};
};

}  // namespace

int main(int argc, char** argv) {
  using rm_nav::utils::LogInfo;
  using rm_nav::utils::LogLevel;
  using rm_nav::utils::Logger;

  Logger::Instance().Initialize({LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("stm32_test", status.message);
    return 1;
  }

  PtyPair pair;
  std::unique_ptr<MockStm32Peer> mock_peer;
  if (options.device.empty()) {
    status = CreatePtyPair(&pair);
    if (!status.ok()) {
      rm_nav::utils::LogError("stm32_test", status.message);
      return 1;
    }
    options.device = pair.slave_path;
    mock_peer = std::make_unique<MockStm32Peer>(pair.master_fd);
    mock_peer->Start();
    LogInfo("stm32_test", std::string("mock STM32 slave path=") + options.device);
  }

  rm_nav::drivers::stm32::Stm32Bridge bridge;
  rm_nav::drivers::stm32::Stm32BridgeConfig config;
  config.serial.device_path = options.device;
  config.serial.baud_rate = options.baud_rate;
  config.serial.read_timeout_ms = 20;
  status = bridge.Configure(config);
  if (!status.ok()) {
    rm_nav::utils::LogError("stm32_test", status.message);
    ClosePtyPair(&pair);
    return 1;
  }

  rm_nav::utils::RecorderWriter recorder;
  if (!options.record_path.empty()) {
    status = recorder.Open(options.record_path);
    if (!status.ok()) {
      rm_nav::utils::LogError("stm32_test", status.message);
      bridge.Close();
      ClosePtyPair(&pair);
      return 1;
    }
  }

  const auto start = rm_nav::common::Now();
  auto next_heartbeat = start;
  auto next_cmd = start;
  std::size_t odom_count = 0;
  std::size_t referee_count = 0;

  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             rm_nav::common::Now() - start)
             .count() < options.duration_ms) {
    const auto now = rm_nav::common::Now();
    if (now >= next_heartbeat) {
      status = bridge.SendHeartbeat();
      if (!status.ok()) {
        rm_nav::utils::LogError("stm32_test", status.message);
        break;
      }
      next_heartbeat = now + std::chrono::milliseconds(200);
    }
    if (now >= next_cmd) {
      rm_nav::data::ChassisCmd cmd;
      cmd.stamp = now;
      cmd.vx_mps = 0.80F;
      cmd.vy_mps = 0.15F;
      cmd.wz_radps = 0.25F;
      status = bridge.SendChassisCmd(cmd);
      if (!status.ok()) {
        rm_nav::utils::LogError("stm32_test", status.message);
        break;
      }
      if (!options.record_path.empty()) {
        recorder.WriteChassisCmd(cmd);
      }
      next_cmd = now + std::chrono::milliseconds(100);
    }

    status = bridge.SpinOnce();
    if (!status.ok()) {
      rm_nav::utils::LogError("stm32_test", status.message);
      break;
    }

    if (auto odom = bridge.TakeOdomState(); odom.has_value()) {
      ++odom_count;
      LogInfo("stm32_test",
              "odom seq=" + std::to_string(odom->sequence) + " x=" +
                  std::to_string(odom->x_m) + " y=" + std::to_string(odom->y_m) +
                  " yaw=" + std::to_string(odom->yaw_rad));
      if (!options.record_path.empty()) {
        recorder.WriteOdomState(*odom);
      }
    }

    if (auto referee = bridge.TakeRefereeState(); referee.has_value()) {
      ++referee_count;
      LogInfo("stm32_test",
              "referee seq=" + std::to_string(referee->sequence) + " hp=" +
                  std::to_string(referee->robot_hp) + " ammo=" +
                  std::to_string(referee->ammo));
      if (!options.record_path.empty()) {
        recorder.WriteRefereeState(*referee);
      }
    }

    rm_nav::common::SleepFor(std::chrono::milliseconds(20));
  }

  recorder.Close();
  bridge.Close();
  if (mock_peer) {
    mock_peer->Stop();
  }
  ClosePtyPair(&pair);

  const auto health = bridge.health_report();
  LogInfo("stm32_test",
          "summary tx=" + std::to_string(health.tx_packets) + " rx=" +
              std::to_string(health.rx_packets) + " odom=" +
              std::to_string(odom_count) + " referee=" +
              std::to_string(referee_count) + " crc_errors=" +
              std::to_string(health.crc_errors) + " parse_errors=" +
              std::to_string(health.parse_errors));
  return 0;
}
