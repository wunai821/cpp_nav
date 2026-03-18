#include <cassert>
#include <cstdint>
#include <iostream>
#include <vector>

#include "rm_nav/common/status.hpp"
#include "rm_nav/common/time.hpp"
#include "rm_nav/protocol/protocol_codec.hpp"

int main() {
  rm_nav::protocol::ProtocolCodec codec;
  rm_nav::protocol::ProtocolCodec rx_codec;

  rm_nav::protocol::HeartbeatPacket heartbeat;
  heartbeat.monotonic_ns = 123456789ULL;
  heartbeat.state = 7U;
  const auto heartbeat_bytes = codec.EncodeHeartbeat(heartbeat);
  assert(rx_codec.PushRxBytes(heartbeat_bytes.data(), heartbeat_bytes.size()).ok());
  const auto decoded_heartbeat_packet = rx_codec.TryDecode();
  assert(decoded_heartbeat_packet.has_value());
  rm_nav::protocol::HeartbeatPacket decoded_heartbeat;
  assert(codec.DecodeHeartbeat(*decoded_heartbeat_packet, &decoded_heartbeat).ok());
  assert(decoded_heartbeat.monotonic_ns == heartbeat.monotonic_ns);
  assert(decoded_heartbeat.state == heartbeat.state);

  rm_nav::protocol::NavCommandPacket nav_command;
  nav_command.cmd.stamp = rm_nav::common::FromNanoseconds(222333444);
  nav_command.cmd.vx_mps = 1.2F;
  nav_command.cmd.vy_mps = -0.3F;
  nav_command.cmd.wz_radps = 0.8F;
  nav_command.cmd.brake = true;
  const auto nav_bytes = codec.EncodeNavCommand(nav_command);
  assert(rx_codec.PushRxBytes(nav_bytes.data(), nav_bytes.size()).ok());
  const auto decoded_nav_packet = rx_codec.TryDecode();
  assert(decoded_nav_packet.has_value());
  rm_nav::protocol::NavCommandPacket decoded_nav_command;
  assert(codec.DecodeNavCommand(*decoded_nav_packet, &decoded_nav_command).ok());
  assert(decoded_nav_command.cmd.brake == nav_command.cmd.brake);
  assert(decoded_nav_command.cmd.vx_mps == nav_command.cmd.vx_mps);
  assert(decoded_nav_command.cmd.vy_mps == nav_command.cmd.vy_mps);
  assert(decoded_nav_command.cmd.wz_radps == nav_command.cmd.wz_radps);
  assert(rm_nav::common::ToNanoseconds(decoded_nav_command.cmd.stamp) ==
         rm_nav::common::ToNanoseconds(nav_command.cmd.stamp));

  rm_nav::protocol::OdomFeedbackPacket odom_feedback;
  odom_feedback.odom.stamp = rm_nav::common::FromNanoseconds(987654321);
  odom_feedback.odom.sequence = 42U;
  odom_feedback.odom.x_m = 1.5F;
  odom_feedback.odom.y_m = -2.0F;
  odom_feedback.odom.yaw_rad = 0.4F;
  odom_feedback.odom.vx_mps = 0.6F;
  odom_feedback.odom.vy_mps = 0.1F;
  odom_feedback.odom.wz_radps = -0.2F;
  const auto odom_bytes = codec.EncodeOdomFeedback(odom_feedback);
  assert(rx_codec.PushRxBytes(odom_bytes.data(), odom_bytes.size()).ok());
  const auto decoded_odom_packet = rx_codec.TryDecode();
  assert(decoded_odom_packet.has_value());
  rm_nav::protocol::OdomFeedbackPacket decoded_odom;
  assert(codec.DecodeOdomFeedback(*decoded_odom_packet, &decoded_odom).ok());
  assert(decoded_odom.odom.sequence == odom_feedback.odom.sequence);
  assert(decoded_odom.odom.x_m == odom_feedback.odom.x_m);
  assert(decoded_odom.odom.y_m == odom_feedback.odom.y_m);
  assert(decoded_odom.odom.yaw_rad == odom_feedback.odom.yaw_rad);

  rm_nav::protocol::RefereePacket referee;
  referee.state.stamp = rm_nav::common::FromNanoseconds(555666777);
  referee.state.sequence = 11U;
  referee.state.is_online = true;
  referee.state.game_stage = 3U;
  referee.state.robot_hp = 350U;
  referee.state.ammo = 120U;
  referee.state.remaining_time_s = 95U;
  const auto referee_bytes = codec.EncodeRefereeState(referee);
  assert(rx_codec.PushRxBytes(referee_bytes.data(), referee_bytes.size()).ok());
  const auto decoded_referee_packet = rx_codec.TryDecode();
  assert(decoded_referee_packet.has_value());
  rm_nav::protocol::RefereePacket decoded_referee;
  assert(codec.DecodeRefereeState(*decoded_referee_packet, &decoded_referee).ok());
  assert(decoded_referee.state.is_online == referee.state.is_online);
  assert(decoded_referee.state.game_stage == referee.state.game_stage);
  assert(decoded_referee.state.robot_hp == referee.state.robot_hp);
  assert(decoded_referee.state.ammo == referee.state.ammo);
  assert(decoded_referee.state.remaining_time_s == referee.state.remaining_time_s);

  auto corrupted_bytes = nav_bytes;
  corrupted_bytes.back() ^= 0xFFU;
  assert(rx_codec.PushRxBytes(corrupted_bytes.data(), corrupted_bytes.size()).ok());
  const auto corrupted_packet = rx_codec.TryDecode();
  assert(!corrupted_packet.has_value());

  std::cout << "test_protocol_codec passed\n";
  return 0;
}
