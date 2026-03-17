#include "rm_nav/debug/foxglove_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rm_nav/utils/logger.hpp"

namespace rm_nav::debug {
namespace {

constexpr char kWebsocketGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
constexpr char kFoxgloveProtocolV1[] = "foxglove.sdk.v1";
constexpr char kFoxgloveProtocolLegacy[] = "foxglove.websocket.v1";

std::string Trim(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1U);
}

std::string ToLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

std::string JsonEscape(std::string_view value) {
  std::ostringstream stream;
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        stream << ch;
        break;
    }
  }
  return stream.str();
}

std::string Base64Encode(const std::uint8_t* data, std::size_t size) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((size + 2U) / 3U) * 4U);
  std::size_t index = 0U;
  while (index + 3U <= size) {
    const std::uint32_t chunk =
        (static_cast<std::uint32_t>(data[index]) << 16U) |
        (static_cast<std::uint32_t>(data[index + 1U]) << 8U) |
        static_cast<std::uint32_t>(data[index + 2U]);
    encoded.push_back(kAlphabet[(chunk >> 18U) & 0x3FU]);
    encoded.push_back(kAlphabet[(chunk >> 12U) & 0x3FU]);
    encoded.push_back(kAlphabet[(chunk >> 6U) & 0x3FU]);
    encoded.push_back(kAlphabet[chunk & 0x3FU]);
    index += 3U;
  }

  if (index < size) {
    std::uint32_t chunk = static_cast<std::uint32_t>(data[index]) << 16U;
    encoded.push_back(kAlphabet[(chunk >> 18U) & 0x3FU]);
    if (index + 1U < size) {
      chunk |= static_cast<std::uint32_t>(data[index + 1U]) << 8U;
      encoded.push_back(kAlphabet[(chunk >> 12U) & 0x3FU]);
      encoded.push_back(kAlphabet[(chunk >> 6U) & 0x3FU]);
      encoded.push_back('=');
    } else {
      encoded.push_back(kAlphabet[(chunk >> 12U) & 0x3FU]);
      encoded.push_back('=');
      encoded.push_back('=');
    }
  }

  return encoded;
}

std::string Base64Encode(std::string_view value) {
  return Base64Encode(reinterpret_cast<const std::uint8_t*>(value.data()), value.size());
}

std::array<std::uint32_t, 5> Sha1Digest(std::string_view value) {
  std::vector<std::uint8_t> message(value.begin(), value.end());
  const std::uint64_t bit_length = static_cast<std::uint64_t>(message.size()) * 8ULL;
  message.push_back(0x80U);
  while ((message.size() % 64U) != 56U) {
    message.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    message.push_back(static_cast<std::uint8_t>((bit_length >> shift) & 0xFFU));
  }

  std::uint32_t h0 = 0x67452301U;
  std::uint32_t h1 = 0xEFCDAB89U;
  std::uint32_t h2 = 0x98BADCFEU;
  std::uint32_t h3 = 0x10325476U;
  std::uint32_t h4 = 0xC3D2E1F0U;

  for (std::size_t block = 0; block < message.size(); block += 64U) {
    std::array<std::uint32_t, 80> words{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t offset = block + index * 4U;
      words[index] = (static_cast<std::uint32_t>(message[offset]) << 24U) |
                     (static_cast<std::uint32_t>(message[offset + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(message[offset + 2U]) << 8U) |
                     static_cast<std::uint32_t>(message[offset + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const std::uint32_t value =
          words[index - 3U] ^ words[index - 8U] ^ words[index - 14U] ^ words[index - 16U];
      words[index] = (value << 1U) | (value >> 31U);
    }

    std::uint32_t a = h0;
    std::uint32_t b = h1;
    std::uint32_t c = h2;
    std::uint32_t d = h3;
    std::uint32_t e = h4;

    for (std::size_t index = 0; index < 80U; ++index) {
      std::uint32_t f = 0U;
      std::uint32_t k = 0U;
      if (index < 20U) {
        f = (b & c) | ((~b) & d);
        k = 0x5A827999U;
      } else if (index < 40U) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1U;
      } else if (index < 60U) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCU;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6U;
      }
      const std::uint32_t temp =
          ((a << 5U) | (a >> 27U)) + f + e + k + words[index];
      e = d;
      d = c;
      c = (b << 30U) | (b >> 2U);
      b = a;
      a = temp;
    }

    h0 += a;
    h1 += b;
    h2 += c;
    h3 += d;
    h4 += e;
  }

  return {h0, h1, h2, h3, h4};
}

std::string WebsocketAccept(std::string_view sec_websocket_key) {
  const auto digest = Sha1Digest(std::string(sec_websocket_key) + kWebsocketGuid);
  std::array<std::uint8_t, 20> bytes{};
  for (std::size_t index = 0; index < digest.size(); ++index) {
    bytes[index * 4U] = static_cast<std::uint8_t>((digest[index] >> 24U) & 0xFFU);
    bytes[index * 4U + 1U] = static_cast<std::uint8_t>((digest[index] >> 16U) & 0xFFU);
    bytes[index * 4U + 2U] = static_cast<std::uint8_t>((digest[index] >> 8U) & 0xFFU);
    bytes[index * 4U + 3U] = static_cast<std::uint8_t>(digest[index] & 0xFFU);
  }
  return Base64Encode(bytes.data(), bytes.size());
}

bool SendAll(int fd, const std::uint8_t* data, std::size_t size) {
  std::size_t sent = 0U;
  while (sent < size) {
    const ssize_t rc = send(fd, data + sent, size - sent, 0);
    if (rc <= 0) {
      return false;
    }
    sent += static_cast<std::size_t>(rc);
  }
  return true;
}

bool SendTextFrame(int fd, std::string_view payload) {
  std::vector<std::uint8_t> frame;
  frame.reserve(payload.size() + 16U);
  frame.push_back(0x81U);
  if (payload.size() < 126U) {
    frame.push_back(static_cast<std::uint8_t>(payload.size()));
  } else if (payload.size() <= 0xFFFFU) {
    frame.push_back(126U);
    frame.push_back(static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU));
    frame.push_back(static_cast<std::uint8_t>(payload.size() & 0xFFU));
  } else {
    frame.push_back(127U);
    for (int shift = 56; shift >= 0; shift -= 8) {
      frame.push_back(static_cast<std::uint8_t>((payload.size() >> shift) & 0xFFU));
    }
  }
  frame.insert(frame.end(), payload.begin(), payload.end());
  return SendAll(fd, frame.data(), frame.size());
}

bool SendBinaryFrame(int fd, const std::vector<std::uint8_t>& payload) {
  std::vector<std::uint8_t> frame;
  frame.reserve(payload.size() + 16U);
  frame.push_back(0x82U);
  if (payload.size() < 126U) {
    frame.push_back(static_cast<std::uint8_t>(payload.size()));
  } else if (payload.size() <= 0xFFFFU) {
    frame.push_back(126U);
    frame.push_back(static_cast<std::uint8_t>((payload.size() >> 8U) & 0xFFU));
    frame.push_back(static_cast<std::uint8_t>(payload.size() & 0xFFU));
  } else {
    frame.push_back(127U);
    for (int shift = 56; shift >= 0; shift -= 8) {
      frame.push_back(static_cast<std::uint8_t>((payload.size() >> shift) & 0xFFU));
    }
  }
  frame.insert(frame.end(), payload.begin(), payload.end());
  return SendAll(fd, frame.data(), frame.size());
}

struct ParsedFrame {
  std::uint8_t opcode{0U};
  std::vector<std::uint8_t> payload{};
};

bool RecvExact(int fd, std::uint8_t* data, std::size_t size) {
  std::size_t received = 0U;
  while (received < size) {
    const ssize_t rc = recv(fd, data + received, size - received, 0);
    if (rc <= 0) {
      return false;
    }
    received += static_cast<std::size_t>(rc);
  }
  return true;
}

bool ReadFrame(int fd, ParsedFrame* frame) {
  if (frame == nullptr) {
    return false;
  }

  std::uint8_t header[2]{};
  if (!RecvExact(fd, header, sizeof(header))) {
    return false;
  }
  const std::uint8_t opcode = header[0] & 0x0FU;
  const bool masked = (header[1] & 0x80U) != 0U;
  std::uint64_t payload_size = header[1] & 0x7FU;
  if (payload_size == 126U) {
    std::uint8_t extended[2]{};
    if (!RecvExact(fd, extended, sizeof(extended))) {
      return false;
    }
    payload_size = (static_cast<std::uint64_t>(extended[0]) << 8U) |
                   static_cast<std::uint64_t>(extended[1]);
  } else if (payload_size == 127U) {
    std::uint8_t extended[8]{};
    if (!RecvExact(fd, extended, sizeof(extended))) {
      return false;
    }
    payload_size = 0U;
    for (std::uint8_t byte : extended) {
      payload_size = (payload_size << 8U) | static_cast<std::uint64_t>(byte);
    }
  }

  std::array<std::uint8_t, 4> mask{};
  if (masked && !RecvExact(fd, mask.data(), mask.size())) {
    return false;
  }

  frame->opcode = opcode;
  frame->payload.resize(static_cast<std::size_t>(payload_size));
  if (payload_size > 0U &&
      !RecvExact(fd, frame->payload.data(), static_cast<std::size_t>(payload_size))) {
    return false;
  }
  if (masked) {
    for (std::size_t index = 0; index < frame->payload.size(); ++index) {
      frame->payload[index] ^= mask[index % mask.size()];
    }
  }
  return true;
}

std::string BuildServerInfoJson(std::string_view name) {
  return "{\"op\":\"serverInfo\",\"name\":\"" + JsonEscape(name) +
         "\",\"capabilities\":[],\"supportedEncodings\":[\"json\"],\"metadata\":{}}";
}

std::string BuildAdvertiseJson(const std::vector<FoxgloveChannel>& channels) {
  std::ostringstream stream;
  stream << "{\"op\":\"advertise\",\"channels\":[";
  for (std::size_t index = 0; index < channels.size(); ++index) {
    const auto& channel = channels[index];
    const bool is_json_schema = channel.schema_encoding == "jsonschema";
    const std::string encoded_schema =
        is_json_schema ? JsonEscape(channel.schema) : Base64Encode(channel.schema);
    stream << "{"
           << "\"id\":" << channel.id << ","
           << "\"topic\":\"" << JsonEscape(channel.topic) << "\","
           << "\"encoding\":\"" << JsonEscape(channel.encoding) << "\","
           << "\"schemaName\":\"" << JsonEscape(channel.schema_name) << "\","
           << "\"schemaEncoding\":\"" << JsonEscape(channel.schema_encoding) << "\","
           << "\"schema\":\"" << encoded_schema << "\""
           << "}";
    if (index + 1U != channels.size()) {
      stream << ",";
    }
  }
  stream << "]}";
  return stream.str();
}

std::string BuildStatusJson(const DebugEnvelope& envelope) {
  return "{\"source_stage\":\"" + JsonEscape(envelope.source_stage) +
         "\",\"schema\":\"" + JsonEscape(envelope.schema) +
         "\",\"summary\":\"" + JsonEscape(envelope.summary) + "\"}";
}

bool ContainsProtocol(std::string_view offered, std::string_view wanted) {
  std::size_t begin = 0U;
  while (begin < offered.size()) {
    const auto comma = offered.find(',', begin);
    const auto token = Trim(std::string(offered.substr(begin, comma - begin)));
    if (token == wanted) {
      return true;
    }
    if (comma == std::string_view::npos) {
      break;
    }
    begin = comma + 1U;
  }
  return false;
}

bool ExtractUint(std::string_view text, std::string_view key, std::uint32_t* value,
                 std::size_t from = 0U) {
  if (value == nullptr) {
    return false;
  }
  const std::string pattern = "\"" + std::string(key) + "\"";
  const auto key_pos = text.find(pattern, from);
  if (key_pos == std::string_view::npos) {
    return false;
  }
  const auto colon = text.find(':', key_pos + pattern.size());
  if (colon == std::string_view::npos) {
    return false;
  }
  std::size_t begin = colon + 1U;
  while (begin < text.size() &&
         std::isspace(static_cast<unsigned char>(text[begin]))) {
    ++begin;
  }
  std::size_t end = begin;
  while (end < text.size() &&
         std::isdigit(static_cast<unsigned char>(text[end]))) {
    ++end;
  }
  if (end == begin) {
    return false;
  }
  *value = static_cast<std::uint32_t>(std::stoul(std::string(text.substr(begin, end - begin))));
  return true;
}

std::vector<std::pair<std::uint32_t, std::uint32_t>> ParseSubscribeRequest(std::string_view text) {
  std::vector<std::pair<std::uint32_t, std::uint32_t>> subscriptions;
  std::size_t cursor = 0U;
  while (true) {
    std::uint32_t id = 0U;
    std::uint32_t channel_id = 0U;
    const auto object_pos = text.find('{', cursor);
    if (object_pos == std::string_view::npos) {
      break;
    }
    if (!ExtractUint(text, "id", &id, object_pos) ||
        !ExtractUint(text, "channelId", &channel_id, object_pos)) {
      cursor = object_pos + 1U;
      continue;
    }
    subscriptions.emplace_back(id, channel_id);
    const auto end = text.find('}', object_pos);
    if (end == std::string_view::npos) {
      break;
    }
    cursor = end + 1U;
  }
  return subscriptions;
}

std::vector<std::uint32_t> ParseUnsubscribeRequest(std::string_view text) {
  std::vector<std::uint32_t> ids;
  const auto bracket = text.find('[');
  if (bracket == std::string_view::npos) {
    return ids;
  }
  std::size_t cursor = bracket + 1U;
  while (cursor < text.size()) {
    while (cursor < text.size() &&
           !std::isdigit(static_cast<unsigned char>(text[cursor]))) {
      ++cursor;
    }
    if (cursor >= text.size()) {
      break;
    }
    std::size_t end = cursor;
    while (end < text.size() &&
           std::isdigit(static_cast<unsigned char>(text[end]))) {
      ++end;
    }
    ids.push_back(static_cast<std::uint32_t>(
        std::stoul(std::string(text.substr(cursor, end - cursor)))));
    cursor = end + 1U;
  }
  return ids;
}

std::vector<std::uint8_t> BuildMessageData(std::uint32_t subscription_id,
                                           common::TimePoint stamp,
                                           std::string_view json_payload) {
  std::vector<std::uint8_t> buffer;
  buffer.reserve(1U + 4U + 8U + json_payload.size());
  buffer.push_back(0x01U);
  for (int shift = 0; shift < 32; shift += 8) {
    buffer.push_back(static_cast<std::uint8_t>((subscription_id >> shift) & 0xFFU));
  }
  const auto timestamp_ns = static_cast<std::uint64_t>(common::ToNanoseconds(stamp));
  for (int shift = 0; shift < 64; shift += 8) {
    buffer.push_back(static_cast<std::uint8_t>((timestamp_ns >> shift) & 0xFFU));
  }
  buffer.insert(buffer.end(), json_payload.begin(), json_payload.end());
  return buffer;
}

}  // namespace

class FoxgloveServer::Impl {
 public:
  struct ClientConnection {
    int fd{-1};
    std::mutex send_mutex{};
    std::mutex subscriptions_mutex{};
    std::unordered_map<std::uint32_t, std::uint32_t> subscriptions{};
    std::thread reader_thread{};
    std::atomic<bool> connected{true};
  };

  common::Status Start(const FoxgloveServerOptions& options) {
    if (running_.load(std::memory_order_acquire)) {
      return common::Status::InvalidArgument("foxglove server already running");
    }

    options_ = options;
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
      return common::Status::Unavailable("failed to create foxglove socket");
    }

    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(options.port);
    if (inet_pton(AF_INET, options.host.c_str(), &address.sin_addr) != 1) {
      close(listen_fd_);
      listen_fd_ = -1;
      return common::Status::InvalidArgument("foxglove host must be an IPv4 address");
    }
    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      close(listen_fd_);
      listen_fd_ = -1;
      return common::Status::Unavailable("failed to bind foxglove port");
    }
    if (listen(listen_fd_, 4) != 0) {
      close(listen_fd_);
      listen_fd_ = -1;
      return common::Status::Unavailable("failed to listen on foxglove port");
    }

    sockaddr_in actual{};
    socklen_t actual_size = sizeof(actual);
    if (getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&actual), &actual_size) == 0) {
      bound_port_.store(ntohs(actual.sin_port), std::memory_order_release);
    } else {
      bound_port_.store(options.port, std::memory_order_release);
    }

    running_.store(true, std::memory_order_release);
    accept_thread_ = std::thread([this]() { AcceptLoop(); });
    utils::LogInfo("foxglove", "listening on ws://" + options_.host + ":" +
                                    std::to_string(bound_port_.load()));
    return common::Status::Ok();
  }

  void Stop() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
      return;
    }

    if (listen_fd_ >= 0) {
      shutdown(listen_fd_, SHUT_RDWR);
      close(listen_fd_);
      listen_fd_ = -1;
    }
    if (accept_thread_.joinable()) {
      accept_thread_.join();
    }

    std::vector<std::shared_ptr<ClientConnection>> clients;
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients = clients_;
    }
    for (const auto& client : clients) {
      client->connected.store(false, std::memory_order_release);
      if (client->fd >= 0) {
        shutdown(client->fd, SHUT_RDWR);
        close(client->fd);
        client->fd = -1;
      }
    }
    for (const auto& client : clients) {
      if (client->reader_thread.joinable()) {
        client->reader_thread.join();
      }
    }
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients_.clear();
    }
  }

  bool is_running() const { return running_.load(std::memory_order_acquire); }

  std::uint16_t bound_port() const { return bound_port_.load(std::memory_order_acquire); }

  common::Status Advertise(const std::vector<FoxgloveChannel>& channels) {
    {
      std::lock_guard<std::mutex> lock(channels_mutex_);
      channels_ = channels;
      advertise_json_ = BuildAdvertiseJson(channels_);
    }
    BroadcastText(advertise_json_);
    return common::Status::Ok();
  }

  common::Status PublishJson(std::uint32_t channel_id, std::string_view json,
                             common::TimePoint stamp) {
    std::vector<std::shared_ptr<ClientConnection>> clients;
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients = clients_;
    }

    for (const auto& client : clients) {
      std::vector<std::uint32_t> matching_subscriptions;
      {
        std::lock_guard<std::mutex> lock(client->subscriptions_mutex);
        for (const auto& subscription : client->subscriptions) {
          if (subscription.second == channel_id) {
            matching_subscriptions.push_back(subscription.first);
          }
        }
      }
      for (const auto subscription_id : matching_subscriptions) {
        const auto payload = BuildMessageData(subscription_id, stamp, json);
        std::lock_guard<std::mutex> send_lock(client->send_mutex);
        if (!SendBinaryFrame(client->fd, payload)) {
          client->connected.store(false, std::memory_order_release);
          break;
        }
      }
    }
    CleanupClients();
    return common::Status::Ok();
  }

  void Broadcast(const DebugEnvelope& envelope) {
    PublishJson(0U, BuildStatusJson(envelope), envelope.stamp);
  }

 private:
  void AcceptLoop() {
    while (running_.load(std::memory_order_acquire)) {
      sockaddr_in address{};
      socklen_t address_size = sizeof(address);
      const int client_fd =
          accept(listen_fd_, reinterpret_cast<sockaddr*>(&address), &address_size);
      if (client_fd < 0) {
        if (running_.load(std::memory_order_acquire)) {
          utils::LogWarn("foxglove", "accept failed");
        }
        continue;
      }

      auto client = std::make_shared<ClientConnection>();
      client->fd = client_fd;
      if (!Handshake(client_fd)) {
        close(client_fd);
        continue;
      }

      {
        std::lock_guard<std::mutex> lock(client->send_mutex);
        SendTextFrame(client_fd, BuildServerInfoJson(options_.name));
        if (!advertise_json_.empty()) {
          SendTextFrame(client_fd, advertise_json_);
        }
      }

      client->reader_thread = std::thread([this, client]() { ReadLoop(client); });
      {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.push_back(client);
      }
    }
  }

  bool Handshake(int client_fd) const {
    std::string request;
    request.reserve(1024U);
    std::array<char, 1024> buffer{};
    while (request.find("\r\n\r\n") == std::string::npos) {
      const ssize_t rc = recv(client_fd, buffer.data(), buffer.size(), 0);
      if (rc <= 0) {
        return false;
      }
      request.append(buffer.data(), static_cast<std::size_t>(rc));
      if (request.size() > 16384U) {
        return false;
      }
    }

    std::istringstream stream(request);
    std::string request_line;
    std::getline(stream, request_line);
    if (request_line.find("GET ") != 0) {
      return false;
    }

    std::unordered_map<std::string, std::string> headers;
    std::string line;
    while (std::getline(stream, line)) {
      if (line == "\r" || line.empty()) {
        break;
      }
      const auto colon = line.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      const auto key = ToLower(Trim(line.substr(0, colon)));
      auto value = Trim(line.substr(colon + 1U));
      if (!value.empty() && value.back() == '\r') {
        value.pop_back();
      }
      headers[key] = value;
    }

    const auto key_it = headers.find("sec-websocket-key");
    if (key_it == headers.end()) {
      return false;
    }
    const auto protocol_it = headers.find("sec-websocket-protocol");
    std::string protocol = kFoxgloveProtocolV1;
    if (protocol_it != headers.end()) {
      if (ContainsProtocol(protocol_it->second, kFoxgloveProtocolV1)) {
        protocol = kFoxgloveProtocolV1;
      } else if (ContainsProtocol(protocol_it->second, kFoxgloveProtocolLegacy)) {
        protocol = kFoxgloveProtocolLegacy;
      }
    }

    const std::string response =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " +
        WebsocketAccept(key_it->second) + "\r\n" +
        "Sec-WebSocket-Protocol: " + protocol + "\r\n\r\n";
    return SendAll(client_fd, reinterpret_cast<const std::uint8_t*>(response.data()),
                   response.size());
  }

  void ReadLoop(const std::shared_ptr<ClientConnection>& client) {
    while (client->connected.load(std::memory_order_acquire)) {
      ParsedFrame frame;
      if (!ReadFrame(client->fd, &frame)) {
        break;
      }

      if (frame.opcode == 0x8U) {
        break;
      }
      if (frame.opcode == 0x9U) {
        std::lock_guard<std::mutex> lock(client->send_mutex);
        std::vector<std::uint8_t> pong;
        pong.push_back(0x8AU);
        pong.push_back(static_cast<std::uint8_t>(frame.payload.size()));
        pong.insert(pong.end(), frame.payload.begin(), frame.payload.end());
        SendAll(client->fd, pong.data(), pong.size());
        continue;
      }
      if (frame.opcode != 0x1U) {
        continue;
      }

      const std::string text(frame.payload.begin(), frame.payload.end());
      if (text.find("\"op\":\"subscribe\"") != std::string::npos ||
          text.find("\"op\": \"subscribe\"") != std::string::npos) {
        const auto subscriptions = ParseSubscribeRequest(text);
        std::lock_guard<std::mutex> lock(client->subscriptions_mutex);
        for (const auto& subscription : subscriptions) {
          client->subscriptions[subscription.first] = subscription.second;
        }
        continue;
      }
      if (text.find("\"op\":\"unsubscribe\"") != std::string::npos ||
          text.find("\"op\": \"unsubscribe\"") != std::string::npos) {
        const auto ids = ParseUnsubscribeRequest(text);
        std::lock_guard<std::mutex> lock(client->subscriptions_mutex);
        for (const auto id : ids) {
          client->subscriptions.erase(id);
        }
      }
    }

    client->connected.store(false, std::memory_order_release);
    if (client->fd >= 0) {
      shutdown(client->fd, SHUT_RDWR);
      close(client->fd);
      client->fd = -1;
    }
  }

  void BroadcastText(const std::string& text) {
    if (text.empty()) {
      return;
    }
    std::vector<std::shared_ptr<ClientConnection>> clients;
    {
      std::lock_guard<std::mutex> lock(clients_mutex_);
      clients = clients_;
    }
    for (const auto& client : clients) {
      if (!client->connected.load(std::memory_order_acquire)) {
        continue;
      }
      std::lock_guard<std::mutex> send_lock(client->send_mutex);
      if (!SendTextFrame(client->fd, text)) {
        client->connected.store(false, std::memory_order_release);
      }
    }
    CleanupClients();
  }

  void CleanupClients() {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    clients_.erase(
        std::remove_if(clients_.begin(), clients_.end(),
                       [](const std::shared_ptr<ClientConnection>& client) {
                         if (client->connected.load(std::memory_order_acquire)) {
                           return false;
                         }
                         if (client->reader_thread.joinable()) {
                           client->reader_thread.join();
                         }
                         return true;
                       }),
        clients_.end());
  }

  FoxgloveServerOptions options_{};
  int listen_fd_{-1};
  std::atomic<bool> running_{false};
  std::atomic<std::uint16_t> bound_port_{0};
  std::thread accept_thread_{};
  mutable std::mutex channels_mutex_{};
  std::vector<FoxgloveChannel> channels_{};
  std::string advertise_json_{};
  mutable std::mutex clients_mutex_{};
  std::vector<std::shared_ptr<ClientConnection>> clients_{};
};

FoxgloveServer::FoxgloveServer() : impl_(std::make_unique<Impl>()) {}

FoxgloveServer::~FoxgloveServer() { Stop(); }

common::Status FoxgloveServer::Start(const FoxgloveServerOptions& options) {
  return impl_->Start(options);
}

void FoxgloveServer::Stop() { impl_->Stop(); }

bool FoxgloveServer::is_running() const { return impl_->is_running(); }

std::uint16_t FoxgloveServer::bound_port() const { return impl_->bound_port(); }

common::Status FoxgloveServer::Advertise(const std::vector<FoxgloveChannel>& channels) {
  return impl_->Advertise(channels);
}

common::Status FoxgloveServer::PublishJson(std::uint32_t channel_id, std::string_view json,
                                           common::TimePoint stamp) {
  return impl_->PublishJson(channel_id, json, stamp);
}

void FoxgloveServer::Broadcast(const DebugEnvelope& envelope) {
  impl_->Broadcast(envelope);
}

}  // namespace rm_nav::debug
