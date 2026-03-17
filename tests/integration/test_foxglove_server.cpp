#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/debug/foxglove_server.hpp"

namespace {

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

struct HttpHandshake {
  std::string response{};
  std::vector<std::uint8_t> remainder{};
};

HttpHandshake ReadHttpResponse(int fd) {
  HttpHandshake handshake;
  std::string response;
  std::array<char, 1024> buffer{};
  while (response.find("\r\n\r\n") == std::string::npos) {
    const ssize_t rc = recv(fd, buffer.data(), buffer.size(), 0);
    if (rc <= 0) {
      break;
    }
    response.append(buffer.data(), static_cast<std::size_t>(rc));
  }
  const auto header_end = response.find("\r\n\r\n");
  if (header_end == std::string::npos) {
    handshake.response = response;
    return handshake;
  }
  handshake.response = response.substr(0, header_end + 4U);
  const auto payload_begin = header_end + 4U;
  if (payload_begin < response.size()) {
    handshake.remainder.assign(response.begin() + static_cast<std::ptrdiff_t>(payload_begin),
                               response.end());
  }
  return handshake;
}

std::string ReadTextFrame(int fd, std::vector<std::uint8_t>* remainder) {
  std::uint8_t header[2]{};
  std::size_t copied = 0U;
  if (remainder != nullptr) {
    while (!remainder->empty() && copied < sizeof(header)) {
      header[copied++] = remainder->front();
      remainder->erase(remainder->begin());
    }
  }
  if (copied < sizeof(header)) {
    assert(RecvExact(fd, header + copied, sizeof(header) - copied));
  }
  assert((header[0] & 0x0FU) == 0x1U);
  std::size_t size = header[1] & 0x7FU;
  if (size == 126U) {
    std::uint8_t ext[2]{};
    copied = 0U;
    if (remainder != nullptr) {
      while (!remainder->empty() && copied < sizeof(ext)) {
        ext[copied++] = remainder->front();
        remainder->erase(remainder->begin());
      }
    }
    if (copied < sizeof(ext)) {
      assert(RecvExact(fd, ext + copied, sizeof(ext) - copied));
    }
    size = (static_cast<std::size_t>(ext[0]) << 8U) | static_cast<std::size_t>(ext[1]);
  }
  std::string payload(size, '\0');
  std::size_t payload_copied = 0U;
  if (remainder != nullptr) {
    while (!remainder->empty() && payload_copied < size) {
      payload[payload_copied++] = static_cast<char>(remainder->front());
      remainder->erase(remainder->begin());
    }
  }
  if (payload_copied < size) {
    assert(RecvExact(fd, reinterpret_cast<std::uint8_t*>(payload.data()) + payload_copied,
                     size - payload_copied));
  }
  return payload;
}

std::vector<std::uint8_t> ReadBinaryFrame(int fd) {
  std::uint8_t header[2]{};
  assert(RecvExact(fd, header, sizeof(header)));
  assert((header[0] & 0x0FU) == 0x2U);
  std::size_t size = header[1] & 0x7FU;
  if (size == 126U) {
    std::uint8_t ext[2]{};
    assert(RecvExact(fd, ext, sizeof(ext)));
    size = (static_cast<std::size_t>(ext[0]) << 8U) | static_cast<std::size_t>(ext[1]);
  }
  std::vector<std::uint8_t> payload(size);
  if (size > 0U) {
    assert(RecvExact(fd, payload.data(), size));
  }
  return payload;
}

void SendMaskedTextFrame(int fd, const std::string& text) {
  const std::array<std::uint8_t, 4> mask = {0x12U, 0x34U, 0x56U, 0x78U};
  std::vector<std::uint8_t> frame;
  frame.reserve(text.size() + 8U);
  frame.push_back(0x81U);
  frame.push_back(static_cast<std::uint8_t>(0x80U | text.size()));
  frame.insert(frame.end(), mask.begin(), mask.end());
  for (std::size_t index = 0; index < text.size(); ++index) {
    frame.push_back(static_cast<std::uint8_t>(text[index]) ^ mask[index % mask.size()]);
  }
  assert(SendAll(fd, frame.data(), frame.size()));
}

}  // namespace

int main() {
  rm_nav::debug::FoxgloveServer server;
  rm_nav::debug::FoxgloveServerOptions options;
  options.host = "127.0.0.1";
  options.port = 0;
  assert(server.Start(options).ok());
  assert(server.Advertise({{1U, "/test/topic", "json", "test.Schema", "jsonschema",
                            "{\"type\":\"object\"}"}})
             .ok());

  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  assert(fd >= 0);
  timeval timeout{};
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(server.bound_port());
  assert(inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) == 1);
  assert(connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);

  const std::string request =
      "GET / HTTP/1.1\r\n"
      "Host: 127.0.0.1\r\n"
      "Upgrade: websocket\r\n"
      "Connection: Upgrade\r\n"
      "Sec-WebSocket-Version: 13\r\n"
      "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
      "Sec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n";
  assert(SendAll(fd, reinterpret_cast<const std::uint8_t*>(request.data()), request.size()));

  auto handshake = ReadHttpResponse(fd);
  assert(handshake.response.find("101 Switching Protocols") != std::string::npos);
  assert(handshake.response.find("Sec-WebSocket-Protocol: foxglove.sdk.v1") !=
         std::string::npos);

  const auto server_info = ReadTextFrame(fd, &handshake.remainder);
  const auto advertise = ReadTextFrame(fd, &handshake.remainder);
  assert(server_info.find("\"op\":\"serverInfo\"") != std::string::npos);
  assert(advertise.find("\"op\":\"advertise\"") != std::string::npos);
  assert(advertise.find("/test/topic") != std::string::npos);
  assert(advertise.find("eyJ0eXBlIj") == std::string::npos);
  assert(advertise.find("\\\"type\\\":\\\"object\\\"") != std::string::npos);

  SendMaskedTextFrame(fd, R"({"op":"subscribe","subscriptions":[{"id":42,"channelId":1}]})");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  assert(server.PublishJson(1U, R"({"value":123})", rm_nav::common::Now()).ok());
  const auto payload = ReadBinaryFrame(fd);
  assert(payload.size() > 13U);
  assert(payload[0] == 0x01U);
  const std::uint32_t subscription_id = static_cast<std::uint32_t>(payload[1]) |
                                        (static_cast<std::uint32_t>(payload[2]) << 8U) |
                                        (static_cast<std::uint32_t>(payload[3]) << 16U) |
                                        (static_cast<std::uint32_t>(payload[4]) << 24U);
  assert(subscription_id == 42U);
  const std::string message(payload.begin() + 13, payload.end());
  assert(message.find("\"value\":123") != std::string::npos);

  close(fd);
  server.Stop();
  std::cout << "test_foxglove_server passed\n";
  return 0;
}
