#include "rm_nav/mapping/map_serializer.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string_view>
#include <vector>

namespace rm_nav::mapping {
namespace {

void AppendBigEndian32(std::vector<std::uint8_t>* bytes, std::uint32_t value) {
  bytes->push_back(static_cast<std::uint8_t>((value >> 24U) & 0xFFU));
  bytes->push_back(static_cast<std::uint8_t>((value >> 16U) & 0xFFU));
  bytes->push_back(static_cast<std::uint8_t>((value >> 8U) & 0xFFU));
  bytes->push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

std::uint32_t Adler32(const std::vector<std::uint8_t>& data) {
  std::uint32_t a = 1U;
  std::uint32_t b = 0U;
  for (const auto byte : data) {
    a = (a + byte) % 65521U;
    b = (b + a) % 65521U;
  }
  return (b << 16U) | a;
}

std::uint32_t Crc32(std::string_view type, const std::vector<std::uint8_t>& data) {
  std::uint32_t crc = 0xFFFFFFFFU;
  const auto update = [&crc](std::uint8_t byte) {
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = -(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  };
  for (const char ch : type) {
    update(static_cast<std::uint8_t>(ch));
  }
  for (const auto byte : data) {
    update(byte);
  }
  return ~crc;
}

void AppendChunk(std::vector<std::uint8_t>* png, std::string_view type,
                 const std::vector<std::uint8_t>& data) {
  AppendBigEndian32(png, static_cast<std::uint32_t>(data.size()));
  png->insert(png->end(), type.begin(), type.end());
  png->insert(png->end(), data.begin(), data.end());
  AppendBigEndian32(png, Crc32(type, data));
}

std::vector<std::uint8_t> BuildPngData(const data::GridMap2D& occupancy) {
  std::vector<std::uint8_t> raw;
  raw.reserve((static_cast<std::size_t>(occupancy.width) + 1U) * occupancy.height);
  for (std::uint32_t y = 0; y < occupancy.height; ++y) {
    raw.push_back(0U);
    for (std::uint32_t x = 0; x < occupancy.width; ++x) {
      const auto value =
          occupancy.occupancy[static_cast<std::size_t>(y) * occupancy.width + x];
      if (value == 255U) {
        raw.push_back(127U);
      } else {
        raw.push_back(static_cast<std::uint8_t>(255U - value));
      }
    }
  }

  std::vector<std::uint8_t> zlib;
  zlib.push_back(0x78U);
  zlib.push_back(0x01U);
  std::size_t offset = 0U;
  while (offset < raw.size()) {
    const std::size_t remaining = raw.size() - offset;
    const std::uint16_t block_size =
        static_cast<std::uint16_t>(std::min<std::size_t>(remaining, 65535U));
    const bool final_block = (offset + block_size) == raw.size();
    zlib.push_back(final_block ? 0x01U : 0x00U);
    zlib.push_back(static_cast<std::uint8_t>(block_size & 0xFFU));
    zlib.push_back(static_cast<std::uint8_t>((block_size >> 8U) & 0xFFU));
    const std::uint16_t nlen = static_cast<std::uint16_t>(~block_size);
    zlib.push_back(static_cast<std::uint8_t>(nlen & 0xFFU));
    zlib.push_back(static_cast<std::uint8_t>((nlen >> 8U) & 0xFFU));
    zlib.insert(zlib.end(), raw.begin() + static_cast<std::ptrdiff_t>(offset),
                raw.begin() + static_cast<std::ptrdiff_t>(offset + block_size));
    offset += block_size;
  }
  AppendBigEndian32(&zlib, Adler32(raw));
  return zlib;
}

common::Status WritePcd(const std::filesystem::path& path,
                        const std::vector<data::PointXYZI>& points) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return common::Status::Unavailable("failed to open global_map.pcd");
  }
  output << "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\n";
  output << "COUNT 1 1 1 1\nWIDTH " << points.size() << "\nHEIGHT 1\n";
  output << "VIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n";
  for (const auto& point : points) {
    output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << "\n";
  }
  return output.good() ? common::Status::Ok()
                       : common::Status::InternalError("failed to write global_map.pcd");
}

common::Status WriteBin(const std::filesystem::path& path, const data::GridMap2D& occupancy) {
  std::ofstream output(path, std::ios::binary);
  if (!output.is_open()) {
    return common::Status::Unavailable("failed to open occupancy.bin");
  }
  output.write(reinterpret_cast<const char*>(occupancy.occupancy.data()),
               static_cast<std::streamsize>(occupancy.occupancy.size()));
  return output.good() ? common::Status::Ok()
                       : common::Status::InternalError("failed to write occupancy.bin");
}

common::Status WritePng(const std::filesystem::path& path, const data::GridMap2D& occupancy) {
  const std::array<std::uint8_t, 8> signature = {
      0x89U, 0x50U, 0x4EU, 0x47U, 0x0DU, 0x0AU, 0x1AU, 0x0AU};
  std::vector<std::uint8_t> png(signature.begin(), signature.end());

  std::vector<std::uint8_t> ihdr;
  AppendBigEndian32(&ihdr, occupancy.width);
  AppendBigEndian32(&ihdr, occupancy.height);
  ihdr.push_back(8U);
  ihdr.push_back(0U);
  ihdr.push_back(0U);
  ihdr.push_back(0U);
  ihdr.push_back(0U);
  AppendChunk(&png, "IHDR", ihdr);

  const auto idat = BuildPngData(occupancy);
  AppendChunk(&png, "IDAT", idat);
  AppendChunk(&png, "IEND", {});

  std::ofstream output(path, std::ios::binary);
  if (!output.is_open()) {
    return common::Status::Unavailable("failed to open occupancy.png");
  }
  output.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
  return output.good() ? common::Status::Ok()
                       : common::Status::InternalError("failed to write occupancy.png");
}

common::Status WriteMeta(const std::filesystem::path& path, const data::GridMap2D& occupancy) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return common::Status::Unavailable("failed to open map_meta.json");
  }
  output << "{\n";
  output << "  \"width\": " << occupancy.width << ",\n";
  output << "  \"height\": " << occupancy.height << ",\n";
  output << "  \"resolution_m\": " << occupancy.resolution_m << ",\n";
  output << "  \"origin_x_m\": " << occupancy.origin.position.x << ",\n";
  output << "  \"origin_y_m\": " << occupancy.origin.position.y << "\n";
  output << "}\n";
  return output.good() ? common::Status::Ok()
                       : common::Status::InternalError("failed to write map_meta.json");
}

}  // namespace

common::Status MapSerializer::Save(const std::filesystem::path& output_dir,
                                   const std::vector<data::PointXYZI>& global_points,
                                   const data::GridMap2D& occupancy,
                                   MapArtifactPaths* artifact_paths) const {
  if (global_points.empty() || occupancy.occupancy.empty()) {
    return common::Status::InvalidArgument("mapping artifacts are empty");
  }

  std::filesystem::create_directories(output_dir);
  const auto pcd_path = output_dir / "global_map.pcd";
  const auto bin_path = output_dir / "occupancy.bin";
  const auto png_path = output_dir / "occupancy.png";
  const auto meta_path = output_dir / "map_meta.json";

  auto status = WritePcd(pcd_path, global_points);
  if (!status.ok()) {
    return status;
  }
  status = WriteBin(bin_path, occupancy);
  if (!status.ok()) {
    return status;
  }
  status = WritePng(png_path, occupancy);
  if (!status.ok()) {
    return status;
  }
  status = WriteMeta(meta_path, occupancy);
  if (!status.ok()) {
    return status;
  }

  if (artifact_paths != nullptr) {
    artifact_paths->global_map_pcd_path = pcd_path.lexically_normal().string();
    artifact_paths->occupancy_bin_path = bin_path.lexically_normal().string();
    artifact_paths->occupancy_png_path = png_path.lexically_normal().string();
    artifact_paths->map_meta_path = meta_path.lexically_normal().string();
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
