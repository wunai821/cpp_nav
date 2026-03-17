#include "rm_nav/localization/map_loader.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::localization {
namespace {

std::filesystem::path ResolveAssetPath(const std::string& config_dir,
                                       const std::string& raw_path) {
  const std::filesystem::path path(raw_path);
  if (path.is_absolute()) {
    return path;
  }
  return std::filesystem::path(config_dir) / path;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return {};
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  return stream.str();
}

bool ExtractJsonNumber(const std::string& text, const std::string& key, double* value) {
  if (value == nullptr) {
    return false;
  }
  const std::string pattern = "\"" + key + "\"";
  const std::size_t key_pos = text.find(pattern);
  if (key_pos == std::string::npos) {
    return false;
  }
  const std::size_t colon_pos = text.find(':', key_pos + pattern.size());
  if (colon_pos == std::string::npos) {
    return false;
  }
  std::size_t begin = colon_pos + 1U;
  while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin]))) {
    ++begin;
  }
  std::size_t end = begin;
  while (end < text.size() &&
         (std::isdigit(static_cast<unsigned char>(text[end])) || text[end] == '-' ||
          text[end] == '+' || text[end] == '.' || text[end] == 'e' ||
          text[end] == 'E')) {
    ++end;
  }
  if (begin == end) {
    return false;
  }
  try {
    *value = std::stod(text.substr(begin, end - begin));
    return true;
  } catch (...) {
    return false;
  }
}

common::Status LoadMapMeta(const std::filesystem::path& path, data::GridMap2D* occupancy) {
  if (occupancy == nullptr) {
    return common::Status::InvalidArgument("occupancy output is null");
  }
  const std::string text = ReadTextFile(path);
  if (text.empty()) {
    return common::Status::Unavailable("map meta file is empty");
  }

  double width = 0.0;
  double height = 0.0;
  double resolution = occupancy->resolution_m;
  double origin_x = 0.0;
  double origin_y = 0.0;
  if (!ExtractJsonNumber(text, "width", &width) ||
      !ExtractJsonNumber(text, "height", &height) ||
      !ExtractJsonNumber(text, "resolution_m", &resolution)) {
    return common::Status::InvalidArgument("map meta missing required fields");
  }
  ExtractJsonNumber(text, "origin_x_m", &origin_x);
  ExtractJsonNumber(text, "origin_y_m", &origin_y);

  occupancy->width = static_cast<std::uint32_t>(std::max(0.0, width));
  occupancy->height = static_cast<std::uint32_t>(std::max(0.0, height));
  occupancy->resolution_m = static_cast<float>(resolution);
  occupancy->origin.reference_frame = tf::kMapFrame;
  occupancy->origin.child_frame = tf::kMapFrame;
  occupancy->origin.position.x = static_cast<float>(origin_x);
  occupancy->origin.position.y = static_cast<float>(origin_y);
  occupancy->origin.is_valid = true;
  return common::Status::Ok();
}

common::Status LoadOccupancyBin(const std::filesystem::path& path, data::GridMap2D* occupancy) {
  if (occupancy == nullptr) {
    return common::Status::InvalidArgument("occupancy output is null");
  }
  if (occupancy->width == 0U || occupancy->height == 0U) {
    return common::Status::InvalidArgument("occupancy dimensions must be set before bin load");
  }

  const std::size_t cell_count =
      static_cast<std::size_t>(occupancy->width) * occupancy->height;
  occupancy->occupancy.assign(cell_count, 0U);

  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) {
    return common::Status::Unavailable("failed to open occupancy bin");
  }
  input.read(reinterpret_cast<char*>(occupancy->occupancy.data()),
             static_cast<std::streamsize>(cell_count));
  if (!input.good() && !input.eof()) {
    return common::Status::InvalidArgument("failed to read occupancy bin");
  }
  return common::Status::Ok();
}

common::Status LoadPointCloudPcd(const std::filesystem::path& path,
                                 std::vector<data::PointXYZI>* points) {
  if (points == nullptr) {
    return common::Status::InvalidArgument("point cloud output is null");
  }
  std::ifstream input(path);
  if (!input.is_open()) {
    return common::Status::Unavailable("failed to open global map pcd");
  }

  points->clear();
  std::string line;
  bool data_section = false;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    if (!data_section) {
      if (line.rfind("DATA", 0) == 0) {
        data_section = true;
      }
      continue;
    }

    std::istringstream stream(line);
    data::PointXYZI point;
    point.relative_time_s = 0.0F;
    if (!(stream >> point.x >> point.y >> point.z)) {
      continue;
    }
    if (!(stream >> point.intensity)) {
      point.intensity = 1.0F;
    }
    points->push_back(point);
  }

  return points->empty() ? common::Status::Unavailable("global map pcd is empty")
                         : common::Status::Ok();
}

}  // namespace

common::Status MapLoader::Load(const std::string& config_dir,
                               const config::LocalizationConfig& config,
                               StaticMap* map) const {
  if (map == nullptr) {
    return common::Status::InvalidArgument("static map output is null");
  }

  *map = {};
  const auto meta_path = ResolveAssetPath(config_dir, config.map_meta_path);
  const auto occupancy_path = ResolveAssetPath(config_dir, config.occupancy_path);
  const auto global_map_path = ResolveAssetPath(config_dir, config.global_map_pcd_path);

  map->occupancy_path = occupancy_path.lexically_normal().string();
  map->global_map_path = global_map_path.lexically_normal().string();

  auto status = LoadMapMeta(meta_path, &map->occupancy);
  if (!status.ok()) {
    return status;
  }

  if (std::filesystem::exists(occupancy_path)) {
    status = LoadOccupancyBin(occupancy_path, &map->occupancy);
    if (!status.ok()) {
      return status;
    }
    map->occupancy_loaded = true;
  }

  status = LoadPointCloudPcd(global_map_path, &map->global_points);
  if (!status.ok()) {
    return status;
  }
  map->global_map_loaded = true;
  map->occupancy.stamp = common::Now();
  return common::Status::Ok();
}

}  // namespace rm_nav::localization
