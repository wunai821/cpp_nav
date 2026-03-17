#include <cassert>
#include <iostream>

#include "rm_nav/config/localization_config.hpp"
#include "rm_nav/localization/map_loader.hpp"

int main() {
  rm_nav::config::LocalizationConfig config;
  rm_nav::localization::MapLoader loader;
  rm_nav::localization::StaticMap map;

  const auto status = loader.Load("config", config, &map);
  assert(status.ok());
  assert(map.global_map_loaded);
  assert(map.occupancy_loaded);
  assert(map.occupancy.width == 240U);
  assert(map.occupancy.height == 120U);
  assert(map.global_points.size() == 95U);

  std::cout << "test_map_loader passed\n";
  return 0;
}
