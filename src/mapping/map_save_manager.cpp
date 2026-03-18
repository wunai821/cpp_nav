#include "rm_nav/mapping/map_save_manager.hpp"

#include <filesystem>

namespace rm_nav::mapping {
namespace {

common::Status RemoveTreeIfExists(const std::filesystem::path& path) {
  std::error_code error;
  std::filesystem::remove_all(path, error);
  if (error) {
    return common::Status::InternalError("failed to remove existing directory");
  }
  return common::Status::Ok();
}

common::Status RenameTree(const std::filesystem::path& source,
                          const std::filesystem::path& destination) {
  std::error_code error;
  std::filesystem::create_directories(destination.parent_path(), error);
  if (error) {
    return common::Status::InternalError("failed to create destination parent");
  }
  std::filesystem::rename(source, destination, error);
  if (error) {
    return common::Status::InternalError("failed to rename map directory");
  }
  return common::Status::Ok();
}

MapArtifactPaths BuildArtifactPaths(const std::filesystem::path& output_dir) {
  MapArtifactPaths artifacts;
  artifacts.global_map_pcd_path = (output_dir / "global_map.pcd").lexically_normal().string();
  artifacts.occupancy_bin_path = (output_dir / "occupancy.bin").lexically_normal().string();
  artifacts.occupancy_png_path = (output_dir / "occupancy.png").lexically_normal().string();
  artifacts.map_meta_path = (output_dir / "map_meta.json").lexically_normal().string();
  artifacts.validation_report_path =
      (output_dir / "map_validation_report.json").lexically_normal().string();
  return artifacts;
}

}  // namespace

common::Status MapSaveManager::SaveAndActivate(const MapStorageLayout& layout,
                                               const config::MappingConfig& config,
                                               const std::vector<data::PointXYZI>& global_points,
                                               const data::GridMap2D& occupancy,
                                               const std::vector<MappingTrajectorySample>& trajectory_samples,
                                               MapArtifactPaths* artifact_paths,
                                               MapValidationReport* validation_report,
                                               MapSaveFailureKind* failure_kind) const {
  if (layout.active_dir.empty() || layout.staging_dir.empty() || layout.last_good_dir.empty() ||
      layout.failed_dir.empty()) {
    return common::Status::InvalidArgument("map storage layout is incomplete");
  }
  if (failure_kind != nullptr) {
    *failure_kind = MapSaveFailureKind::kNone;
  }

  auto status = RemoveTreeIfExists(layout.staging_dir);
  if (!status.ok()) {
    if (failure_kind != nullptr) {
      *failure_kind = MapSaveFailureKind::kStorageSwitchFailed;
    }
    return status;
  }

  MapArtifactPaths staging_artifacts;
  status = serializer_.Save(layout.staging_dir, global_points, occupancy, &staging_artifacts);
  if (!status.ok()) {
    if (failure_kind != nullptr) {
      *failure_kind = MapSaveFailureKind::kWriteFailed;
    }
    return status;
  }

  MapValidationReport report;
  status = validator_.Validate(config, global_points, occupancy, trajectory_samples, &report);
  const auto report_write_status =
      validator_.WriteReport(layout.staging_dir / "map_validation_report.json", report);
  if (!report_write_status.ok()) {
    return report_write_status;
  }
  if (validation_report != nullptr) {
    *validation_report = report;
  }

  if (!status.ok()) {
    if (failure_kind != nullptr) {
      *failure_kind = MapSaveFailureKind::kValidationFailed;
    }
    auto cleanup_status = RemoveTreeIfExists(layout.failed_dir);
    if (!cleanup_status.ok()) {
      return cleanup_status;
    }
    cleanup_status = RenameTree(layout.staging_dir, layout.failed_dir);
    if (!cleanup_status.ok()) {
      return cleanup_status;
    }
    return status;
  }

  if (std::filesystem::exists(layout.active_dir)) {
    status = RemoveTreeIfExists(layout.last_good_dir);
    if (!status.ok()) {
      if (failure_kind != nullptr) {
        *failure_kind = MapSaveFailureKind::kStorageSwitchFailed;
      }
      return status;
    }
    status = RenameTree(layout.active_dir, layout.last_good_dir);
    if (!status.ok()) {
      if (failure_kind != nullptr) {
        *failure_kind = MapSaveFailureKind::kStorageSwitchFailed;
      }
      return status;
    }
  }

  status = RenameTree(layout.staging_dir, layout.active_dir);
  if (!status.ok()) {
    if (failure_kind != nullptr) {
      *failure_kind = MapSaveFailureKind::kStorageSwitchFailed;
    }
    return status;
  }

  if (artifact_paths != nullptr) {
    *artifact_paths = BuildArtifactPaths(layout.active_dir);
  }
  return common::Status::Ok();
}

}  // namespace rm_nav::mapping
