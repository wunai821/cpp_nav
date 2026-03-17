#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/imu_packet.hpp"
#include "rm_nav/data/lidar_frame.hpp"
#include "rm_nav/data/preint_block.hpp"
#include "rm_nav/sync/deskew.hpp"
#include "rm_nav/sync/imu_preintegrator.hpp"
#include "rm_nav/utils/file_io.hpp"
#include "rm_nav/utils/logger.hpp"

namespace {

struct Options {
  std::string record_path{};
  std::string imu_record_path{};
  std::string output_dir{"logs/deskew_view"};
  std::size_t frame_order{0};
  std::size_t sample_step{1};
  rm_nav::common::TimeNs imu_margin_ns{1000000};
  bool synthetic_demo{false};
};

struct LoadedData {
  std::vector<rm_nav::data::LidarFrame> lidar_frames{};
  std::vector<rm_nav::data::ImuPacket> imu_packets{};
};

struct ProcessedFrame {
  rm_nav::data::LidarFrame raw{};
  rm_nav::data::LidarFrame deskewed{};
  rm_nav::data::PreintegratedImuBlock preint{};
  std::size_t imu_count{0};
};

rm_nav::common::Status ParseArgs(int argc, char** argv, Options* options) {
  if (options == nullptr) {
    return rm_nav::common::Status::InvalidArgument("options output is null");
  }
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--record" && index + 1 < argc) {
      options->record_path = argv[++index];
      continue;
    }
    if (arg == "--imu-record" && index + 1 < argc) {
      options->imu_record_path = argv[++index];
      continue;
    }
    if (arg == "--output-dir" && index + 1 < argc) {
      options->output_dir = argv[++index];
      continue;
    }
    if (arg == "--frame-order" && index + 1 < argc) {
      options->frame_order =
          static_cast<std::size_t>(std::strtoull(argv[++index], nullptr, 10));
      continue;
    }
    if (arg == "--sample-step" && index + 1 < argc) {
      options->sample_step =
          static_cast<std::size_t>(std::strtoull(argv[++index], nullptr, 10));
      continue;
    }
    if (arg == "--imu-margin-ns" && index + 1 < argc) {
      options->imu_margin_ns = static_cast<rm_nav::common::TimeNs>(
          std::strtoll(argv[++index], nullptr, 10));
      continue;
    }
    if (arg == "--synthetic-demo") {
      options->synthetic_demo = true;
      continue;
    }
    return rm_nav::common::Status::InvalidArgument(
        "usage: deskew_viewer (--record <file> [--imu-record <file>] | --synthetic-demo) "
        "[--frame-order N] [--sample-step N] [--imu-margin-ns NS] [--output-dir dir]");
  }

  if (options->sample_step == 0U) {
    return rm_nav::common::Status::InvalidArgument("sample step must be positive");
  }
  if (options->imu_margin_ns < 0) {
    return rm_nav::common::Status::InvalidArgument("imu margin must be non-negative");
  }
  if (options->synthetic_demo && !options->record_path.empty()) {
    return rm_nav::common::Status::InvalidArgument(
        "synthetic demo cannot be combined with --record");
  }
  if (!options->synthetic_demo && options->record_path.empty()) {
    return rm_nav::common::Status::InvalidArgument(
        "deskew_viewer requires --record <file> or --synthetic-demo");
  }
  return rm_nav::common::Status::Ok();
}

rm_nav::common::Status LoadRecord(const std::string& path, bool load_lidar, bool load_imu,
                                  LoadedData* data) {
  if (data == nullptr) {
    return rm_nav::common::Status::InvalidArgument("loaded data output is null");
  }

  rm_nav::utils::RecorderReader reader;
  auto status = reader.Open(path);
  if (!status.ok()) {
    return status;
  }

  while (true) {
    auto message = reader.ReadNext();
    if (!message.has_value()) {
      break;
    }
    if (load_lidar && message->channel == rm_nav::utils::RecordChannel::kLidarFrame) {
      rm_nav::data::LidarFrame frame;
      status = rm_nav::utils::DecodeLidarFrame(*message, &frame);
      if (!status.ok()) {
        return status;
      }
      data->lidar_frames.push_back(std::move(frame));
      continue;
    }
    if (load_imu && message->channel == rm_nav::utils::RecordChannel::kImuPacket) {
      rm_nav::data::ImuPacket packet;
      status = rm_nav::utils::DecodeImuPacket(*message, &packet);
      if (!status.ok()) {
        return status;
      }
      data->imu_packets.push_back(packet);
    }
  }
  return rm_nav::common::Status::Ok();
}

rm_nav::data::PointXYZI RotateYaw(const rm_nav::data::PointXYZI& point, float yaw_rad) {
  const float cos_yaw = std::cos(yaw_rad);
  const float sin_yaw = std::sin(yaw_rad);
  rm_nav::data::PointXYZI rotated = point;
  rotated.x = cos_yaw * point.x - sin_yaw * point.y;
  rotated.y = sin_yaw * point.x + cos_yaw * point.y;
  return rotated;
}

void BuildSyntheticDemo(LoadedData* data) {
  data->lidar_frames.clear();
  data->imu_packets.clear();

  rm_nav::data::LidarFrame frame;
  frame.scan_begin_stamp = rm_nav::common::FromNanoseconds(0);
  frame.scan_end_stamp = rm_nav::common::FromNanoseconds(100000000);
  frame.stamp = frame.scan_end_stamp;
  frame.frame_index = 0;
  constexpr std::size_t kPointCount = 240;
  constexpr float kTotalYawRad = 0.75F;
  constexpr float kPi = 3.14159265358979323846F;
  const float duration_s = 0.1F;
  frame.points.reserve(kPointCount);
  for (std::size_t index = 0; index < kPointCount; ++index) {
    const float alpha =
        static_cast<float>(index) / static_cast<float>(kPointCount - 1U);
    rm_nav::data::PointXYZI ideal;
    ideal.x = 6.0F + 0.15F * std::sin(alpha * 8.0F * kPi);
    ideal.y = -3.0F + 6.0F * alpha;
    ideal.z = 0.2F * std::sin(alpha * 3.0F * kPi);
    ideal.intensity = 255.0F * alpha;
    ideal.relative_time_s = alpha * duration_s;
    auto distorted = RotateYaw(ideal, kTotalYawRad * alpha);
    distorted.intensity = ideal.intensity;
    distorted.relative_time_s = ideal.relative_time_s;
    frame.points.push_back(distorted);
  }
  data->lidar_frames.push_back(frame);

  constexpr std::size_t kImuCount = 21;
  const float yaw_rate = kTotalYawRad / duration_s;
  for (std::size_t index = 0; index < kImuCount; ++index) {
    rm_nav::data::ImuPacket packet;
    packet.stamp = rm_nav::common::FromNanoseconds(
        static_cast<rm_nav::common::TimeNs>(index) * 5000000);
    packet.sample_index = static_cast<std::uint32_t>(index);
    packet.angular_velocity.z = yaw_rate;
    packet.is_valid = true;
    data->imu_packets.push_back(packet);
  }
}

void SortLoadedData(LoadedData* data) {
  std::sort(data->lidar_frames.begin(), data->lidar_frames.end(),
            [](const rm_nav::data::LidarFrame& lhs,
               const rm_nav::data::LidarFrame& rhs) { return lhs.stamp < rhs.stamp; });
  std::sort(data->imu_packets.begin(), data->imu_packets.end(),
            [](const rm_nav::data::ImuPacket& lhs,
               const rm_nav::data::ImuPacket& rhs) { return lhs.stamp < rhs.stamp; });
}

rm_nav::common::Status SelectFrame(const LoadedData& data, std::size_t frame_order,
                                   rm_nav::data::LidarFrame* frame) {
  if (frame == nullptr) {
    return rm_nav::common::Status::InvalidArgument("frame output is null");
  }
  if (frame_order >= data.lidar_frames.size()) {
    return rm_nav::common::Status::InvalidArgument("requested frame order is out of range");
  }
  *frame = data.lidar_frames[frame_order];
  return rm_nav::common::Status::Ok();
}

std::vector<rm_nav::data::ImuPacket> SliceImuPackets(
    const std::vector<rm_nav::data::ImuPacket>& packets, rm_nav::common::TimePoint begin_stamp,
    rm_nav::common::TimePoint end_stamp, rm_nav::common::TimeNs margin_ns) {
  const auto slice_begin = begin_stamp - std::chrono::nanoseconds(margin_ns);
  const auto slice_end = end_stamp + std::chrono::nanoseconds(margin_ns);
  std::vector<rm_nav::data::ImuPacket> slice;
  for (const auto& packet : packets) {
    if (packet.stamp < slice_begin || packet.stamp > slice_end) {
      continue;
    }
    slice.push_back(packet);
  }
  return slice;
}

rm_nav::common::Status ProcessFrame(const rm_nav::data::LidarFrame& frame,
                                    const std::vector<rm_nav::data::ImuPacket>& imu_packets,
                                    rm_nav::common::TimeNs imu_margin_ns,
                                    ProcessedFrame* processed) {
  if (processed == nullptr) {
    return rm_nav::common::Status::InvalidArgument("processed frame output is null");
  }

  processed->raw = frame;
  processed->deskewed = frame;
  processed->preint = {};

  const auto imu_slice =
      SliceImuPackets(imu_packets, frame.scan_begin_stamp, frame.scan_end_stamp, imu_margin_ns);
  processed->imu_count = imu_slice.size();

  rm_nav::sync::ImuPreintegrator preintegrator;
  auto status = preintegrator.Integrate(imu_slice.data(), imu_slice.size(),
                                        frame.scan_begin_stamp, frame.scan_end_stamp,
                                        &processed->preint);
  if (!status.ok() && status.code != rm_nav::common::StatusCode::kNotReady) {
    return status;
  }
  if (!status.ok()) {
    rm_nav::utils::LogWarn("deskew_viewer",
                           "imu preintegration is not ready, exporting raw cloud as fallback");
  }

  rm_nav::sync::Deskew deskew;
  status = deskew.Apply(frame, processed->preint, &processed->deskewed);
  if (!status.ok()) {
    return status;
  }
  return rm_nav::common::Status::Ok();
}

rm_nav::common::Status WritePly(const std::filesystem::path& path,
                                const rm_nav::data::LidarFrame& frame) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return rm_nav::common::Status::Unavailable("failed to open point cloud export");
  }

  output << "ply\n";
  output << "format ascii 1.0\n";
  output << "comment rm_nav deskew viewer export\n";
  output << "element vertex " << frame.points.size() << "\n";
  output << "property float x\n";
  output << "property float y\n";
  output << "property float z\n";
  output << "property float intensity\n";
  output << "property float relative_time_s\n";
  output << "end_header\n";
  output << std::fixed << std::setprecision(6);
  for (const auto& point : frame.points) {
    output << point.x << ' ' << point.y << ' ' << point.z << ' ' << point.intensity << ' '
           << point.relative_time_s << "\n";
  }
  return output.good() ? rm_nav::common::Status::Ok()
                       : rm_nav::common::Status::InternalError(
                             "failed to write point cloud export");
}

std::string BuildPointArrayScript(const std::vector<rm_nav::data::PointXYZI>& points,
                                  std::size_t sample_step) {
  std::ostringstream stream;
  stream << '[';
  bool first = true;
  for (std::size_t index = 0; index < points.size(); index += sample_step) {
    const auto& point = points[index];
    if (!first) {
      stream << ',';
    }
    first = false;
    stream << '[' << std::fixed << std::setprecision(4) << point.x << ',' << point.y << ','
           << point.z << ',' << point.relative_time_s << ']';
  }
  stream << ']';
  return stream.str();
}

rm_nav::common::Status WriteSummary(const std::filesystem::path& path,
                                    const ProcessedFrame& processed) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return rm_nav::common::Status::Unavailable("failed to open summary export");
  }

  const double duration_ms =
      static_cast<double>(rm_nav::common::ToNanoseconds(processed.raw.scan_end_stamp -
                                                        processed.raw.scan_begin_stamp)) /
      1e6;
  output << std::fixed << std::setprecision(6);
  output << "frame_index=" << processed.raw.frame_index << "\n";
  output << "points=" << processed.raw.points.size() << "\n";
  output << "imu_packets=" << processed.imu_count << "\n";
  output << "scan_duration_ms=" << duration_ms << "\n";
  output << "deskewed=" << (processed.deskewed.is_deskewed ? "true" : "false") << "\n";
  output << "delta_rpy_xyz=" << processed.preint.delta_rpy.x << ","
         << processed.preint.delta_rpy.y << "," << processed.preint.delta_rpy.z << "\n";
  output << "delta_velocity_xyz=" << processed.preint.delta_velocity.x << ","
         << processed.preint.delta_velocity.y << "," << processed.preint.delta_velocity.z
         << "\n";
  output << "delta_position_xyz=" << processed.preint.delta_position.x << ","
         << processed.preint.delta_position.y << "," << processed.preint.delta_position.z
         << "\n";
  return output.good() ? rm_nav::common::Status::Ok()
                       : rm_nav::common::Status::InternalError("failed to write summary export");
}

rm_nav::common::Status WriteHtml(const std::filesystem::path& path,
                                 const ProcessedFrame& processed,
                                 std::size_t sample_step) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return rm_nav::common::Status::Unavailable("failed to open html export");
  }

  const std::string raw_points = BuildPointArrayScript(processed.raw.points, sample_step);
  const std::string deskewed_points =
      BuildPointArrayScript(processed.deskewed.points, sample_step);
  const double scan_duration_ms =
      static_cast<double>(rm_nav::common::ToNanoseconds(processed.raw.scan_end_stamp -
                                                        processed.raw.scan_begin_stamp)) /
      1e6;
  const double yaw_deg = static_cast<double>(processed.preint.delta_rpy.z) * 180.0 /
                         3.14159265358979323846;

  output << "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n";
  output << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  output << "<title>Deskew Viewer</title>\n";
  output << "<style>\n";
  output << "body{margin:0;font-family:Georgia,serif;background:#f5efe6;color:#1f1f1f;}\n";
  output << ".wrap{max-width:1400px;margin:0 auto;padding:24px;}\n";
  output << ".hero{display:grid;gap:8px;margin-bottom:20px;}\n";
  output << ".hero h1{margin:0;font-size:32px;}\n";
  output << ".hero p{margin:0;color:#5e5548;}\n";
  output << ".stats{display:flex;flex-wrap:wrap;gap:10px;margin-top:10px;}\n";
  output << ".chip{background:#fffaf2;border:1px solid #d5c6af;border-radius:999px;";
  output << "padding:8px 12px;font-size:14px;}\n";
  output << ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));";
  output << "gap:18px;}\n";
  output << ".panel{background:#fffdf8;border:1px solid #d9cbb7;border-radius:18px;";
  output << "padding:16px;box-shadow:0 18px 48px rgba(74,53,28,0.08);}\n";
  output << ".panel h2{margin:0 0 12px 0;font-size:22px;}\n";
  output << "canvas{width:100%;aspect-ratio:1/1;background:#f7f4ee;border-radius:14px;";
  output << "border:1px solid #e2d6c4;display:block;}\n";
  output << ".foot{margin-top:14px;font-size:13px;color:#6a5f52;}\n";
  output << "</style>\n</head>\n<body>\n<div class=\"wrap\">\n";
  output << "<section class=\"hero\">\n";
  output << "<h1>Deskew Before / After</h1>\n";
  output << "<p>Top-down XY preview using a shared scale so shape changes are easy to spot.</p>\n";
  output << "<div class=\"stats\">\n";
  output << "<span class=\"chip\">frame_order=" << processed.raw.frame_index << "</span>\n";
  output << "<span class=\"chip\">points=" << processed.raw.points.size() << "</span>\n";
  output << "<span class=\"chip\">imu_packets=" << processed.imu_count << "</span>\n";
  output << "<span class=\"chip\">scan_ms=" << std::fixed << std::setprecision(2)
         << scan_duration_ms << "</span>\n";
  output << "<span class=\"chip\">delta_yaw_deg=" << std::fixed << std::setprecision(2)
         << yaw_deg << "</span>\n";
  output << "<span class=\"chip\">deskewed="
         << (processed.deskewed.is_deskewed ? "true" : "false") << "</span>\n";
  output << "</div>\n</section>\n";
  output << "<section class=\"grid\">\n";
  output << "<div class=\"panel\"><h2>Raw Cloud</h2><canvas id=\"raw\"></canvas></div>\n";
  output << "<div class=\"panel\"><h2>Deskewed Cloud</h2><canvas id=\"deskewed\"></canvas></div>\n";
  output << "</section>\n";
  output << "<div class=\"foot\">Color follows point relative time within the scan: early points ";
  output << "are cool, later points are warm.</div>\n";
  output << "</div>\n<script>\n";
  output << "const rawPoints=" << raw_points << ";\n";
  output << "const deskewedPoints=" << deskewed_points << ";\n";
  output << "const scanDurationS="
         << std::fixed << std::setprecision(6) << (scan_duration_ms / 1000.0) << ";\n";
  output << "function bounds(arrays){let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;";
  output << "for(const pts of arrays){for(const p of pts){minX=Math.min(minX,p[0]);";
  output << "maxX=Math.max(maxX,p[0]);minY=Math.min(minY,p[1]);maxY=Math.max(maxY,p[1]);}}";
  output << "if(!isFinite(minX)){minX=-1;maxX=1;minY=-1;maxY=1;}return {minX,maxX,minY,maxY};}\n";
  output << "function color(alpha){const t=Math.max(0,Math.min(1,alpha));";
  output << "const r=Math.round(34+t*205);const g=Math.round(112+t*70);";
  output << "const b=Math.round(179-t*120);return `rgba(${r},${g},${b},0.85)`;}\n";
  output << "function draw(id,points,box){const canvas=document.getElementById(id);";
  output << "const ctx=canvas.getContext('2d');const size=canvas.clientWidth||560;";
  output << "canvas.width=size;canvas.height=size;ctx.clearRect(0,0,size,size);";
  output << "ctx.fillStyle='#f7f4ee';ctx.fillRect(0,0,size,size);const pad=32;";
  output << "const spanX=Math.max(1e-3,box.maxX-box.minX);const spanY=Math.max(1e-3,box.maxY-box.minY);";
  output << "const scale=Math.min((size-pad*2)/spanX,(size-pad*2)/spanY);";
  output << "ctx.strokeStyle='#d8ccbb';ctx.lineWidth=1;ctx.strokeRect(pad,pad,size-pad*2,size-pad*2);";
  output << "for(const p of points){const x=pad+(p[0]-box.minX)*scale;";
  output << "const y=size-(pad+(p[1]-box.minY)*scale);";
  output << "ctx.fillStyle=color(p[3]/Math.max(scanDurationS,1e-6));";
  output << "ctx.beginPath();ctx.arc(x,y,2.2,0,Math.PI*2);ctx.fill();}}\n";
  output << "const box=bounds([rawPoints,deskewedPoints]);draw('raw',rawPoints,box);";
  output << "draw('deskewed',deskewedPoints,box);\n";
  output << "window.addEventListener('resize',()=>{draw('raw',rawPoints,box);";
  output << "draw('deskewed',deskewedPoints,box);});\n";
  output << "</script>\n</body>\n</html>\n";
  return output.good() ? rm_nav::common::Status::Ok()
                       : rm_nav::common::Status::InternalError("failed to write html export");
}

}  // namespace

int main(int argc, char** argv) {
  rm_nav::utils::Logger::Instance().Initialize({rm_nav::utils::LogLevel::kInfo});

  Options options;
  auto status = ParseArgs(argc, argv, &options);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }

  LoadedData data;
  if (options.synthetic_demo) {
    BuildSyntheticDemo(&data);
  } else {
    status = LoadRecord(options.record_path, true, options.imu_record_path.empty(), &data);
    if (!status.ok()) {
      rm_nav::utils::LogError("deskew_viewer", status.message);
      return 1;
    }
    if (!options.imu_record_path.empty()) {
      status = LoadRecord(options.imu_record_path, false, true, &data);
      if (!status.ok()) {
        rm_nav::utils::LogError("deskew_viewer", status.message);
        return 1;
      }
    }
  }

  SortLoadedData(&data);
  if (data.lidar_frames.empty()) {
    rm_nav::utils::LogError("deskew_viewer", "no lidar frames found");
    return 1;
  }

  rm_nav::data::LidarFrame selected_frame;
  status = SelectFrame(data, options.frame_order, &selected_frame);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }

  ProcessedFrame processed;
  status = ProcessFrame(selected_frame, data.imu_packets, options.imu_margin_ns, &processed);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }

  const std::filesystem::path output_dir(options.output_dir);
  std::filesystem::create_directories(output_dir);

  const std::filesystem::path raw_path = output_dir / "raw_frame.ply";
  const std::filesystem::path deskewed_path = output_dir / "deskewed_frame.ply";
  const std::filesystem::path summary_path = output_dir / "summary.txt";
  const std::filesystem::path html_path = output_dir / "compare.html";

  status = WritePly(raw_path, processed.raw);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }
  status = WritePly(deskewed_path, processed.deskewed);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }
  status = WriteSummary(summary_path, processed);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }
  status = WriteHtml(html_path, processed, options.sample_step);
  if (!status.ok()) {
    rm_nav::utils::LogError("deskew_viewer", status.message);
    return 1;
  }

  rm_nav::utils::LogInfo(
      "deskew_viewer",
      "exported raw=" + raw_path.string() + " deskewed=" + deskewed_path.string() +
          " html=" + html_path.string() + " imu_packets=" +
          std::to_string(processed.imu_count) + " deskewed=" +
          (processed.deskewed.is_deskewed ? std::string("true") : std::string("false")));
  return 0;
}
