#pragma once

#include <cstdint>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "rm_nav/common/time.hpp"
#include "rm_nav/data/point_types.hpp"
#include "rm_nav/tf/frame_ids.hpp"

namespace rm_nav::data {

class SharedPointCloud {
 public:
  using Storage = std::vector<PointXYZI>;
  using value_type = Storage::value_type;
  using size_type = Storage::size_type;
  using iterator = Storage::iterator;
  using const_iterator = Storage::const_iterator;

  SharedPointCloud() : storage_(std::make_shared<Storage>()) {}

  bool empty() const { return storage_->empty(); }
  size_type size() const { return storage_->size(); }

  void clear() {
    EnsureUnique();
    storage_->clear();
  }

  void reserve(size_type count) {
    EnsureUnique();
    storage_->reserve(count);
  }

  void resize(size_type count) {
    EnsureUnique();
    storage_->resize(count);
  }

  void push_back(const value_type& point) {
    EnsureUnique();
    storage_->push_back(point);
  }

  void push_back(value_type&& point) {
    EnsureUnique();
    storage_->push_back(std::move(point));
  }

  value_type& operator[](size_type index) {
    EnsureUnique();
    return (*storage_)[index];
  }

  const value_type& operator[](size_type index) const { return (*storage_)[index]; }

  value_type& front() {
    EnsureUnique();
    return storage_->front();
  }

  const value_type& front() const { return storage_->front(); }

  value_type& back() {
    EnsureUnique();
    return storage_->back();
  }

  const value_type& back() const { return storage_->back(); }

  iterator begin() {
    EnsureUnique();
    return storage_->begin();
  }

  iterator end() {
    EnsureUnique();
    return storage_->end();
  }

  const_iterator begin() const { return storage_->begin(); }
  const_iterator end() const { return storage_->end(); }
  const_iterator cbegin() const { return storage_->cbegin(); }
  const_iterator cend() const { return storage_->cend(); }

  operator const Storage&() const { return *storage_; }

  Storage& MutableView() {
    EnsureUnique();
    return *storage_;
  }

  const Storage& view() const { return *storage_; }

  std::size_t use_count() const { return storage_.use_count(); }

 private:
  void EnsureUnique() {
    if (!storage_) {
      storage_ = std::make_shared<Storage>();
      return;
    }
    if (!storage_.unique()) {
      storage_ = std::make_shared<Storage>(*storage_);
    }
  }

  std::shared_ptr<Storage> storage_{};
};

struct LidarFrame {
  common::TimePoint stamp{};
  common::TimePoint scan_begin_stamp{};
  common::TimePoint scan_end_stamp{};
  std::string_view frame_id{tf::kLaserFrame};
  std::uint32_t frame_index{0};
  SharedPointCloud points{};
  bool is_deskewed{false};
};

}  // namespace rm_nav::data
