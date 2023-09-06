// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: planner/local_path_info.proto

#ifndef PROTOBUF_planner_2flocal_5fpath_5finfo_2eproto__INCLUDED
#define PROTOBUF_planner_2flocal_5fpath_5finfo_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "base/header.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_planner_2flocal_5fpath_5finfo_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[3];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsPathPointImpl();
void InitDefaultsPathPoint();
void InitDefaultsTrajectoryPointImpl();
void InitDefaultsTrajectoryPoint();
void InitDefaultsLocalPathInfoImpl();
void InitDefaultsLocalPathInfo();
inline void InitDefaults() {
  InitDefaultsPathPoint();
  InitDefaultsTrajectoryPoint();
  InitDefaultsLocalPathInfo();
}
}  // namespace protobuf_planner_2flocal_5fpath_5finfo_2eproto
namespace xsproto {
namespace planner {
class LocalPathInfo;
class LocalPathInfoDefaultTypeInternal;
extern LocalPathInfoDefaultTypeInternal _LocalPathInfo_default_instance_;
class PathPoint;
class PathPointDefaultTypeInternal;
extern PathPointDefaultTypeInternal _PathPoint_default_instance_;
class TrajectoryPoint;
class TrajectoryPointDefaultTypeInternal;
extern TrajectoryPointDefaultTypeInternal _TrajectoryPoint_default_instance_;
}  // namespace planner
}  // namespace xsproto
namespace xsproto {
namespace planner {

enum VehicleCommand {
  NONE = 0,
  ES = 200,
  ST = 201,
  AD_SPEED = 202,
  AD_DISTANCE = 203,
  AD_POINT = 204,
  BK_SPEED = 205,
  BK_POINT = 206,
  IG = 207,
  FO = 208,
  RT_CTL = 209,
  AD_CIRCLE = 210,
  AD_HEADING = 211,
  BK_HEADING = 212,
  MEC_POINT = 213,
  MEC_DEGREE = 214,
  VehicleCommand_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  VehicleCommand_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool VehicleCommand_IsValid(int value);
const VehicleCommand VehicleCommand_MIN = NONE;
const VehicleCommand VehicleCommand_MAX = MEC_DEGREE;
const int VehicleCommand_ARRAYSIZE = VehicleCommand_MAX + 1;

const ::google::protobuf::EnumDescriptor* VehicleCommand_descriptor();
inline const ::std::string& VehicleCommand_Name(VehicleCommand value) {
  return ::google::protobuf::internal::NameOfEnum(
    VehicleCommand_descriptor(), value);
}
inline bool VehicleCommand_Parse(
    const ::std::string& name, VehicleCommand* value) {
  return ::google::protobuf::internal::ParseNamedEnum<VehicleCommand>(
    VehicleCommand_descriptor(), name, value);
}
enum TurningState {
  NO_TURN = 0,
  LEFT_TURN = 1,
  FORWARD_TURN = 2,
  RIGHT_TURN = 3,
  U_TURN = 4,
  TurningState_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  TurningState_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool TurningState_IsValid(int value);
const TurningState TurningState_MIN = NO_TURN;
const TurningState TurningState_MAX = U_TURN;
const int TurningState_ARRAYSIZE = TurningState_MAX + 1;

const ::google::protobuf::EnumDescriptor* TurningState_descriptor();
inline const ::std::string& TurningState_Name(TurningState value) {
  return ::google::protobuf::internal::NameOfEnum(
    TurningState_descriptor(), value);
}
inline bool TurningState_Parse(
    const ::std::string& name, TurningState* value) {
  return ::google::protobuf::internal::ParseNamedEnum<TurningState>(
    TurningState_descriptor(), name, value);
}
// ===================================================================

class PathPoint : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.planner.PathPoint) */ {
 public:
  PathPoint();
  virtual ~PathPoint();

  PathPoint(const PathPoint& from);

  inline PathPoint& operator=(const PathPoint& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PathPoint(PathPoint&& from) noexcept
    : PathPoint() {
    *this = ::std::move(from);
  }

  inline PathPoint& operator=(PathPoint&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const PathPoint& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PathPoint* internal_default_instance() {
    return reinterpret_cast<const PathPoint*>(
               &_PathPoint_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(PathPoint* other);
  friend void swap(PathPoint& a, PathPoint& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PathPoint* New() const PROTOBUF_FINAL { return New(NULL); }

  PathPoint* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const PathPoint& from);
  void MergeFrom(const PathPoint& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(PathPoint* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // double x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // double y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // double l_X = 3;
  void clear_l_x();
  static const int kLXFieldNumber = 3;
  double l_x() const;
  void set_l_x(double value);

  // double l_y = 4;
  void clear_l_y();
  static const int kLYFieldNumber = 4;
  double l_y() const;
  void set_l_y(double value);

  // double r_x = 5;
  void clear_r_x();
  static const int kRXFieldNumber = 5;
  double r_x() const;
  void set_r_x(double value);

  // double r_y = 6;
  void clear_r_y();
  static const int kRYFieldNumber = 6;
  double r_y() const;
  void set_r_y(double value);

  // double direction = 7;
  void clear_direction();
  static const int kDirectionFieldNumber = 7;
  double direction() const;
  void set_direction(double value);

  // double curvature = 8;
  void clear_curvature();
  static const int kCurvatureFieldNumber = 8;
  double curvature() const;
  void set_curvature(double value);

  // double dcurvature = 9;
  void clear_dcurvature();
  static const int kDcurvatureFieldNumber = 9;
  double dcurvature() const;
  void set_dcurvature(double value);

  // double ddcurvature = 10;
  void clear_ddcurvature();
  static const int kDdcurvatureFieldNumber = 10;
  double ddcurvature() const;
  void set_ddcurvature(double value);

  // @@protoc_insertion_point(class_scope:xsproto.planner.PathPoint)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double x_;
  double y_;
  double l_x_;
  double l_y_;
  double r_x_;
  double r_y_;
  double direction_;
  double curvature_;
  double dcurvature_;
  double ddcurvature_;
  mutable int _cached_size_;
  friend struct ::protobuf_planner_2flocal_5fpath_5finfo_2eproto::TableStruct;
  friend void ::protobuf_planner_2flocal_5fpath_5finfo_2eproto::InitDefaultsPathPointImpl();
};
// -------------------------------------------------------------------

class TrajectoryPoint : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.planner.TrajectoryPoint) */ {
 public:
  TrajectoryPoint();
  virtual ~TrajectoryPoint();

  TrajectoryPoint(const TrajectoryPoint& from);

  inline TrajectoryPoint& operator=(const TrajectoryPoint& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  TrajectoryPoint(TrajectoryPoint&& from) noexcept
    : TrajectoryPoint() {
    *this = ::std::move(from);
  }

  inline TrajectoryPoint& operator=(TrajectoryPoint&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const TrajectoryPoint& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrajectoryPoint* internal_default_instance() {
    return reinterpret_cast<const TrajectoryPoint*>(
               &_TrajectoryPoint_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(TrajectoryPoint* other);
  friend void swap(TrajectoryPoint& a, TrajectoryPoint& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline TrajectoryPoint* New() const PROTOBUF_FINAL { return New(NULL); }

  TrajectoryPoint* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const TrajectoryPoint& from);
  void MergeFrom(const TrajectoryPoint& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(TrajectoryPoint* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .xsproto.planner.PathPoint path_point = 1;
  bool has_path_point() const;
  void clear_path_point();
  static const int kPathPointFieldNumber = 1;
  const ::xsproto::planner::PathPoint& path_point() const;
  ::xsproto::planner::PathPoint* release_path_point();
  ::xsproto::planner::PathPoint* mutable_path_point();
  void set_allocated_path_point(::xsproto::planner::PathPoint* path_point);

  // double speed = 2;
  void clear_speed();
  static const int kSpeedFieldNumber = 2;
  double speed() const;
  void set_speed(double value);

  // double a = 3;
  void clear_a();
  static const int kAFieldNumber = 3;
  double a() const;
  void set_a(double value);

  // double da = 4;
  void clear_da();
  static const int kDaFieldNumber = 4;
  double da() const;
  void set_da(double value);

  // double t = 5;
  void clear_t();
  static const int kTFieldNumber = 5;
  double t() const;
  void set_t(double value);

  // @@protoc_insertion_point(class_scope:xsproto.planner.TrajectoryPoint)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::planner::PathPoint* path_point_;
  double speed_;
  double a_;
  double da_;
  double t_;
  mutable int _cached_size_;
  friend struct ::protobuf_planner_2flocal_5fpath_5finfo_2eproto::TableStruct;
  friend void ::protobuf_planner_2flocal_5fpath_5finfo_2eproto::InitDefaultsTrajectoryPointImpl();
};
// -------------------------------------------------------------------

class LocalPathInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.planner.LocalPathInfo) */ {
 public:
  LocalPathInfo();
  virtual ~LocalPathInfo();

  LocalPathInfo(const LocalPathInfo& from);

  inline LocalPathInfo& operator=(const LocalPathInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  LocalPathInfo(LocalPathInfo&& from) noexcept
    : LocalPathInfo() {
    *this = ::std::move(from);
  }

  inline LocalPathInfo& operator=(LocalPathInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const LocalPathInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LocalPathInfo* internal_default_instance() {
    return reinterpret_cast<const LocalPathInfo*>(
               &_LocalPathInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(LocalPathInfo* other);
  friend void swap(LocalPathInfo& a, LocalPathInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline LocalPathInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  LocalPathInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const LocalPathInfo& from);
  void MergeFrom(const LocalPathInfo& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(LocalPathInfo* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .xsproto.planner.TrajectoryPoint path = 2;
  int path_size() const;
  void clear_path();
  static const int kPathFieldNumber = 2;
  const ::xsproto::planner::TrajectoryPoint& path(int index) const;
  ::xsproto::planner::TrajectoryPoint* mutable_path(int index);
  ::xsproto::planner::TrajectoryPoint* add_path();
  ::google::protobuf::RepeatedPtrField< ::xsproto::planner::TrajectoryPoint >*
      mutable_path();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::planner::TrajectoryPoint >&
      path() const;

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // .xsproto.planner.PathPoint dest_point = 14;
  bool has_dest_point() const;
  void clear_dest_point();
  static const int kDestPointFieldNumber = 14;
  const ::xsproto::planner::PathPoint& dest_point() const;
  ::xsproto::planner::PathPoint* release_dest_point();
  ::xsproto::planner::PathPoint* mutable_dest_point();
  void set_allocated_dest_point(::xsproto::planner::PathPoint* dest_point);

  // double speed = 4;
  void clear_speed();
  static const int kSpeedFieldNumber = 4;
  double speed() const;
  void set_speed(double value);

  // double exp_velocity = 5;
  void clear_exp_velocity();
  static const int kExpVelocityFieldNumber = 5;
  double exp_velocity() const;
  void set_exp_velocity(double value);

  // .xsproto.planner.VehicleCommand vehicle_command = 3;
  void clear_vehicle_command();
  static const int kVehicleCommandFieldNumber = 3;
  ::xsproto::planner::VehicleCommand vehicle_command() const;
  void set_vehicle_command(::xsproto::planner::VehicleCommand value);

  // int32 close_sensor = 7;
  void clear_close_sensor();
  static const int kCloseSensorFieldNumber = 7;
  ::google::protobuf::int32 close_sensor() const;
  void set_close_sensor(::google::protobuf::int32 value);

  // double exp_curvature = 6;
  void clear_exp_curvature();
  static const int kExpCurvatureFieldNumber = 6;
  double exp_curvature() const;
  void set_exp_curvature(double value);

  // double lateral_bias = 9;
  void clear_lateral_bias();
  static const int kLateralBiasFieldNumber = 9;
  double lateral_bias() const;
  void set_lateral_bias(double value);

  // .xsproto.planner.TurningState turning_state = 8;
  void clear_turning_state();
  static const int kTurningStateFieldNumber = 8;
  ::xsproto::planner::TurningState turning_state() const;
  void set_turning_state(::xsproto::planner::TurningState value);

  // bool hand_brake = 13;
  void clear_hand_brake();
  static const int kHandBrakeFieldNumber = 13;
  bool hand_brake() const;
  void set_hand_brake(bool value);

  // double acceleration = 11;
  void clear_acceleration();
  static const int kAccelerationFieldNumber = 11;
  double acceleration() const;
  void set_acceleration(double value);

  // double write_sys_time = 12;
  void clear_write_sys_time();
  static const int kWriteSysTimeFieldNumber = 12;
  double write_sys_time() const;
  void set_write_sys_time(double value);

  // @@protoc_insertion_point(class_scope:xsproto.planner.LocalPathInfo)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::planner::TrajectoryPoint > path_;
  ::xsproto::base::Header* header_;
  ::xsproto::planner::PathPoint* dest_point_;
  double speed_;
  double exp_velocity_;
  int vehicle_command_;
  ::google::protobuf::int32 close_sensor_;
  double exp_curvature_;
  double lateral_bias_;
  int turning_state_;
  bool hand_brake_;
  double acceleration_;
  double write_sys_time_;
  mutable int _cached_size_;
  friend struct ::protobuf_planner_2flocal_5fpath_5finfo_2eproto::TableStruct;
  friend void ::protobuf_planner_2flocal_5fpath_5finfo_2eproto::InitDefaultsLocalPathInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PathPoint

// double x = 1;
inline void PathPoint::clear_x() {
  x_ = 0;
}
inline double PathPoint::x() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.x)
  return x_;
}
inline void PathPoint::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.x)
}

// double y = 2;
inline void PathPoint::clear_y() {
  y_ = 0;
}
inline double PathPoint::y() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.y)
  return y_;
}
inline void PathPoint::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.y)
}

// double l_X = 3;
inline void PathPoint::clear_l_x() {
  l_x_ = 0;
}
inline double PathPoint::l_x() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.l_X)
  return l_x_;
}
inline void PathPoint::set_l_x(double value) {
  
  l_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.l_X)
}

// double l_y = 4;
inline void PathPoint::clear_l_y() {
  l_y_ = 0;
}
inline double PathPoint::l_y() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.l_y)
  return l_y_;
}
inline void PathPoint::set_l_y(double value) {
  
  l_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.l_y)
}

// double r_x = 5;
inline void PathPoint::clear_r_x() {
  r_x_ = 0;
}
inline double PathPoint::r_x() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.r_x)
  return r_x_;
}
inline void PathPoint::set_r_x(double value) {
  
  r_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.r_x)
}

// double r_y = 6;
inline void PathPoint::clear_r_y() {
  r_y_ = 0;
}
inline double PathPoint::r_y() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.r_y)
  return r_y_;
}
inline void PathPoint::set_r_y(double value) {
  
  r_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.r_y)
}

// double direction = 7;
inline void PathPoint::clear_direction() {
  direction_ = 0;
}
inline double PathPoint::direction() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.direction)
  return direction_;
}
inline void PathPoint::set_direction(double value) {
  
  direction_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.direction)
}

// double curvature = 8;
inline void PathPoint::clear_curvature() {
  curvature_ = 0;
}
inline double PathPoint::curvature() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.curvature)
  return curvature_;
}
inline void PathPoint::set_curvature(double value) {
  
  curvature_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.curvature)
}

// double dcurvature = 9;
inline void PathPoint::clear_dcurvature() {
  dcurvature_ = 0;
}
inline double PathPoint::dcurvature() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.dcurvature)
  return dcurvature_;
}
inline void PathPoint::set_dcurvature(double value) {
  
  dcurvature_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.dcurvature)
}

// double ddcurvature = 10;
inline void PathPoint::clear_ddcurvature() {
  ddcurvature_ = 0;
}
inline double PathPoint::ddcurvature() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.PathPoint.ddcurvature)
  return ddcurvature_;
}
inline void PathPoint::set_ddcurvature(double value) {
  
  ddcurvature_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.PathPoint.ddcurvature)
}

// -------------------------------------------------------------------

// TrajectoryPoint

// .xsproto.planner.PathPoint path_point = 1;
inline bool TrajectoryPoint::has_path_point() const {
  return this != internal_default_instance() && path_point_ != NULL;
}
inline void TrajectoryPoint::clear_path_point() {
  if (GetArenaNoVirtual() == NULL && path_point_ != NULL) {
    delete path_point_;
  }
  path_point_ = NULL;
}
inline const ::xsproto::planner::PathPoint& TrajectoryPoint::path_point() const {
  const ::xsproto::planner::PathPoint* p = path_point_;
  // @@protoc_insertion_point(field_get:xsproto.planner.TrajectoryPoint.path_point)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::planner::PathPoint*>(
      &::xsproto::planner::_PathPoint_default_instance_);
}
inline ::xsproto::planner::PathPoint* TrajectoryPoint::release_path_point() {
  // @@protoc_insertion_point(field_release:xsproto.planner.TrajectoryPoint.path_point)
  
  ::xsproto::planner::PathPoint* temp = path_point_;
  path_point_ = NULL;
  return temp;
}
inline ::xsproto::planner::PathPoint* TrajectoryPoint::mutable_path_point() {
  
  if (path_point_ == NULL) {
    path_point_ = new ::xsproto::planner::PathPoint;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.planner.TrajectoryPoint.path_point)
  return path_point_;
}
inline void TrajectoryPoint::set_allocated_path_point(::xsproto::planner::PathPoint* path_point) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete path_point_;
  }
  if (path_point) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      path_point = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, path_point, submessage_arena);
    }
    
  } else {
    
  }
  path_point_ = path_point;
  // @@protoc_insertion_point(field_set_allocated:xsproto.planner.TrajectoryPoint.path_point)
}

// double speed = 2;
inline void TrajectoryPoint::clear_speed() {
  speed_ = 0;
}
inline double TrajectoryPoint::speed() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.TrajectoryPoint.speed)
  return speed_;
}
inline void TrajectoryPoint::set_speed(double value) {
  
  speed_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.TrajectoryPoint.speed)
}

// double a = 3;
inline void TrajectoryPoint::clear_a() {
  a_ = 0;
}
inline double TrajectoryPoint::a() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.TrajectoryPoint.a)
  return a_;
}
inline void TrajectoryPoint::set_a(double value) {
  
  a_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.TrajectoryPoint.a)
}

// double da = 4;
inline void TrajectoryPoint::clear_da() {
  da_ = 0;
}
inline double TrajectoryPoint::da() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.TrajectoryPoint.da)
  return da_;
}
inline void TrajectoryPoint::set_da(double value) {
  
  da_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.TrajectoryPoint.da)
}

// double t = 5;
inline void TrajectoryPoint::clear_t() {
  t_ = 0;
}
inline double TrajectoryPoint::t() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.TrajectoryPoint.t)
  return t_;
}
inline void TrajectoryPoint::set_t(double value) {
  
  t_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.TrajectoryPoint.t)
}

// -------------------------------------------------------------------

// LocalPathInfo

// .xsproto.base.Header header = 1;
inline bool LocalPathInfo::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& LocalPathInfo::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* LocalPathInfo::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.planner.LocalPathInfo.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* LocalPathInfo::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.planner.LocalPathInfo.header)
  return header_;
}
inline void LocalPathInfo::set_allocated_header(::xsproto::base::Header* header) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(header_);
  }
  if (header) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      header = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    
  } else {
    
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:xsproto.planner.LocalPathInfo.header)
}

// repeated .xsproto.planner.TrajectoryPoint path = 2;
inline int LocalPathInfo::path_size() const {
  return path_.size();
}
inline void LocalPathInfo::clear_path() {
  path_.Clear();
}
inline const ::xsproto::planner::TrajectoryPoint& LocalPathInfo::path(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.path)
  return path_.Get(index);
}
inline ::xsproto::planner::TrajectoryPoint* LocalPathInfo::mutable_path(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.planner.LocalPathInfo.path)
  return path_.Mutable(index);
}
inline ::xsproto::planner::TrajectoryPoint* LocalPathInfo::add_path() {
  // @@protoc_insertion_point(field_add:xsproto.planner.LocalPathInfo.path)
  return path_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::planner::TrajectoryPoint >*
LocalPathInfo::mutable_path() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.planner.LocalPathInfo.path)
  return &path_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::planner::TrajectoryPoint >&
LocalPathInfo::path() const {
  // @@protoc_insertion_point(field_list:xsproto.planner.LocalPathInfo.path)
  return path_;
}

// .xsproto.planner.VehicleCommand vehicle_command = 3;
inline void LocalPathInfo::clear_vehicle_command() {
  vehicle_command_ = 0;
}
inline ::xsproto::planner::VehicleCommand LocalPathInfo::vehicle_command() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.vehicle_command)
  return static_cast< ::xsproto::planner::VehicleCommand >(vehicle_command_);
}
inline void LocalPathInfo::set_vehicle_command(::xsproto::planner::VehicleCommand value) {
  
  vehicle_command_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.vehicle_command)
}

// double speed = 4;
inline void LocalPathInfo::clear_speed() {
  speed_ = 0;
}
inline double LocalPathInfo::speed() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.speed)
  return speed_;
}
inline void LocalPathInfo::set_speed(double value) {
  
  speed_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.speed)
}

// double exp_velocity = 5;
inline void LocalPathInfo::clear_exp_velocity() {
  exp_velocity_ = 0;
}
inline double LocalPathInfo::exp_velocity() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.exp_velocity)
  return exp_velocity_;
}
inline void LocalPathInfo::set_exp_velocity(double value) {
  
  exp_velocity_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.exp_velocity)
}

// double exp_curvature = 6;
inline void LocalPathInfo::clear_exp_curvature() {
  exp_curvature_ = 0;
}
inline double LocalPathInfo::exp_curvature() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.exp_curvature)
  return exp_curvature_;
}
inline void LocalPathInfo::set_exp_curvature(double value) {
  
  exp_curvature_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.exp_curvature)
}

// int32 close_sensor = 7;
inline void LocalPathInfo::clear_close_sensor() {
  close_sensor_ = 0;
}
inline ::google::protobuf::int32 LocalPathInfo::close_sensor() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.close_sensor)
  return close_sensor_;
}
inline void LocalPathInfo::set_close_sensor(::google::protobuf::int32 value) {
  
  close_sensor_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.close_sensor)
}

// .xsproto.planner.TurningState turning_state = 8;
inline void LocalPathInfo::clear_turning_state() {
  turning_state_ = 0;
}
inline ::xsproto::planner::TurningState LocalPathInfo::turning_state() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.turning_state)
  return static_cast< ::xsproto::planner::TurningState >(turning_state_);
}
inline void LocalPathInfo::set_turning_state(::xsproto::planner::TurningState value) {
  
  turning_state_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.turning_state)
}

// double lateral_bias = 9;
inline void LocalPathInfo::clear_lateral_bias() {
  lateral_bias_ = 0;
}
inline double LocalPathInfo::lateral_bias() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.lateral_bias)
  return lateral_bias_;
}
inline void LocalPathInfo::set_lateral_bias(double value) {
  
  lateral_bias_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.lateral_bias)
}

// double acceleration = 11;
inline void LocalPathInfo::clear_acceleration() {
  acceleration_ = 0;
}
inline double LocalPathInfo::acceleration() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.acceleration)
  return acceleration_;
}
inline void LocalPathInfo::set_acceleration(double value) {
  
  acceleration_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.acceleration)
}

// double write_sys_time = 12;
inline void LocalPathInfo::clear_write_sys_time() {
  write_sys_time_ = 0;
}
inline double LocalPathInfo::write_sys_time() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.write_sys_time)
  return write_sys_time_;
}
inline void LocalPathInfo::set_write_sys_time(double value) {
  
  write_sys_time_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.write_sys_time)
}

// bool hand_brake = 13;
inline void LocalPathInfo::clear_hand_brake() {
  hand_brake_ = false;
}
inline bool LocalPathInfo::hand_brake() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.hand_brake)
  return hand_brake_;
}
inline void LocalPathInfo::set_hand_brake(bool value) {
  
  hand_brake_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.LocalPathInfo.hand_brake)
}

// .xsproto.planner.PathPoint dest_point = 14;
inline bool LocalPathInfo::has_dest_point() const {
  return this != internal_default_instance() && dest_point_ != NULL;
}
inline void LocalPathInfo::clear_dest_point() {
  if (GetArenaNoVirtual() == NULL && dest_point_ != NULL) {
    delete dest_point_;
  }
  dest_point_ = NULL;
}
inline const ::xsproto::planner::PathPoint& LocalPathInfo::dest_point() const {
  const ::xsproto::planner::PathPoint* p = dest_point_;
  // @@protoc_insertion_point(field_get:xsproto.planner.LocalPathInfo.dest_point)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::planner::PathPoint*>(
      &::xsproto::planner::_PathPoint_default_instance_);
}
inline ::xsproto::planner::PathPoint* LocalPathInfo::release_dest_point() {
  // @@protoc_insertion_point(field_release:xsproto.planner.LocalPathInfo.dest_point)
  
  ::xsproto::planner::PathPoint* temp = dest_point_;
  dest_point_ = NULL;
  return temp;
}
inline ::xsproto::planner::PathPoint* LocalPathInfo::mutable_dest_point() {
  
  if (dest_point_ == NULL) {
    dest_point_ = new ::xsproto::planner::PathPoint;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.planner.LocalPathInfo.dest_point)
  return dest_point_;
}
inline void LocalPathInfo::set_allocated_dest_point(::xsproto::planner::PathPoint* dest_point) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete dest_point_;
  }
  if (dest_point) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      dest_point = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, dest_point, submessage_arena);
    }
    
  } else {
    
  }
  dest_point_ = dest_point;
  // @@protoc_insertion_point(field_set_allocated:xsproto.planner.LocalPathInfo.dest_point)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace planner
}  // namespace xsproto

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::xsproto::planner::VehicleCommand> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::planner::VehicleCommand>() {
  return ::xsproto::planner::VehicleCommand_descriptor();
}
template <> struct is_proto_enum< ::xsproto::planner::TurningState> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::planner::TurningState>() {
  return ::xsproto::planner::TurningState_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_planner_2flocal_5fpath_5finfo_2eproto__INCLUDED
