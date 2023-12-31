// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: perception/obstacle_object_info.proto

#ifndef PROTOBUF_perception_2fobstacle_5fobject_5finfo_2eproto__INCLUDED
#define PROTOBUF_perception_2fobstacle_5fobject_5finfo_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
#include "base/header.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_perception_2fobstacle_5fobject_5finfo_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsObstacleObjectImpl();
void InitDefaultsObstacleObject();
void InitDefaultsObstacleObjectInfoImpl();
void InitDefaultsObstacleObjectInfo();
inline void InitDefaults() {
  InitDefaultsObstacleObject();
  InitDefaultsObstacleObjectInfo();
}
}  // namespace protobuf_perception_2fobstacle_5fobject_5finfo_2eproto
namespace xsproto {
namespace perception {
class ObstacleObject;
class ObstacleObjectDefaultTypeInternal;
extern ObstacleObjectDefaultTypeInternal _ObstacleObject_default_instance_;
class ObstacleObjectInfo;
class ObstacleObjectInfoDefaultTypeInternal;
extern ObstacleObjectInfoDefaultTypeInternal _ObstacleObjectInfo_default_instance_;
}  // namespace perception
}  // namespace xsproto
namespace xsproto {
namespace perception {

// ===================================================================

class ObstacleObject : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.ObstacleObject) */ {
 public:
  ObstacleObject();
  virtual ~ObstacleObject();

  ObstacleObject(const ObstacleObject& from);

  inline ObstacleObject& operator=(const ObstacleObject& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ObstacleObject(ObstacleObject&& from) noexcept
    : ObstacleObject() {
    *this = ::std::move(from);
  }

  inline ObstacleObject& operator=(ObstacleObject&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const ObstacleObject& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ObstacleObject* internal_default_instance() {
    return reinterpret_cast<const ObstacleObject*>(
               &_ObstacleObject_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ObstacleObject* other);
  friend void swap(ObstacleObject& a, ObstacleObject& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ObstacleObject* New() const PROTOBUF_FINAL { return New(NULL); }

  ObstacleObject* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ObstacleObject& from);
  void MergeFrom(const ObstacleObject& from);
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
  void InternalSwap(ObstacleObject* other);
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

  // repeated float corner_x = 17;
  int corner_x_size() const;
  void clear_corner_x();
  static const int kCornerXFieldNumber = 17;
  float corner_x(int index) const;
  void set_corner_x(int index, float value);
  void add_corner_x(float value);
  const ::google::protobuf::RepeatedField< float >&
      corner_x() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_corner_x();

  // repeated float corner_y = 18;
  int corner_y_size() const;
  void clear_corner_y();
  static const int kCornerYFieldNumber = 18;
  float corner_y(int index) const;
  void set_corner_y(int index, float value);
  void add_corner_y(float value);
  const ::google::protobuf::RepeatedField< float >&
      corner_y() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_corner_y();

  // repeated float border_x = 20;
  int border_x_size() const;
  void clear_border_x();
  static const int kBorderXFieldNumber = 20;
  float border_x(int index) const;
  void set_border_x(int index, float value);
  void add_border_x(float value);
  const ::google::protobuf::RepeatedField< float >&
      border_x() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_border_x();

  // repeated float border_y = 21;
  int border_y_size() const;
  void clear_border_y();
  static const int kBorderYFieldNumber = 21;
  float border_y(int index) const;
  void set_border_y(int index, float value);
  void add_border_y(float value);
  const ::google::protobuf::RepeatedField< float >&
      border_y() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_border_y();

  // uint32 track_id = 1;
  void clear_track_id();
  static const int kTrackIdFieldNumber = 1;
  ::google::protobuf::uint32 track_id() const;
  void set_track_id(::google::protobuf::uint32 value);

  // uint32 track_state = 2;
  void clear_track_state();
  static const int kTrackStateFieldNumber = 2;
  ::google::protobuf::uint32 track_state() const;
  void set_track_state(::google::protobuf::uint32 value);

  // double birth_time = 4;
  void clear_birth_time();
  static const int kBirthTimeFieldNumber = 4;
  double birth_time() const;
  void set_birth_time(double value);

  // uint32 track_count = 3;
  void clear_track_count();
  static const int kTrackCountFieldNumber = 3;
  ::google::protobuf::uint32 track_count() const;
  void set_track_count(::google::protobuf::uint32 value);

  // uint32 obj_type = 6;
  void clear_obj_type();
  static const int kObjTypeFieldNumber = 6;
  ::google::protobuf::uint32 obj_type() const;
  void set_obj_type(::google::protobuf::uint32 value);

  // double lose_time = 5;
  void clear_lose_time();
  static const int kLoseTimeFieldNumber = 5;
  double lose_time() const;
  void set_lose_time(double value);

  // uint32 obj_subtype = 7;
  void clear_obj_subtype();
  static const int kObjSubtypeFieldNumber = 7;
  ::google::protobuf::uint32 obj_subtype() const;
  void set_obj_subtype(::google::protobuf::uint32 value);

  // float score = 8;
  void clear_score();
  static const int kScoreFieldNumber = 8;
  float score() const;
  void set_score(float value);

  // float center_x = 9;
  void clear_center_x();
  static const int kCenterXFieldNumber = 9;
  float center_x() const;
  void set_center_x(float value);

  // float center_y = 10;
  void clear_center_y();
  static const int kCenterYFieldNumber = 10;
  float center_y() const;
  void set_center_y(float value);

  // float width = 11;
  void clear_width();
  static const int kWidthFieldNumber = 11;
  float width() const;
  void set_width(float value);

  // float length = 12;
  void clear_length();
  static const int kLengthFieldNumber = 12;
  float length() const;
  void set_length(float value);

  // float r_angle = 13;
  void clear_r_angle();
  static const int kRAngleFieldNumber = 13;
  float r_angle() const;
  void set_r_angle(float value);

  // float height_max = 14;
  void clear_height_max();
  static const int kHeightMaxFieldNumber = 14;
  float height_max() const;
  void set_height_max(float value);

  // float height_min = 15;
  void clear_height_min();
  static const int kHeightMinFieldNumber = 15;
  float height_min() const;
  void set_height_min(float value);

  // uint32 nearest_idx = 16;
  void clear_nearest_idx();
  static const int kNearestIdxFieldNumber = 16;
  ::google::protobuf::uint32 nearest_idx() const;
  void set_nearest_idx(::google::protobuf::uint32 value);

  // uint32 border_num = 19;
  void clear_border_num();
  static const int kBorderNumFieldNumber = 19;
  ::google::protobuf::uint32 border_num() const;
  void set_border_num(::google::protobuf::uint32 value);

  // float velocity_x = 22;
  void clear_velocity_x();
  static const int kVelocityXFieldNumber = 22;
  float velocity_x() const;
  void set_velocity_x(float value);

  // float velocity_y = 23;
  void clear_velocity_y();
  static const int kVelocityYFieldNumber = 23;
  float velocity_y() const;
  void set_velocity_y(float value);

  // float speed = 24;
  void clear_speed();
  static const int kSpeedFieldNumber = 24;
  float speed() const;
  void set_speed(float value);

  // float motion_prob = 25;
  void clear_motion_prob();
  static const int kMotionProbFieldNumber = 25;
  float motion_prob() const;
  void set_motion_prob(float value);

  // uint32 source_type = 26;
  void clear_source_type();
  static const int kSourceTypeFieldNumber = 26;
  ::google::protobuf::uint32 source_type() const;
  void set_source_type(::google::protobuf::uint32 value);

  // float front_heading = 27;
  void clear_front_heading();
  static const int kFrontHeadingFieldNumber = 27;
  float front_heading() const;
  void set_front_heading(float value);

  // float angular_speed = 28;
  void clear_angular_speed();
  static const int kAngularSpeedFieldNumber = 28;
  float angular_speed() const;
  void set_angular_speed(float value);

  // @@protoc_insertion_point(class_scope:xsproto.perception.ObstacleObject)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< float > corner_x_;
  mutable int _corner_x_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > corner_y_;
  mutable int _corner_y_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > border_x_;
  mutable int _border_x_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > border_y_;
  mutable int _border_y_cached_byte_size_;
  ::google::protobuf::uint32 track_id_;
  ::google::protobuf::uint32 track_state_;
  double birth_time_;
  ::google::protobuf::uint32 track_count_;
  ::google::protobuf::uint32 obj_type_;
  double lose_time_;
  ::google::protobuf::uint32 obj_subtype_;
  float score_;
  float center_x_;
  float center_y_;
  float width_;
  float length_;
  float r_angle_;
  float height_max_;
  float height_min_;
  ::google::protobuf::uint32 nearest_idx_;
  ::google::protobuf::uint32 border_num_;
  float velocity_x_;
  float velocity_y_;
  float speed_;
  float motion_prob_;
  ::google::protobuf::uint32 source_type_;
  float front_heading_;
  float angular_speed_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2fobstacle_5fobject_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2fobstacle_5fobject_5finfo_2eproto::InitDefaultsObstacleObjectImpl();
};
// -------------------------------------------------------------------

class ObstacleObjectInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.ObstacleObjectInfo) */ {
 public:
  ObstacleObjectInfo();
  virtual ~ObstacleObjectInfo();

  ObstacleObjectInfo(const ObstacleObjectInfo& from);

  inline ObstacleObjectInfo& operator=(const ObstacleObjectInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ObstacleObjectInfo(ObstacleObjectInfo&& from) noexcept
    : ObstacleObjectInfo() {
    *this = ::std::move(from);
  }

  inline ObstacleObjectInfo& operator=(ObstacleObjectInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const ObstacleObjectInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ObstacleObjectInfo* internal_default_instance() {
    return reinterpret_cast<const ObstacleObjectInfo*>(
               &_ObstacleObjectInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(ObstacleObjectInfo* other);
  friend void swap(ObstacleObjectInfo& a, ObstacleObjectInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ObstacleObjectInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  ObstacleObjectInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ObstacleObjectInfo& from);
  void MergeFrom(const ObstacleObjectInfo& from);
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
  void InternalSwap(ObstacleObjectInfo* other);
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

  // repeated .xsproto.perception.ObstacleObject obs_objs = 3;
  int obs_objs_size() const;
  void clear_obs_objs();
  static const int kObsObjsFieldNumber = 3;
  const ::xsproto::perception::ObstacleObject& obs_objs(int index) const;
  ::xsproto::perception::ObstacleObject* mutable_obs_objs(int index);
  ::xsproto::perception::ObstacleObject* add_obs_objs();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::ObstacleObject >*
      mutable_obs_objs();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::ObstacleObject >&
      obs_objs() const;

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // uint32 output_mode = 2;
  void clear_output_mode();
  static const int kOutputModeFieldNumber = 2;
  ::google::protobuf::uint32 output_mode() const;
  void set_output_mode(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:xsproto.perception.ObstacleObjectInfo)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::ObstacleObject > obs_objs_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::uint32 output_mode_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2fobstacle_5fobject_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2fobstacle_5fobject_5finfo_2eproto::InitDefaultsObstacleObjectInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ObstacleObject

// uint32 track_id = 1;
inline void ObstacleObject::clear_track_id() {
  track_id_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::track_id() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.track_id)
  return track_id_;
}
inline void ObstacleObject::set_track_id(::google::protobuf::uint32 value) {
  
  track_id_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.track_id)
}

// uint32 track_state = 2;
inline void ObstacleObject::clear_track_state() {
  track_state_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::track_state() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.track_state)
  return track_state_;
}
inline void ObstacleObject::set_track_state(::google::protobuf::uint32 value) {
  
  track_state_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.track_state)
}

// uint32 track_count = 3;
inline void ObstacleObject::clear_track_count() {
  track_count_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::track_count() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.track_count)
  return track_count_;
}
inline void ObstacleObject::set_track_count(::google::protobuf::uint32 value) {
  
  track_count_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.track_count)
}

// double birth_time = 4;
inline void ObstacleObject::clear_birth_time() {
  birth_time_ = 0;
}
inline double ObstacleObject::birth_time() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.birth_time)
  return birth_time_;
}
inline void ObstacleObject::set_birth_time(double value) {
  
  birth_time_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.birth_time)
}

// double lose_time = 5;
inline void ObstacleObject::clear_lose_time() {
  lose_time_ = 0;
}
inline double ObstacleObject::lose_time() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.lose_time)
  return lose_time_;
}
inline void ObstacleObject::set_lose_time(double value) {
  
  lose_time_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.lose_time)
}

// uint32 obj_type = 6;
inline void ObstacleObject::clear_obj_type() {
  obj_type_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::obj_type() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.obj_type)
  return obj_type_;
}
inline void ObstacleObject::set_obj_type(::google::protobuf::uint32 value) {
  
  obj_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.obj_type)
}

// uint32 obj_subtype = 7;
inline void ObstacleObject::clear_obj_subtype() {
  obj_subtype_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::obj_subtype() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.obj_subtype)
  return obj_subtype_;
}
inline void ObstacleObject::set_obj_subtype(::google::protobuf::uint32 value) {
  
  obj_subtype_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.obj_subtype)
}

// float score = 8;
inline void ObstacleObject::clear_score() {
  score_ = 0;
}
inline float ObstacleObject::score() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.score)
  return score_;
}
inline void ObstacleObject::set_score(float value) {
  
  score_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.score)
}

// float center_x = 9;
inline void ObstacleObject::clear_center_x() {
  center_x_ = 0;
}
inline float ObstacleObject::center_x() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.center_x)
  return center_x_;
}
inline void ObstacleObject::set_center_x(float value) {
  
  center_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.center_x)
}

// float center_y = 10;
inline void ObstacleObject::clear_center_y() {
  center_y_ = 0;
}
inline float ObstacleObject::center_y() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.center_y)
  return center_y_;
}
inline void ObstacleObject::set_center_y(float value) {
  
  center_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.center_y)
}

// float width = 11;
inline void ObstacleObject::clear_width() {
  width_ = 0;
}
inline float ObstacleObject::width() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.width)
  return width_;
}
inline void ObstacleObject::set_width(float value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.width)
}

// float length = 12;
inline void ObstacleObject::clear_length() {
  length_ = 0;
}
inline float ObstacleObject::length() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.length)
  return length_;
}
inline void ObstacleObject::set_length(float value) {
  
  length_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.length)
}

// float r_angle = 13;
inline void ObstacleObject::clear_r_angle() {
  r_angle_ = 0;
}
inline float ObstacleObject::r_angle() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.r_angle)
  return r_angle_;
}
inline void ObstacleObject::set_r_angle(float value) {
  
  r_angle_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.r_angle)
}

// float height_max = 14;
inline void ObstacleObject::clear_height_max() {
  height_max_ = 0;
}
inline float ObstacleObject::height_max() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.height_max)
  return height_max_;
}
inline void ObstacleObject::set_height_max(float value) {
  
  height_max_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.height_max)
}

// float height_min = 15;
inline void ObstacleObject::clear_height_min() {
  height_min_ = 0;
}
inline float ObstacleObject::height_min() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.height_min)
  return height_min_;
}
inline void ObstacleObject::set_height_min(float value) {
  
  height_min_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.height_min)
}

// uint32 nearest_idx = 16;
inline void ObstacleObject::clear_nearest_idx() {
  nearest_idx_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::nearest_idx() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.nearest_idx)
  return nearest_idx_;
}
inline void ObstacleObject::set_nearest_idx(::google::protobuf::uint32 value) {
  
  nearest_idx_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.nearest_idx)
}

// repeated float corner_x = 17;
inline int ObstacleObject::corner_x_size() const {
  return corner_x_.size();
}
inline void ObstacleObject::clear_corner_x() {
  corner_x_.Clear();
}
inline float ObstacleObject::corner_x(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.corner_x)
  return corner_x_.Get(index);
}
inline void ObstacleObject::set_corner_x(int index, float value) {
  corner_x_.Set(index, value);
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.corner_x)
}
inline void ObstacleObject::add_corner_x(float value) {
  corner_x_.Add(value);
  // @@protoc_insertion_point(field_add:xsproto.perception.ObstacleObject.corner_x)
}
inline const ::google::protobuf::RepeatedField< float >&
ObstacleObject::corner_x() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.ObstacleObject.corner_x)
  return corner_x_;
}
inline ::google::protobuf::RepeatedField< float >*
ObstacleObject::mutable_corner_x() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.ObstacleObject.corner_x)
  return &corner_x_;
}

// repeated float corner_y = 18;
inline int ObstacleObject::corner_y_size() const {
  return corner_y_.size();
}
inline void ObstacleObject::clear_corner_y() {
  corner_y_.Clear();
}
inline float ObstacleObject::corner_y(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.corner_y)
  return corner_y_.Get(index);
}
inline void ObstacleObject::set_corner_y(int index, float value) {
  corner_y_.Set(index, value);
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.corner_y)
}
inline void ObstacleObject::add_corner_y(float value) {
  corner_y_.Add(value);
  // @@protoc_insertion_point(field_add:xsproto.perception.ObstacleObject.corner_y)
}
inline const ::google::protobuf::RepeatedField< float >&
ObstacleObject::corner_y() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.ObstacleObject.corner_y)
  return corner_y_;
}
inline ::google::protobuf::RepeatedField< float >*
ObstacleObject::mutable_corner_y() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.ObstacleObject.corner_y)
  return &corner_y_;
}

// uint32 border_num = 19;
inline void ObstacleObject::clear_border_num() {
  border_num_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::border_num() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.border_num)
  return border_num_;
}
inline void ObstacleObject::set_border_num(::google::protobuf::uint32 value) {
  
  border_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.border_num)
}

// repeated float border_x = 20;
inline int ObstacleObject::border_x_size() const {
  return border_x_.size();
}
inline void ObstacleObject::clear_border_x() {
  border_x_.Clear();
}
inline float ObstacleObject::border_x(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.border_x)
  return border_x_.Get(index);
}
inline void ObstacleObject::set_border_x(int index, float value) {
  border_x_.Set(index, value);
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.border_x)
}
inline void ObstacleObject::add_border_x(float value) {
  border_x_.Add(value);
  // @@protoc_insertion_point(field_add:xsproto.perception.ObstacleObject.border_x)
}
inline const ::google::protobuf::RepeatedField< float >&
ObstacleObject::border_x() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.ObstacleObject.border_x)
  return border_x_;
}
inline ::google::protobuf::RepeatedField< float >*
ObstacleObject::mutable_border_x() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.ObstacleObject.border_x)
  return &border_x_;
}

// repeated float border_y = 21;
inline int ObstacleObject::border_y_size() const {
  return border_y_.size();
}
inline void ObstacleObject::clear_border_y() {
  border_y_.Clear();
}
inline float ObstacleObject::border_y(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.border_y)
  return border_y_.Get(index);
}
inline void ObstacleObject::set_border_y(int index, float value) {
  border_y_.Set(index, value);
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.border_y)
}
inline void ObstacleObject::add_border_y(float value) {
  border_y_.Add(value);
  // @@protoc_insertion_point(field_add:xsproto.perception.ObstacleObject.border_y)
}
inline const ::google::protobuf::RepeatedField< float >&
ObstacleObject::border_y() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.ObstacleObject.border_y)
  return border_y_;
}
inline ::google::protobuf::RepeatedField< float >*
ObstacleObject::mutable_border_y() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.ObstacleObject.border_y)
  return &border_y_;
}

// float velocity_x = 22;
inline void ObstacleObject::clear_velocity_x() {
  velocity_x_ = 0;
}
inline float ObstacleObject::velocity_x() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.velocity_x)
  return velocity_x_;
}
inline void ObstacleObject::set_velocity_x(float value) {
  
  velocity_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.velocity_x)
}

// float velocity_y = 23;
inline void ObstacleObject::clear_velocity_y() {
  velocity_y_ = 0;
}
inline float ObstacleObject::velocity_y() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.velocity_y)
  return velocity_y_;
}
inline void ObstacleObject::set_velocity_y(float value) {
  
  velocity_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.velocity_y)
}

// float speed = 24;
inline void ObstacleObject::clear_speed() {
  speed_ = 0;
}
inline float ObstacleObject::speed() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.speed)
  return speed_;
}
inline void ObstacleObject::set_speed(float value) {
  
  speed_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.speed)
}

// float motion_prob = 25;
inline void ObstacleObject::clear_motion_prob() {
  motion_prob_ = 0;
}
inline float ObstacleObject::motion_prob() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.motion_prob)
  return motion_prob_;
}
inline void ObstacleObject::set_motion_prob(float value) {
  
  motion_prob_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.motion_prob)
}

// uint32 source_type = 26;
inline void ObstacleObject::clear_source_type() {
  source_type_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObject::source_type() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.source_type)
  return source_type_;
}
inline void ObstacleObject::set_source_type(::google::protobuf::uint32 value) {
  
  source_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.source_type)
}

// float front_heading = 27;
inline void ObstacleObject::clear_front_heading() {
  front_heading_ = 0;
}
inline float ObstacleObject::front_heading() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.front_heading)
  return front_heading_;
}
inline void ObstacleObject::set_front_heading(float value) {
  
  front_heading_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.front_heading)
}

// float angular_speed = 28;
inline void ObstacleObject::clear_angular_speed() {
  angular_speed_ = 0;
}
inline float ObstacleObject::angular_speed() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObject.angular_speed)
  return angular_speed_;
}
inline void ObstacleObject::set_angular_speed(float value) {
  
  angular_speed_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObject.angular_speed)
}

// -------------------------------------------------------------------

// ObstacleObjectInfo

// .xsproto.base.Header header = 1;
inline bool ObstacleObjectInfo::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& ObstacleObjectInfo::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObjectInfo.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* ObstacleObjectInfo::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.perception.ObstacleObjectInfo.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* ObstacleObjectInfo::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.perception.ObstacleObjectInfo.header)
  return header_;
}
inline void ObstacleObjectInfo::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.perception.ObstacleObjectInfo.header)
}

// uint32 output_mode = 2;
inline void ObstacleObjectInfo::clear_output_mode() {
  output_mode_ = 0u;
}
inline ::google::protobuf::uint32 ObstacleObjectInfo::output_mode() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObjectInfo.output_mode)
  return output_mode_;
}
inline void ObstacleObjectInfo::set_output_mode(::google::protobuf::uint32 value) {
  
  output_mode_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.ObstacleObjectInfo.output_mode)
}

// repeated .xsproto.perception.ObstacleObject obs_objs = 3;
inline int ObstacleObjectInfo::obs_objs_size() const {
  return obs_objs_.size();
}
inline void ObstacleObjectInfo::clear_obs_objs() {
  obs_objs_.Clear();
}
inline const ::xsproto::perception::ObstacleObject& ObstacleObjectInfo::obs_objs(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.ObstacleObjectInfo.obs_objs)
  return obs_objs_.Get(index);
}
inline ::xsproto::perception::ObstacleObject* ObstacleObjectInfo::mutable_obs_objs(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.ObstacleObjectInfo.obs_objs)
  return obs_objs_.Mutable(index);
}
inline ::xsproto::perception::ObstacleObject* ObstacleObjectInfo::add_obs_objs() {
  // @@protoc_insertion_point(field_add:xsproto.perception.ObstacleObjectInfo.obs_objs)
  return obs_objs_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::ObstacleObject >*
ObstacleObjectInfo::mutable_obs_objs() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.ObstacleObjectInfo.obs_objs)
  return &obs_objs_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::ObstacleObject >&
ObstacleObjectInfo::obs_objs() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.ObstacleObjectInfo.obs_objs)
  return obs_objs_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_perception_2fobstacle_5fobject_5finfo_2eproto__INCLUDED
