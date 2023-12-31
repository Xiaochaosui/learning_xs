// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: perception/radar_object_detector.proto

#ifndef PROTOBUF_perception_2fradar_5fobject_5fdetector_2eproto__INCLUDED
#define PROTOBUF_perception_2fradar_5fobject_5fdetector_2eproto__INCLUDED

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
#include "perception/perception_common.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_perception_2fradar_5fobject_5fdetector_2eproto {
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
void InitDefaultsRadarObjectFilterImpl();
void InitDefaultsRadarObjectFilter();
void InitDefaultsRadarObjectDetectorImpl();
void InitDefaultsRadarObjectDetector();
inline void InitDefaults() {
  InitDefaultsRadarObjectFilter();
  InitDefaultsRadarObjectDetector();
}
}  // namespace protobuf_perception_2fradar_5fobject_5fdetector_2eproto
namespace xsproto {
namespace perception {
class RadarObjectDetector;
class RadarObjectDetectorDefaultTypeInternal;
extern RadarObjectDetectorDefaultTypeInternal _RadarObjectDetector_default_instance_;
class RadarObjectFilter;
class RadarObjectFilterDefaultTypeInternal;
extern RadarObjectFilterDefaultTypeInternal _RadarObjectFilter_default_instance_;
}  // namespace perception
}  // namespace xsproto
namespace xsproto {
namespace perception {

// ===================================================================

class RadarObjectFilter : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.RadarObjectFilter) */ {
 public:
  RadarObjectFilter();
  virtual ~RadarObjectFilter();

  RadarObjectFilter(const RadarObjectFilter& from);

  inline RadarObjectFilter& operator=(const RadarObjectFilter& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RadarObjectFilter(RadarObjectFilter&& from) noexcept
    : RadarObjectFilter() {
    *this = ::std::move(from);
  }

  inline RadarObjectFilter& operator=(RadarObjectFilter&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RadarObjectFilter& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RadarObjectFilter* internal_default_instance() {
    return reinterpret_cast<const RadarObjectFilter*>(
               &_RadarObjectFilter_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(RadarObjectFilter* other);
  friend void swap(RadarObjectFilter& a, RadarObjectFilter& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RadarObjectFilter* New() const PROTOBUF_FINAL { return New(NULL); }

  RadarObjectFilter* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const RadarObjectFilter& from);
  void MergeFrom(const RadarObjectFilter& from);
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
  void InternalSwap(RadarObjectFilter* other);
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

  // uint32 id = 1;
  void clear_id();
  static const int kIdFieldNumber = 1;
  ::google::protobuf::uint32 id() const;
  void set_id(::google::protobuf::uint32 value);

  // uint32 type = 2;
  void clear_type();
  static const int kTypeFieldNumber = 2;
  ::google::protobuf::uint32 type() const;
  void set_type(::google::protobuf::uint32 value);

  // float width = 3;
  void clear_width();
  static const int kWidthFieldNumber = 3;
  float width() const;
  void set_width(float value);

  // float length = 4;
  void clear_length();
  static const int kLengthFieldNumber = 4;
  float length() const;
  void set_length(float value);

  // float speed_x = 5;
  void clear_speed_x();
  static const int kSpeedXFieldNumber = 5;
  float speed_x() const;
  void set_speed_x(float value);

  // float speed_y = 6;
  void clear_speed_y();
  static const int kSpeedYFieldNumber = 6;
  float speed_y() const;
  void set_speed_y(float value);

  // float pos_x = 7;
  void clear_pos_x();
  static const int kPosXFieldNumber = 7;
  float pos_x() const;
  void set_pos_x(float value);

  // float pos_y = 8;
  void clear_pos_y();
  static const int kPosYFieldNumber = 8;
  float pos_y() const;
  void set_pos_y(float value);

  // float speed_dir = 9;
  void clear_speed_dir();
  static const int kSpeedDirFieldNumber = 9;
  float speed_dir() const;
  void set_speed_dir(float value);

  // @@protoc_insertion_point(class_scope:xsproto.perception.RadarObjectFilter)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::uint32 id_;
  ::google::protobuf::uint32 type_;
  float width_;
  float length_;
  float speed_x_;
  float speed_y_;
  float pos_x_;
  float pos_y_;
  float speed_dir_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2fradar_5fobject_5fdetector_2eproto::TableStruct;
  friend void ::protobuf_perception_2fradar_5fobject_5fdetector_2eproto::InitDefaultsRadarObjectFilterImpl();
};
// -------------------------------------------------------------------

class RadarObjectDetector : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.RadarObjectDetector) */ {
 public:
  RadarObjectDetector();
  virtual ~RadarObjectDetector();

  RadarObjectDetector(const RadarObjectDetector& from);

  inline RadarObjectDetector& operator=(const RadarObjectDetector& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RadarObjectDetector(RadarObjectDetector&& from) noexcept
    : RadarObjectDetector() {
    *this = ::std::move(from);
  }

  inline RadarObjectDetector& operator=(RadarObjectDetector&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RadarObjectDetector& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RadarObjectDetector* internal_default_instance() {
    return reinterpret_cast<const RadarObjectDetector*>(
               &_RadarObjectDetector_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(RadarObjectDetector* other);
  friend void swap(RadarObjectDetector& a, RadarObjectDetector& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RadarObjectDetector* New() const PROTOBUF_FINAL { return New(NULL); }

  RadarObjectDetector* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const RadarObjectDetector& from);
  void MergeFrom(const RadarObjectDetector& from);
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
  void InternalSwap(RadarObjectDetector* other);
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

  // repeated .xsproto.perception.RadarObjectFilter radar_objects = 3;
  int radar_objects_size() const;
  void clear_radar_objects();
  static const int kRadarObjectsFieldNumber = 3;
  const ::xsproto::perception::RadarObjectFilter& radar_objects(int index) const;
  ::xsproto::perception::RadarObjectFilter* mutable_radar_objects(int index);
  ::xsproto::perception::RadarObjectFilter* add_radar_objects();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RadarObjectFilter >*
      mutable_radar_objects();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RadarObjectFilter >&
      radar_objects() const;

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // .xsproto.perception.ExtrinsicParameters extrinsic_params = 2;
  bool has_extrinsic_params() const;
  void clear_extrinsic_params();
  static const int kExtrinsicParamsFieldNumber = 2;
  const ::xsproto::perception::ExtrinsicParameters& extrinsic_params() const;
  ::xsproto::perception::ExtrinsicParameters* release_extrinsic_params();
  ::xsproto::perception::ExtrinsicParameters* mutable_extrinsic_params();
  void set_allocated_extrinsic_params(::xsproto::perception::ExtrinsicParameters* extrinsic_params);

  // uint32 radar_id = 4;
  void clear_radar_id();
  static const int kRadarIdFieldNumber = 4;
  ::google::protobuf::uint32 radar_id() const;
  void set_radar_id(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:xsproto.perception.RadarObjectDetector)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RadarObjectFilter > radar_objects_;
  ::xsproto::base::Header* header_;
  ::xsproto::perception::ExtrinsicParameters* extrinsic_params_;
  ::google::protobuf::uint32 radar_id_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2fradar_5fobject_5fdetector_2eproto::TableStruct;
  friend void ::protobuf_perception_2fradar_5fobject_5fdetector_2eproto::InitDefaultsRadarObjectDetectorImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RadarObjectFilter

// uint32 id = 1;
inline void RadarObjectFilter::clear_id() {
  id_ = 0u;
}
inline ::google::protobuf::uint32 RadarObjectFilter::id() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.id)
  return id_;
}
inline void RadarObjectFilter::set_id(::google::protobuf::uint32 value) {
  
  id_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.id)
}

// uint32 type = 2;
inline void RadarObjectFilter::clear_type() {
  type_ = 0u;
}
inline ::google::protobuf::uint32 RadarObjectFilter::type() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.type)
  return type_;
}
inline void RadarObjectFilter::set_type(::google::protobuf::uint32 value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.type)
}

// float width = 3;
inline void RadarObjectFilter::clear_width() {
  width_ = 0;
}
inline float RadarObjectFilter::width() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.width)
  return width_;
}
inline void RadarObjectFilter::set_width(float value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.width)
}

// float length = 4;
inline void RadarObjectFilter::clear_length() {
  length_ = 0;
}
inline float RadarObjectFilter::length() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.length)
  return length_;
}
inline void RadarObjectFilter::set_length(float value) {
  
  length_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.length)
}

// float speed_x = 5;
inline void RadarObjectFilter::clear_speed_x() {
  speed_x_ = 0;
}
inline float RadarObjectFilter::speed_x() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.speed_x)
  return speed_x_;
}
inline void RadarObjectFilter::set_speed_x(float value) {
  
  speed_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.speed_x)
}

// float speed_y = 6;
inline void RadarObjectFilter::clear_speed_y() {
  speed_y_ = 0;
}
inline float RadarObjectFilter::speed_y() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.speed_y)
  return speed_y_;
}
inline void RadarObjectFilter::set_speed_y(float value) {
  
  speed_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.speed_y)
}

// float pos_x = 7;
inline void RadarObjectFilter::clear_pos_x() {
  pos_x_ = 0;
}
inline float RadarObjectFilter::pos_x() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.pos_x)
  return pos_x_;
}
inline void RadarObjectFilter::set_pos_x(float value) {
  
  pos_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.pos_x)
}

// float pos_y = 8;
inline void RadarObjectFilter::clear_pos_y() {
  pos_y_ = 0;
}
inline float RadarObjectFilter::pos_y() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.pos_y)
  return pos_y_;
}
inline void RadarObjectFilter::set_pos_y(float value) {
  
  pos_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.pos_y)
}

// float speed_dir = 9;
inline void RadarObjectFilter::clear_speed_dir() {
  speed_dir_ = 0;
}
inline float RadarObjectFilter::speed_dir() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectFilter.speed_dir)
  return speed_dir_;
}
inline void RadarObjectFilter::set_speed_dir(float value) {
  
  speed_dir_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectFilter.speed_dir)
}

// -------------------------------------------------------------------

// RadarObjectDetector

// .xsproto.base.Header header = 1;
inline bool RadarObjectDetector::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& RadarObjectDetector::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectDetector.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* RadarObjectDetector::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.perception.RadarObjectDetector.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* RadarObjectDetector::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.perception.RadarObjectDetector.header)
  return header_;
}
inline void RadarObjectDetector::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.perception.RadarObjectDetector.header)
}

// .xsproto.perception.ExtrinsicParameters extrinsic_params = 2;
inline bool RadarObjectDetector::has_extrinsic_params() const {
  return this != internal_default_instance() && extrinsic_params_ != NULL;
}
inline const ::xsproto::perception::ExtrinsicParameters& RadarObjectDetector::extrinsic_params() const {
  const ::xsproto::perception::ExtrinsicParameters* p = extrinsic_params_;
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectDetector.extrinsic_params)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::perception::ExtrinsicParameters*>(
      &::xsproto::perception::_ExtrinsicParameters_default_instance_);
}
inline ::xsproto::perception::ExtrinsicParameters* RadarObjectDetector::release_extrinsic_params() {
  // @@protoc_insertion_point(field_release:xsproto.perception.RadarObjectDetector.extrinsic_params)
  
  ::xsproto::perception::ExtrinsicParameters* temp = extrinsic_params_;
  extrinsic_params_ = NULL;
  return temp;
}
inline ::xsproto::perception::ExtrinsicParameters* RadarObjectDetector::mutable_extrinsic_params() {
  
  if (extrinsic_params_ == NULL) {
    extrinsic_params_ = new ::xsproto::perception::ExtrinsicParameters;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.perception.RadarObjectDetector.extrinsic_params)
  return extrinsic_params_;
}
inline void RadarObjectDetector::set_allocated_extrinsic_params(::xsproto::perception::ExtrinsicParameters* extrinsic_params) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(extrinsic_params_);
  }
  if (extrinsic_params) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      extrinsic_params = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, extrinsic_params, submessage_arena);
    }
    
  } else {
    
  }
  extrinsic_params_ = extrinsic_params;
  // @@protoc_insertion_point(field_set_allocated:xsproto.perception.RadarObjectDetector.extrinsic_params)
}

// repeated .xsproto.perception.RadarObjectFilter radar_objects = 3;
inline int RadarObjectDetector::radar_objects_size() const {
  return radar_objects_.size();
}
inline void RadarObjectDetector::clear_radar_objects() {
  radar_objects_.Clear();
}
inline const ::xsproto::perception::RadarObjectFilter& RadarObjectDetector::radar_objects(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectDetector.radar_objects)
  return radar_objects_.Get(index);
}
inline ::xsproto::perception::RadarObjectFilter* RadarObjectDetector::mutable_radar_objects(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.RadarObjectDetector.radar_objects)
  return radar_objects_.Mutable(index);
}
inline ::xsproto::perception::RadarObjectFilter* RadarObjectDetector::add_radar_objects() {
  // @@protoc_insertion_point(field_add:xsproto.perception.RadarObjectDetector.radar_objects)
  return radar_objects_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RadarObjectFilter >*
RadarObjectDetector::mutable_radar_objects() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.RadarObjectDetector.radar_objects)
  return &radar_objects_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RadarObjectFilter >&
RadarObjectDetector::radar_objects() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.RadarObjectDetector.radar_objects)
  return radar_objects_;
}

// uint32 radar_id = 4;
inline void RadarObjectDetector::clear_radar_id() {
  radar_id_ = 0u;
}
inline ::google::protobuf::uint32 RadarObjectDetector::radar_id() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RadarObjectDetector.radar_id)
  return radar_id_;
}
inline void RadarObjectDetector::set_radar_id(::google::protobuf::uint32 value) {
  
  radar_id_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RadarObjectDetector.radar_id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_perception_2fradar_5fobject_5fdetector_2eproto__INCLUDED
