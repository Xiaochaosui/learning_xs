// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: perception/traffic_sign_info.proto

#ifndef PROTOBUF_perception_2ftraffic_5fsign_5finfo_2eproto__INCLUDED
#define PROTOBUF_perception_2ftraffic_5fsign_5finfo_2eproto__INCLUDED

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

namespace protobuf_perception_2ftraffic_5fsign_5finfo_2eproto {
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
void InitDefaultsTrafficSignObjectImpl();
void InitDefaultsTrafficSignObject();
void InitDefaultsTrafficSignImpl();
void InitDefaultsTrafficSign();
void InitDefaultsTrafficSignInfoImpl();
void InitDefaultsTrafficSignInfo();
inline void InitDefaults() {
  InitDefaultsTrafficSignObject();
  InitDefaultsTrafficSign();
  InitDefaultsTrafficSignInfo();
}
}  // namespace protobuf_perception_2ftraffic_5fsign_5finfo_2eproto
namespace xsproto {
namespace perception {
class TrafficSign;
class TrafficSignDefaultTypeInternal;
extern TrafficSignDefaultTypeInternal _TrafficSign_default_instance_;
class TrafficSignInfo;
class TrafficSignInfoDefaultTypeInternal;
extern TrafficSignInfoDefaultTypeInternal _TrafficSignInfo_default_instance_;
class TrafficSignObject;
class TrafficSignObjectDefaultTypeInternal;
extern TrafficSignObjectDefaultTypeInternal _TrafficSignObject_default_instance_;
}  // namespace perception
}  // namespace xsproto
namespace xsproto {
namespace perception {

// ===================================================================

class TrafficSignObject : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.TrafficSignObject) */ {
 public:
  TrafficSignObject();
  virtual ~TrafficSignObject();

  TrafficSignObject(const TrafficSignObject& from);

  inline TrafficSignObject& operator=(const TrafficSignObject& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  TrafficSignObject(TrafficSignObject&& from) noexcept
    : TrafficSignObject() {
    *this = ::std::move(from);
  }

  inline TrafficSignObject& operator=(TrafficSignObject&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const TrafficSignObject& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrafficSignObject* internal_default_instance() {
    return reinterpret_cast<const TrafficSignObject*>(
               &_TrafficSignObject_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(TrafficSignObject* other);
  friend void swap(TrafficSignObject& a, TrafficSignObject& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline TrafficSignObject* New() const PROTOBUF_FINAL { return New(NULL); }

  TrafficSignObject* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const TrafficSignObject& from);
  void MergeFrom(const TrafficSignObject& from);
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
  void InternalSwap(TrafficSignObject* other);
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

  // int32 id = 1;
  void clear_id();
  static const int kIdFieldNumber = 1;
  ::google::protobuf::int32 id() const;
  void set_id(::google::protobuf::int32 value);

  // float x = 2;
  void clear_x();
  static const int kXFieldNumber = 2;
  float x() const;
  void set_x(float value);

  // float y = 3;
  void clear_y();
  static const int kYFieldNumber = 3;
  float y() const;
  void set_y(float value);

  // uint32 type = 4;
  void clear_type();
  static const int kTypeFieldNumber = 4;
  ::google::protobuf::uint32 type() const;
  void set_type(::google::protobuf::uint32 value);

  // sint32 lane = 5;
  void clear_lane();
  static const int kLaneFieldNumber = 5;
  ::google::protobuf::int32 lane() const;
  void set_lane(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.perception.TrafficSignObject)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 id_;
  float x_;
  float y_;
  ::google::protobuf::uint32 type_;
  ::google::protobuf::int32 lane_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2ftraffic_5fsign_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2ftraffic_5fsign_5finfo_2eproto::InitDefaultsTrafficSignObjectImpl();
};
// -------------------------------------------------------------------

class TrafficSign : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.TrafficSign) */ {
 public:
  TrafficSign();
  virtual ~TrafficSign();

  TrafficSign(const TrafficSign& from);

  inline TrafficSign& operator=(const TrafficSign& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  TrafficSign(TrafficSign&& from) noexcept
    : TrafficSign() {
    *this = ::std::move(from);
  }

  inline TrafficSign& operator=(TrafficSign&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const TrafficSign& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrafficSign* internal_default_instance() {
    return reinterpret_cast<const TrafficSign*>(
               &_TrafficSign_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(TrafficSign* other);
  friend void swap(TrafficSign& a, TrafficSign& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline TrafficSign* New() const PROTOBUF_FINAL { return New(NULL); }

  TrafficSign* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const TrafficSign& from);
  void MergeFrom(const TrafficSign& from);
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
  void InternalSwap(TrafficSign* other);
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

  // repeated .xsproto.perception.TrafficSignObject prohibition_sign = 1;
  int prohibition_sign_size() const;
  void clear_prohibition_sign();
  static const int kProhibitionSignFieldNumber = 1;
  const ::xsproto::perception::TrafficSignObject& prohibition_sign(int index) const;
  ::xsproto::perception::TrafficSignObject* mutable_prohibition_sign(int index);
  ::xsproto::perception::TrafficSignObject* add_prohibition_sign();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >*
      mutable_prohibition_sign();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >&
      prohibition_sign() const;

  // repeated .xsproto.perception.TrafficSignObject warning_sign = 2;
  int warning_sign_size() const;
  void clear_warning_sign();
  static const int kWarningSignFieldNumber = 2;
  const ::xsproto::perception::TrafficSignObject& warning_sign(int index) const;
  ::xsproto::perception::TrafficSignObject* mutable_warning_sign(int index);
  ::xsproto::perception::TrafficSignObject* add_warning_sign();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >*
      mutable_warning_sign();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >&
      warning_sign() const;

  // repeated .xsproto.perception.TrafficSignObject indication_sign = 3;
  int indication_sign_size() const;
  void clear_indication_sign();
  static const int kIndicationSignFieldNumber = 3;
  const ::xsproto::perception::TrafficSignObject& indication_sign(int index) const;
  ::xsproto::perception::TrafficSignObject* mutable_indication_sign(int index);
  ::xsproto::perception::TrafficSignObject* add_indication_sign();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >*
      mutable_indication_sign();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >&
      indication_sign() const;

  // repeated .xsproto.perception.Area2D drivable_area = 4;
  int drivable_area_size() const;
  void clear_drivable_area();
  static const int kDrivableAreaFieldNumber = 4;
  const ::xsproto::perception::Area2D& drivable_area(int index) const;
  ::xsproto::perception::Area2D* mutable_drivable_area(int index);
  ::xsproto::perception::Area2D* add_drivable_area();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
      mutable_drivable_area();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
      drivable_area() const;

  // repeated .xsproto.perception.Area2D road_curb = 5;
  int road_curb_size() const;
  void clear_road_curb();
  static const int kRoadCurbFieldNumber = 5;
  const ::xsproto::perception::Area2D& road_curb(int index) const;
  ::xsproto::perception::Area2D* mutable_road_curb(int index);
  ::xsproto::perception::Area2D* add_road_curb();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
      mutable_road_curb();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
      road_curb() const;

  // repeated .xsproto.perception.Area2D speed_bump = 6;
  int speed_bump_size() const;
  void clear_speed_bump();
  static const int kSpeedBumpFieldNumber = 6;
  const ::xsproto::perception::Area2D& speed_bump(int index) const;
  ::xsproto::perception::Area2D* mutable_speed_bump(int index);
  ::xsproto::perception::Area2D* add_speed_bump();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
      mutable_speed_bump();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
      speed_bump() const;

  // repeated .xsproto.perception.Area2D lane_line = 7;
  int lane_line_size() const;
  void clear_lane_line();
  static const int kLaneLineFieldNumber = 7;
  const ::xsproto::perception::Area2D& lane_line(int index) const;
  ::xsproto::perception::Area2D* mutable_lane_line(int index);
  ::xsproto::perception::Area2D* add_lane_line();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
      mutable_lane_line();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
      lane_line() const;

  // @@protoc_insertion_point(class_scope:xsproto.perception.TrafficSign)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject > prohibition_sign_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject > warning_sign_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject > indication_sign_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D > drivable_area_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D > road_curb_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D > speed_bump_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D > lane_line_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2ftraffic_5fsign_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2ftraffic_5fsign_5finfo_2eproto::InitDefaultsTrafficSignImpl();
};
// -------------------------------------------------------------------

class TrafficSignInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.TrafficSignInfo) */ {
 public:
  TrafficSignInfo();
  virtual ~TrafficSignInfo();

  TrafficSignInfo(const TrafficSignInfo& from);

  inline TrafficSignInfo& operator=(const TrafficSignInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  TrafficSignInfo(TrafficSignInfo&& from) noexcept
    : TrafficSignInfo() {
    *this = ::std::move(from);
  }

  inline TrafficSignInfo& operator=(TrafficSignInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const TrafficSignInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const TrafficSignInfo* internal_default_instance() {
    return reinterpret_cast<const TrafficSignInfo*>(
               &_TrafficSignInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(TrafficSignInfo* other);
  friend void swap(TrafficSignInfo& a, TrafficSignInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline TrafficSignInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  TrafficSignInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const TrafficSignInfo& from);
  void MergeFrom(const TrafficSignInfo& from);
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
  void InternalSwap(TrafficSignInfo* other);
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

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // .xsproto.perception.TrafficSign traffic_sign = 2;
  bool has_traffic_sign() const;
  void clear_traffic_sign();
  static const int kTrafficSignFieldNumber = 2;
  const ::xsproto::perception::TrafficSign& traffic_sign() const;
  ::xsproto::perception::TrafficSign* release_traffic_sign();
  ::xsproto::perception::TrafficSign* mutable_traffic_sign();
  void set_allocated_traffic_sign(::xsproto::perception::TrafficSign* traffic_sign);

  // @@protoc_insertion_point(class_scope:xsproto.perception.TrafficSignInfo)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::base::Header* header_;
  ::xsproto::perception::TrafficSign* traffic_sign_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2ftraffic_5fsign_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2ftraffic_5fsign_5finfo_2eproto::InitDefaultsTrafficSignInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// TrafficSignObject

// int32 id = 1;
inline void TrafficSignObject::clear_id() {
  id_ = 0;
}
inline ::google::protobuf::int32 TrafficSignObject::id() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignObject.id)
  return id_;
}
inline void TrafficSignObject::set_id(::google::protobuf::int32 value) {
  
  id_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.TrafficSignObject.id)
}

// float x = 2;
inline void TrafficSignObject::clear_x() {
  x_ = 0;
}
inline float TrafficSignObject::x() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignObject.x)
  return x_;
}
inline void TrafficSignObject::set_x(float value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.TrafficSignObject.x)
}

// float y = 3;
inline void TrafficSignObject::clear_y() {
  y_ = 0;
}
inline float TrafficSignObject::y() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignObject.y)
  return y_;
}
inline void TrafficSignObject::set_y(float value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.TrafficSignObject.y)
}

// uint32 type = 4;
inline void TrafficSignObject::clear_type() {
  type_ = 0u;
}
inline ::google::protobuf::uint32 TrafficSignObject::type() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignObject.type)
  return type_;
}
inline void TrafficSignObject::set_type(::google::protobuf::uint32 value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.TrafficSignObject.type)
}

// sint32 lane = 5;
inline void TrafficSignObject::clear_lane() {
  lane_ = 0;
}
inline ::google::protobuf::int32 TrafficSignObject::lane() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignObject.lane)
  return lane_;
}
inline void TrafficSignObject::set_lane(::google::protobuf::int32 value) {
  
  lane_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.TrafficSignObject.lane)
}

// -------------------------------------------------------------------

// TrafficSign

// repeated .xsproto.perception.TrafficSignObject prohibition_sign = 1;
inline int TrafficSign::prohibition_sign_size() const {
  return prohibition_sign_.size();
}
inline void TrafficSign::clear_prohibition_sign() {
  prohibition_sign_.Clear();
}
inline const ::xsproto::perception::TrafficSignObject& TrafficSign::prohibition_sign(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.prohibition_sign)
  return prohibition_sign_.Get(index);
}
inline ::xsproto::perception::TrafficSignObject* TrafficSign::mutable_prohibition_sign(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.prohibition_sign)
  return prohibition_sign_.Mutable(index);
}
inline ::xsproto::perception::TrafficSignObject* TrafficSign::add_prohibition_sign() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.prohibition_sign)
  return prohibition_sign_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >*
TrafficSign::mutable_prohibition_sign() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.prohibition_sign)
  return &prohibition_sign_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >&
TrafficSign::prohibition_sign() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.prohibition_sign)
  return prohibition_sign_;
}

// repeated .xsproto.perception.TrafficSignObject warning_sign = 2;
inline int TrafficSign::warning_sign_size() const {
  return warning_sign_.size();
}
inline void TrafficSign::clear_warning_sign() {
  warning_sign_.Clear();
}
inline const ::xsproto::perception::TrafficSignObject& TrafficSign::warning_sign(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.warning_sign)
  return warning_sign_.Get(index);
}
inline ::xsproto::perception::TrafficSignObject* TrafficSign::mutable_warning_sign(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.warning_sign)
  return warning_sign_.Mutable(index);
}
inline ::xsproto::perception::TrafficSignObject* TrafficSign::add_warning_sign() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.warning_sign)
  return warning_sign_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >*
TrafficSign::mutable_warning_sign() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.warning_sign)
  return &warning_sign_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >&
TrafficSign::warning_sign() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.warning_sign)
  return warning_sign_;
}

// repeated .xsproto.perception.TrafficSignObject indication_sign = 3;
inline int TrafficSign::indication_sign_size() const {
  return indication_sign_.size();
}
inline void TrafficSign::clear_indication_sign() {
  indication_sign_.Clear();
}
inline const ::xsproto::perception::TrafficSignObject& TrafficSign::indication_sign(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.indication_sign)
  return indication_sign_.Get(index);
}
inline ::xsproto::perception::TrafficSignObject* TrafficSign::mutable_indication_sign(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.indication_sign)
  return indication_sign_.Mutable(index);
}
inline ::xsproto::perception::TrafficSignObject* TrafficSign::add_indication_sign() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.indication_sign)
  return indication_sign_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >*
TrafficSign::mutable_indication_sign() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.indication_sign)
  return &indication_sign_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::TrafficSignObject >&
TrafficSign::indication_sign() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.indication_sign)
  return indication_sign_;
}

// repeated .xsproto.perception.Area2D drivable_area = 4;
inline int TrafficSign::drivable_area_size() const {
  return drivable_area_.size();
}
inline const ::xsproto::perception::Area2D& TrafficSign::drivable_area(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.drivable_area)
  return drivable_area_.Get(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::mutable_drivable_area(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.drivable_area)
  return drivable_area_.Mutable(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::add_drivable_area() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.drivable_area)
  return drivable_area_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
TrafficSign::mutable_drivable_area() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.drivable_area)
  return &drivable_area_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
TrafficSign::drivable_area() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.drivable_area)
  return drivable_area_;
}

// repeated .xsproto.perception.Area2D road_curb = 5;
inline int TrafficSign::road_curb_size() const {
  return road_curb_.size();
}
inline const ::xsproto::perception::Area2D& TrafficSign::road_curb(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.road_curb)
  return road_curb_.Get(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::mutable_road_curb(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.road_curb)
  return road_curb_.Mutable(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::add_road_curb() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.road_curb)
  return road_curb_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
TrafficSign::mutable_road_curb() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.road_curb)
  return &road_curb_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
TrafficSign::road_curb() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.road_curb)
  return road_curb_;
}

// repeated .xsproto.perception.Area2D speed_bump = 6;
inline int TrafficSign::speed_bump_size() const {
  return speed_bump_.size();
}
inline const ::xsproto::perception::Area2D& TrafficSign::speed_bump(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.speed_bump)
  return speed_bump_.Get(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::mutable_speed_bump(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.speed_bump)
  return speed_bump_.Mutable(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::add_speed_bump() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.speed_bump)
  return speed_bump_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
TrafficSign::mutable_speed_bump() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.speed_bump)
  return &speed_bump_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
TrafficSign::speed_bump() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.speed_bump)
  return speed_bump_;
}

// repeated .xsproto.perception.Area2D lane_line = 7;
inline int TrafficSign::lane_line_size() const {
  return lane_line_.size();
}
inline const ::xsproto::perception::Area2D& TrafficSign::lane_line(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSign.lane_line)
  return lane_line_.Get(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::mutable_lane_line(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSign.lane_line)
  return lane_line_.Mutable(index);
}
inline ::xsproto::perception::Area2D* TrafficSign::add_lane_line() {
  // @@protoc_insertion_point(field_add:xsproto.perception.TrafficSign.lane_line)
  return lane_line_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >*
TrafficSign::mutable_lane_line() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.TrafficSign.lane_line)
  return &lane_line_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::Area2D >&
TrafficSign::lane_line() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.TrafficSign.lane_line)
  return lane_line_;
}

// -------------------------------------------------------------------

// TrafficSignInfo

// .xsproto.base.Header header = 1;
inline bool TrafficSignInfo::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& TrafficSignInfo::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignInfo.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* TrafficSignInfo::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.perception.TrafficSignInfo.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* TrafficSignInfo::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSignInfo.header)
  return header_;
}
inline void TrafficSignInfo::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.perception.TrafficSignInfo.header)
}

// .xsproto.perception.TrafficSign traffic_sign = 2;
inline bool TrafficSignInfo::has_traffic_sign() const {
  return this != internal_default_instance() && traffic_sign_ != NULL;
}
inline void TrafficSignInfo::clear_traffic_sign() {
  if (GetArenaNoVirtual() == NULL && traffic_sign_ != NULL) {
    delete traffic_sign_;
  }
  traffic_sign_ = NULL;
}
inline const ::xsproto::perception::TrafficSign& TrafficSignInfo::traffic_sign() const {
  const ::xsproto::perception::TrafficSign* p = traffic_sign_;
  // @@protoc_insertion_point(field_get:xsproto.perception.TrafficSignInfo.traffic_sign)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::perception::TrafficSign*>(
      &::xsproto::perception::_TrafficSign_default_instance_);
}
inline ::xsproto::perception::TrafficSign* TrafficSignInfo::release_traffic_sign() {
  // @@protoc_insertion_point(field_release:xsproto.perception.TrafficSignInfo.traffic_sign)
  
  ::xsproto::perception::TrafficSign* temp = traffic_sign_;
  traffic_sign_ = NULL;
  return temp;
}
inline ::xsproto::perception::TrafficSign* TrafficSignInfo::mutable_traffic_sign() {
  
  if (traffic_sign_ == NULL) {
    traffic_sign_ = new ::xsproto::perception::TrafficSign;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.perception.TrafficSignInfo.traffic_sign)
  return traffic_sign_;
}
inline void TrafficSignInfo::set_allocated_traffic_sign(::xsproto::perception::TrafficSign* traffic_sign) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete traffic_sign_;
  }
  if (traffic_sign) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      traffic_sign = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, traffic_sign, submessage_arena);
    }
    
  } else {
    
  }
  traffic_sign_ = traffic_sign;
  // @@protoc_insertion_point(field_set_allocated:xsproto.perception.TrafficSignInfo.traffic_sign)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_perception_2ftraffic_5fsign_5finfo_2eproto__INCLUDED
