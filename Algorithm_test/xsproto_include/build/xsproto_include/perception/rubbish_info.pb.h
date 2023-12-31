// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: perception/rubbish_info.proto

#ifndef PROTOBUF_perception_2frubbish_5finfo_2eproto__INCLUDED
#define PROTOBUF_perception_2frubbish_5finfo_2eproto__INCLUDED

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

namespace protobuf_perception_2frubbish_5finfo_2eproto {
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
void InitDefaultsRubbishStateImpl();
void InitDefaultsRubbishState();
void InitDefaultsRubbishInfoImpl();
void InitDefaultsRubbishInfo();
inline void InitDefaults() {
  InitDefaultsRubbishState();
  InitDefaultsRubbishInfo();
}
}  // namespace protobuf_perception_2frubbish_5finfo_2eproto
namespace xsproto {
namespace perception {
class RubbishInfo;
class RubbishInfoDefaultTypeInternal;
extern RubbishInfoDefaultTypeInternal _RubbishInfo_default_instance_;
class RubbishState;
class RubbishStateDefaultTypeInternal;
extern RubbishStateDefaultTypeInternal _RubbishState_default_instance_;
}  // namespace perception
}  // namespace xsproto
namespace xsproto {
namespace perception {

// ===================================================================

class RubbishState : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.RubbishState) */ {
 public:
  RubbishState();
  virtual ~RubbishState();

  RubbishState(const RubbishState& from);

  inline RubbishState& operator=(const RubbishState& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RubbishState(RubbishState&& from) noexcept
    : RubbishState() {
    *this = ::std::move(from);
  }

  inline RubbishState& operator=(RubbishState&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RubbishState& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RubbishState* internal_default_instance() {
    return reinterpret_cast<const RubbishState*>(
               &_RubbishState_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(RubbishState* other);
  friend void swap(RubbishState& a, RubbishState& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RubbishState* New() const PROTOBUF_FINAL { return New(NULL); }

  RubbishState* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const RubbishState& from);
  void MergeFrom(const RubbishState& from);
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
  void InternalSwap(RubbishState* other);
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

  // repeated float border_x = 6;
  int border_x_size() const;
  void clear_border_x();
  static const int kBorderXFieldNumber = 6;
  float border_x(int index) const;
  void set_border_x(int index, float value);
  void add_border_x(float value);
  const ::google::protobuf::RepeatedField< float >&
      border_x() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_border_x();

  // repeated float border_y = 7;
  int border_y_size() const;
  void clear_border_y();
  static const int kBorderYFieldNumber = 7;
  float border_y(int index) const;
  void set_border_y(int index, float value);
  void add_border_y(float value);
  const ::google::protobuf::RepeatedField< float >&
      border_y() const;
  ::google::protobuf::RepeatedField< float >*
      mutable_border_y();

  // int32 class = 1;
  void clear_class_();
  static const int kClassFieldNumber = 1;
  ::google::protobuf::int32 class_() const;
  void set_class_(::google::protobuf::int32 value);

  // float score = 2;
  void clear_score();
  static const int kScoreFieldNumber = 2;
  float score() const;
  void set_score(float value);

  // float center_x = 3;
  void clear_center_x();
  static const int kCenterXFieldNumber = 3;
  float center_x() const;
  void set_center_x(float value);

  // float center_y = 4;
  void clear_center_y();
  static const int kCenterYFieldNumber = 4;
  float center_y() const;
  void set_center_y(float value);

  // float center_z = 5;
  void clear_center_z();
  static const int kCenterZFieldNumber = 5;
  float center_z() const;
  void set_center_z(float value);

  // @@protoc_insertion_point(class_scope:xsproto.perception.RubbishState)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< float > border_x_;
  mutable int _border_x_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > border_y_;
  mutable int _border_y_cached_byte_size_;
  ::google::protobuf::int32 class__;
  float score_;
  float center_x_;
  float center_y_;
  float center_z_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2frubbish_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2frubbish_5finfo_2eproto::InitDefaultsRubbishStateImpl();
};
// -------------------------------------------------------------------

class RubbishInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.perception.RubbishInfo) */ {
 public:
  RubbishInfo();
  virtual ~RubbishInfo();

  RubbishInfo(const RubbishInfo& from);

  inline RubbishInfo& operator=(const RubbishInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RubbishInfo(RubbishInfo&& from) noexcept
    : RubbishInfo() {
    *this = ::std::move(from);
  }

  inline RubbishInfo& operator=(RubbishInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RubbishInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RubbishInfo* internal_default_instance() {
    return reinterpret_cast<const RubbishInfo*>(
               &_RubbishInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(RubbishInfo* other);
  friend void swap(RubbishInfo& a, RubbishInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RubbishInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  RubbishInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const RubbishInfo& from);
  void MergeFrom(const RubbishInfo& from);
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
  void InternalSwap(RubbishInfo* other);
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

  // repeated .xsproto.perception.RubbishState rubbish = 2;
  int rubbish_size() const;
  void clear_rubbish();
  static const int kRubbishFieldNumber = 2;
  const ::xsproto::perception::RubbishState& rubbish(int index) const;
  ::xsproto::perception::RubbishState* mutable_rubbish(int index);
  ::xsproto::perception::RubbishState* add_rubbish();
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RubbishState >*
      mutable_rubbish();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RubbishState >&
      rubbish() const;

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // @@protoc_insertion_point(class_scope:xsproto.perception.RubbishInfo)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RubbishState > rubbish_;
  ::xsproto::base::Header* header_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_2frubbish_5finfo_2eproto::TableStruct;
  friend void ::protobuf_perception_2frubbish_5finfo_2eproto::InitDefaultsRubbishInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RubbishState

// int32 class = 1;
inline void RubbishState::clear_class_() {
  class__ = 0;
}
inline ::google::protobuf::int32 RubbishState::class_() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.class)
  return class__;
}
inline void RubbishState::set_class_(::google::protobuf::int32 value) {
  
  class__ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.class)
}

// float score = 2;
inline void RubbishState::clear_score() {
  score_ = 0;
}
inline float RubbishState::score() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.score)
  return score_;
}
inline void RubbishState::set_score(float value) {
  
  score_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.score)
}

// float center_x = 3;
inline void RubbishState::clear_center_x() {
  center_x_ = 0;
}
inline float RubbishState::center_x() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.center_x)
  return center_x_;
}
inline void RubbishState::set_center_x(float value) {
  
  center_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.center_x)
}

// float center_y = 4;
inline void RubbishState::clear_center_y() {
  center_y_ = 0;
}
inline float RubbishState::center_y() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.center_y)
  return center_y_;
}
inline void RubbishState::set_center_y(float value) {
  
  center_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.center_y)
}

// float center_z = 5;
inline void RubbishState::clear_center_z() {
  center_z_ = 0;
}
inline float RubbishState::center_z() const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.center_z)
  return center_z_;
}
inline void RubbishState::set_center_z(float value) {
  
  center_z_ = value;
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.center_z)
}

// repeated float border_x = 6;
inline int RubbishState::border_x_size() const {
  return border_x_.size();
}
inline void RubbishState::clear_border_x() {
  border_x_.Clear();
}
inline float RubbishState::border_x(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.border_x)
  return border_x_.Get(index);
}
inline void RubbishState::set_border_x(int index, float value) {
  border_x_.Set(index, value);
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.border_x)
}
inline void RubbishState::add_border_x(float value) {
  border_x_.Add(value);
  // @@protoc_insertion_point(field_add:xsproto.perception.RubbishState.border_x)
}
inline const ::google::protobuf::RepeatedField< float >&
RubbishState::border_x() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.RubbishState.border_x)
  return border_x_;
}
inline ::google::protobuf::RepeatedField< float >*
RubbishState::mutable_border_x() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.RubbishState.border_x)
  return &border_x_;
}

// repeated float border_y = 7;
inline int RubbishState::border_y_size() const {
  return border_y_.size();
}
inline void RubbishState::clear_border_y() {
  border_y_.Clear();
}
inline float RubbishState::border_y(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishState.border_y)
  return border_y_.Get(index);
}
inline void RubbishState::set_border_y(int index, float value) {
  border_y_.Set(index, value);
  // @@protoc_insertion_point(field_set:xsproto.perception.RubbishState.border_y)
}
inline void RubbishState::add_border_y(float value) {
  border_y_.Add(value);
  // @@protoc_insertion_point(field_add:xsproto.perception.RubbishState.border_y)
}
inline const ::google::protobuf::RepeatedField< float >&
RubbishState::border_y() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.RubbishState.border_y)
  return border_y_;
}
inline ::google::protobuf::RepeatedField< float >*
RubbishState::mutable_border_y() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.RubbishState.border_y)
  return &border_y_;
}

// -------------------------------------------------------------------

// RubbishInfo

// .xsproto.base.Header header = 1;
inline bool RubbishInfo::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& RubbishInfo::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishInfo.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* RubbishInfo::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.perception.RubbishInfo.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* RubbishInfo::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.perception.RubbishInfo.header)
  return header_;
}
inline void RubbishInfo::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.perception.RubbishInfo.header)
}

// repeated .xsproto.perception.RubbishState rubbish = 2;
inline int RubbishInfo::rubbish_size() const {
  return rubbish_.size();
}
inline void RubbishInfo::clear_rubbish() {
  rubbish_.Clear();
}
inline const ::xsproto::perception::RubbishState& RubbishInfo::rubbish(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.perception.RubbishInfo.rubbish)
  return rubbish_.Get(index);
}
inline ::xsproto::perception::RubbishState* RubbishInfo::mutable_rubbish(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.perception.RubbishInfo.rubbish)
  return rubbish_.Mutable(index);
}
inline ::xsproto::perception::RubbishState* RubbishInfo::add_rubbish() {
  // @@protoc_insertion_point(field_add:xsproto.perception.RubbishInfo.rubbish)
  return rubbish_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RubbishState >*
RubbishInfo::mutable_rubbish() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.perception.RubbishInfo.rubbish)
  return &rubbish_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::perception::RubbishState >&
RubbishInfo::rubbish() const {
  // @@protoc_insertion_point(field_list:xsproto.perception.RubbishInfo.rubbish)
  return rubbish_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_perception_2frubbish_5finfo_2eproto__INCLUDED
