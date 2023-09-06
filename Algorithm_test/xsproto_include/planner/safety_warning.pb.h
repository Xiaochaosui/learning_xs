// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: planner/safety_warning.proto

#ifndef PROTOBUF_planner_2fsafety_5fwarning_2eproto__INCLUDED
#define PROTOBUF_planner_2fsafety_5fwarning_2eproto__INCLUDED

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

namespace protobuf_planner_2fsafety_5fwarning_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsSafetyWarningMessageImpl();
void InitDefaultsSafetyWarningMessage();
inline void InitDefaults() {
  InitDefaultsSafetyWarningMessage();
}
}  // namespace protobuf_planner_2fsafety_5fwarning_2eproto
namespace xsproto {
namespace planner {
class SafetyWarningMessage;
class SafetyWarningMessageDefaultTypeInternal;
extern SafetyWarningMessageDefaultTypeInternal _SafetyWarningMessage_default_instance_;
}  // namespace planner
}  // namespace xsproto
namespace xsproto {
namespace planner {

// ===================================================================

class SafetyWarningMessage : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.planner.SafetyWarningMessage) */ {
 public:
  SafetyWarningMessage();
  virtual ~SafetyWarningMessage();

  SafetyWarningMessage(const SafetyWarningMessage& from);

  inline SafetyWarningMessage& operator=(const SafetyWarningMessage& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SafetyWarningMessage(SafetyWarningMessage&& from) noexcept
    : SafetyWarningMessage() {
    *this = ::std::move(from);
  }

  inline SafetyWarningMessage& operator=(SafetyWarningMessage&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const SafetyWarningMessage& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SafetyWarningMessage* internal_default_instance() {
    return reinterpret_cast<const SafetyWarningMessage*>(
               &_SafetyWarningMessage_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SafetyWarningMessage* other);
  friend void swap(SafetyWarningMessage& a, SafetyWarningMessage& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SafetyWarningMessage* New() const PROTOBUF_FINAL { return New(NULL); }

  SafetyWarningMessage* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SafetyWarningMessage& from);
  void MergeFrom(const SafetyWarningMessage& from);
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
  void InternalSwap(SafetyWarningMessage* other);
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

  // double length = 2;
  void clear_length();
  static const int kLengthFieldNumber = 2;
  double length() const;
  void set_length(double value);

  // double min_distance = 3;
  void clear_min_distance();
  static const int kMinDistanceFieldNumber = 3;
  double min_distance() const;
  void set_min_distance(double value);

  // int32 warning_rank = 4;
  void clear_warning_rank();
  static const int kWarningRankFieldNumber = 4;
  ::google::protobuf::int32 warning_rank() const;
  void set_warning_rank(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.planner.SafetyWarningMessage)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::base::Header* header_;
  double length_;
  double min_distance_;
  ::google::protobuf::int32 warning_rank_;
  mutable int _cached_size_;
  friend struct ::protobuf_planner_2fsafety_5fwarning_2eproto::TableStruct;
  friend void ::protobuf_planner_2fsafety_5fwarning_2eproto::InitDefaultsSafetyWarningMessageImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SafetyWarningMessage

// .xsproto.base.Header header = 1;
inline bool SafetyWarningMessage::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& SafetyWarningMessage::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.planner.SafetyWarningMessage.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* SafetyWarningMessage::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.planner.SafetyWarningMessage.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* SafetyWarningMessage::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.planner.SafetyWarningMessage.header)
  return header_;
}
inline void SafetyWarningMessage::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.planner.SafetyWarningMessage.header)
}

// double length = 2;
inline void SafetyWarningMessage::clear_length() {
  length_ = 0;
}
inline double SafetyWarningMessage::length() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.SafetyWarningMessage.length)
  return length_;
}
inline void SafetyWarningMessage::set_length(double value) {
  
  length_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.SafetyWarningMessage.length)
}

// double min_distance = 3;
inline void SafetyWarningMessage::clear_min_distance() {
  min_distance_ = 0;
}
inline double SafetyWarningMessage::min_distance() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.SafetyWarningMessage.min_distance)
  return min_distance_;
}
inline void SafetyWarningMessage::set_min_distance(double value) {
  
  min_distance_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.SafetyWarningMessage.min_distance)
}

// int32 warning_rank = 4;
inline void SafetyWarningMessage::clear_warning_rank() {
  warning_rank_ = 0;
}
inline ::google::protobuf::int32 SafetyWarningMessage::warning_rank() const {
  // @@protoc_insertion_point(field_get:xsproto.planner.SafetyWarningMessage.warning_rank)
  return warning_rank_;
}
inline void SafetyWarningMessage::set_warning_rank(::google::protobuf::int32 value) {
  
  warning_rank_ = value;
  // @@protoc_insertion_point(field_set:xsproto.planner.SafetyWarningMessage.warning_rank)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planner
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_planner_2fsafety_5fwarning_2eproto__INCLUDED
