// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/mapper_request.proto

#ifndef PROTOBUF_business_2fmapper_5frequest_2eproto__INCLUDED
#define PROTOBUF_business_2fmapper_5frequest_2eproto__INCLUDED

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
#include <google/protobuf/any.pb.h>
// @@protoc_insertion_point(includes)

namespace protobuf_business_2fmapper_5frequest_2eproto {
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
void InitDefaultsMapperRequestImpl();
void InitDefaultsMapperRequest();
void InitDefaultsMapperModeImpl();
void InitDefaultsMapperMode();
void InitDefaultsMapperEnvImpl();
void InitDefaultsMapperEnv();
inline void InitDefaults() {
  InitDefaultsMapperRequest();
  InitDefaultsMapperMode();
  InitDefaultsMapperEnv();
}
}  // namespace protobuf_business_2fmapper_5frequest_2eproto
namespace xsproto {
namespace communication {
class MapperEnv;
class MapperEnvDefaultTypeInternal;
extern MapperEnvDefaultTypeInternal _MapperEnv_default_instance_;
class MapperMode;
class MapperModeDefaultTypeInternal;
extern MapperModeDefaultTypeInternal _MapperMode_default_instance_;
class MapperRequest;
class MapperRequestDefaultTypeInternal;
extern MapperRequestDefaultTypeInternal _MapperRequest_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class MapperRequest : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.MapperRequest) */ {
 public:
  MapperRequest();
  virtual ~MapperRequest();

  MapperRequest(const MapperRequest& from);

  inline MapperRequest& operator=(const MapperRequest& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MapperRequest(MapperRequest&& from) noexcept
    : MapperRequest() {
    *this = ::std::move(from);
  }

  inline MapperRequest& operator=(MapperRequest&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const MapperRequest& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapperRequest* internal_default_instance() {
    return reinterpret_cast<const MapperRequest*>(
               &_MapperRequest_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(MapperRequest* other);
  friend void swap(MapperRequest& a, MapperRequest& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MapperRequest* New() const PROTOBUF_FINAL { return New(NULL); }

  MapperRequest* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const MapperRequest& from);
  void MergeFrom(const MapperRequest& from);
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
  void InternalSwap(MapperRequest* other);
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

  // .google.protobuf.Any msg_content = 5;
  bool has_msg_content() const;
  void clear_msg_content();
  static const int kMsgContentFieldNumber = 5;
  const ::google::protobuf::Any& msg_content() const;
  ::google::protobuf::Any* release_msg_content();
  ::google::protobuf::Any* mutable_msg_content();
  void set_allocated_msg_content(::google::protobuf::Any* msg_content);

  // int64 timestamp = 2;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 2;
  ::google::protobuf::int64 timestamp() const;
  void set_timestamp(::google::protobuf::int64 value);

  // int32 seq_num = 3;
  void clear_seq_num();
  static const int kSeqNumFieldNumber = 3;
  ::google::protobuf::int32 seq_num() const;
  void set_seq_num(::google::protobuf::int32 value);

  // int32 msg_code = 4;
  void clear_msg_code();
  static const int kMsgCodeFieldNumber = 4;
  ::google::protobuf::int32 msg_code() const;
  void set_msg_code(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.MapperRequest)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::Any* msg_content_;
  ::google::protobuf::int64 timestamp_;
  ::google::protobuf::int32 seq_num_;
  ::google::protobuf::int32 msg_code_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fmapper_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fmapper_5frequest_2eproto::InitDefaultsMapperRequestImpl();
};
// -------------------------------------------------------------------

class MapperMode : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.MapperMode) */ {
 public:
  MapperMode();
  virtual ~MapperMode();

  MapperMode(const MapperMode& from);

  inline MapperMode& operator=(const MapperMode& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MapperMode(MapperMode&& from) noexcept
    : MapperMode() {
    *this = ::std::move(from);
  }

  inline MapperMode& operator=(MapperMode&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const MapperMode& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapperMode* internal_default_instance() {
    return reinterpret_cast<const MapperMode*>(
               &_MapperMode_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(MapperMode* other);
  friend void swap(MapperMode& a, MapperMode& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MapperMode* New() const PROTOBUF_FINAL { return New(NULL); }

  MapperMode* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const MapperMode& from);
  void MergeFrom(const MapperMode& from);
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
  void InternalSwap(MapperMode* other);
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

  // int32 mode = 1;
  void clear_mode();
  static const int kModeFieldNumber = 1;
  ::google::protobuf::int32 mode() const;
  void set_mode(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.MapperMode)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 mode_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fmapper_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fmapper_5frequest_2eproto::InitDefaultsMapperModeImpl();
};
// -------------------------------------------------------------------

class MapperEnv : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.MapperEnv) */ {
 public:
  MapperEnv();
  virtual ~MapperEnv();

  MapperEnv(const MapperEnv& from);

  inline MapperEnv& operator=(const MapperEnv& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MapperEnv(MapperEnv&& from) noexcept
    : MapperEnv() {
    *this = ::std::move(from);
  }

  inline MapperEnv& operator=(MapperEnv&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const MapperEnv& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapperEnv* internal_default_instance() {
    return reinterpret_cast<const MapperEnv*>(
               &_MapperEnv_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(MapperEnv* other);
  friend void swap(MapperEnv& a, MapperEnv& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MapperEnv* New() const PROTOBUF_FINAL { return New(NULL); }

  MapperEnv* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const MapperEnv& from);
  void MergeFrom(const MapperEnv& from);
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
  void InternalSwap(MapperEnv* other);
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

  // int32 environment = 1;
  void clear_environment();
  static const int kEnvironmentFieldNumber = 1;
  ::google::protobuf::int32 environment() const;
  void set_environment(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.MapperEnv)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 environment_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fmapper_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fmapper_5frequest_2eproto::InitDefaultsMapperEnvImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MapperRequest

// .xsproto.base.Header header = 1;
inline bool MapperRequest::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& MapperRequest::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperRequest.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* MapperRequest::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MapperRequest.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* MapperRequest::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MapperRequest.header)
  return header_;
}
inline void MapperRequest::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MapperRequest.header)
}

// int64 timestamp = 2;
inline void MapperRequest::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 MapperRequest::timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperRequest.timestamp)
  return timestamp_;
}
inline void MapperRequest::set_timestamp(::google::protobuf::int64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapperRequest.timestamp)
}

// int32 seq_num = 3;
inline void MapperRequest::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 MapperRequest::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperRequest.seq_num)
  return seq_num_;
}
inline void MapperRequest::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapperRequest.seq_num)
}

// int32 msg_code = 4;
inline void MapperRequest::clear_msg_code() {
  msg_code_ = 0;
}
inline ::google::protobuf::int32 MapperRequest::msg_code() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperRequest.msg_code)
  return msg_code_;
}
inline void MapperRequest::set_msg_code(::google::protobuf::int32 value) {
  
  msg_code_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapperRequest.msg_code)
}

// .google.protobuf.Any msg_content = 5;
inline bool MapperRequest::has_msg_content() const {
  return this != internal_default_instance() && msg_content_ != NULL;
}
inline const ::google::protobuf::Any& MapperRequest::msg_content() const {
  const ::google::protobuf::Any* p = msg_content_;
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperRequest.msg_content)
  return p != NULL ? *p : *reinterpret_cast<const ::google::protobuf::Any*>(
      &::google::protobuf::_Any_default_instance_);
}
inline ::google::protobuf::Any* MapperRequest::release_msg_content() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MapperRequest.msg_content)
  
  ::google::protobuf::Any* temp = msg_content_;
  msg_content_ = NULL;
  return temp;
}
inline ::google::protobuf::Any* MapperRequest::mutable_msg_content() {
  
  if (msg_content_ == NULL) {
    msg_content_ = new ::google::protobuf::Any;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MapperRequest.msg_content)
  return msg_content_;
}
inline void MapperRequest::set_allocated_msg_content(::google::protobuf::Any* msg_content) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(msg_content_);
  }
  if (msg_content) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      msg_content = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, msg_content, submessage_arena);
    }
    
  } else {
    
  }
  msg_content_ = msg_content;
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MapperRequest.msg_content)
}

// -------------------------------------------------------------------

// MapperMode

// int32 mode = 1;
inline void MapperMode::clear_mode() {
  mode_ = 0;
}
inline ::google::protobuf::int32 MapperMode::mode() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperMode.mode)
  return mode_;
}
inline void MapperMode::set_mode(::google::protobuf::int32 value) {
  
  mode_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapperMode.mode)
}

// -------------------------------------------------------------------

// MapperEnv

// int32 environment = 1;
inline void MapperEnv::clear_environment() {
  environment_ = 0;
}
inline ::google::protobuf::int32 MapperEnv::environment() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapperEnv.environment)
  return environment_;
}
inline void MapperEnv::set_environment(::google::protobuf::int32 value) {
  
  environment_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapperEnv.environment)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fmapper_5frequest_2eproto__INCLUDED
