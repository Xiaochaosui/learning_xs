// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/localisation_init_request.proto

#ifndef PROTOBUF_business_2flocalisation_5finit_5frequest_2eproto__INCLUDED
#define PROTOBUF_business_2flocalisation_5finit_5frequest_2eproto__INCLUDED

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

namespace protobuf_business_2flocalisation_5finit_5frequest_2eproto {
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
void InitDefaultsLocalisationInitRequestImpl();
void InitDefaultsLocalisationInitRequest();
void InitDefaultsInitRequestPoseImpl();
void InitDefaultsInitRequestPose();
inline void InitDefaults() {
  InitDefaultsLocalisationInitRequest();
  InitDefaultsInitRequestPose();
}
}  // namespace protobuf_business_2flocalisation_5finit_5frequest_2eproto
namespace xsproto {
namespace communication {
class InitRequestPose;
class InitRequestPoseDefaultTypeInternal;
extern InitRequestPoseDefaultTypeInternal _InitRequestPose_default_instance_;
class LocalisationInitRequest;
class LocalisationInitRequestDefaultTypeInternal;
extern LocalisationInitRequestDefaultTypeInternal _LocalisationInitRequest_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class LocalisationInitRequest : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.LocalisationInitRequest) */ {
 public:
  LocalisationInitRequest();
  virtual ~LocalisationInitRequest();

  LocalisationInitRequest(const LocalisationInitRequest& from);

  inline LocalisationInitRequest& operator=(const LocalisationInitRequest& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  LocalisationInitRequest(LocalisationInitRequest&& from) noexcept
    : LocalisationInitRequest() {
    *this = ::std::move(from);
  }

  inline LocalisationInitRequest& operator=(LocalisationInitRequest&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const LocalisationInitRequest& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LocalisationInitRequest* internal_default_instance() {
    return reinterpret_cast<const LocalisationInitRequest*>(
               &_LocalisationInitRequest_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(LocalisationInitRequest* other);
  friend void swap(LocalisationInitRequest& a, LocalisationInitRequest& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline LocalisationInitRequest* New() const PROTOBUF_FINAL { return New(NULL); }

  LocalisationInitRequest* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const LocalisationInitRequest& from);
  void MergeFrom(const LocalisationInitRequest& from);
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
  void InternalSwap(LocalisationInitRequest* other);
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

  // @@protoc_insertion_point(class_scope:xsproto.communication.LocalisationInitRequest)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::Any* msg_content_;
  ::google::protobuf::int64 timestamp_;
  ::google::protobuf::int32 seq_num_;
  ::google::protobuf::int32 msg_code_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2flocalisation_5finit_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2flocalisation_5finit_5frequest_2eproto::InitDefaultsLocalisationInitRequestImpl();
};
// -------------------------------------------------------------------

class InitRequestPose : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.InitRequestPose) */ {
 public:
  InitRequestPose();
  virtual ~InitRequestPose();

  InitRequestPose(const InitRequestPose& from);

  inline InitRequestPose& operator=(const InitRequestPose& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  InitRequestPose(InitRequestPose&& from) noexcept
    : InitRequestPose() {
    *this = ::std::move(from);
  }

  inline InitRequestPose& operator=(InitRequestPose&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const InitRequestPose& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const InitRequestPose* internal_default_instance() {
    return reinterpret_cast<const InitRequestPose*>(
               &_InitRequestPose_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(InitRequestPose* other);
  friend void swap(InitRequestPose& a, InitRequestPose& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline InitRequestPose* New() const PROTOBUF_FINAL { return New(NULL); }

  InitRequestPose* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const InitRequestPose& from);
  void MergeFrom(const InitRequestPose& from);
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
  void InternalSwap(InitRequestPose* other);
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

  // int32 x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  ::google::protobuf::int32 x() const;
  void set_x(::google::protobuf::int32 value);

  // int32 y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  ::google::protobuf::int32 y() const;
  void set_y(::google::protobuf::int32 value);

  // int32 yaw = 3;
  void clear_yaw();
  static const int kYawFieldNumber = 3;
  ::google::protobuf::int32 yaw() const;
  void set_yaw(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.InitRequestPose)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 x_;
  ::google::protobuf::int32 y_;
  ::google::protobuf::int32 yaw_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2flocalisation_5finit_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2flocalisation_5finit_5frequest_2eproto::InitDefaultsInitRequestPoseImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LocalisationInitRequest

// .xsproto.base.Header header = 1;
inline bool LocalisationInitRequest::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& LocalisationInitRequest::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.LocalisationInitRequest.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* LocalisationInitRequest::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.LocalisationInitRequest.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* LocalisationInitRequest::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.LocalisationInitRequest.header)
  return header_;
}
inline void LocalisationInitRequest::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.LocalisationInitRequest.header)
}

// int64 timestamp = 2;
inline void LocalisationInitRequest::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 LocalisationInitRequest::timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.LocalisationInitRequest.timestamp)
  return timestamp_;
}
inline void LocalisationInitRequest::set_timestamp(::google::protobuf::int64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.LocalisationInitRequest.timestamp)
}

// int32 seq_num = 3;
inline void LocalisationInitRequest::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 LocalisationInitRequest::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.LocalisationInitRequest.seq_num)
  return seq_num_;
}
inline void LocalisationInitRequest::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.LocalisationInitRequest.seq_num)
}

// int32 msg_code = 4;
inline void LocalisationInitRequest::clear_msg_code() {
  msg_code_ = 0;
}
inline ::google::protobuf::int32 LocalisationInitRequest::msg_code() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.LocalisationInitRequest.msg_code)
  return msg_code_;
}
inline void LocalisationInitRequest::set_msg_code(::google::protobuf::int32 value) {
  
  msg_code_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.LocalisationInitRequest.msg_code)
}

// .google.protobuf.Any msg_content = 5;
inline bool LocalisationInitRequest::has_msg_content() const {
  return this != internal_default_instance() && msg_content_ != NULL;
}
inline const ::google::protobuf::Any& LocalisationInitRequest::msg_content() const {
  const ::google::protobuf::Any* p = msg_content_;
  // @@protoc_insertion_point(field_get:xsproto.communication.LocalisationInitRequest.msg_content)
  return p != NULL ? *p : *reinterpret_cast<const ::google::protobuf::Any*>(
      &::google::protobuf::_Any_default_instance_);
}
inline ::google::protobuf::Any* LocalisationInitRequest::release_msg_content() {
  // @@protoc_insertion_point(field_release:xsproto.communication.LocalisationInitRequest.msg_content)
  
  ::google::protobuf::Any* temp = msg_content_;
  msg_content_ = NULL;
  return temp;
}
inline ::google::protobuf::Any* LocalisationInitRequest::mutable_msg_content() {
  
  if (msg_content_ == NULL) {
    msg_content_ = new ::google::protobuf::Any;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.LocalisationInitRequest.msg_content)
  return msg_content_;
}
inline void LocalisationInitRequest::set_allocated_msg_content(::google::protobuf::Any* msg_content) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.LocalisationInitRequest.msg_content)
}

// -------------------------------------------------------------------

// InitRequestPose

// int32 x = 1;
inline void InitRequestPose::clear_x() {
  x_ = 0;
}
inline ::google::protobuf::int32 InitRequestPose::x() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.InitRequestPose.x)
  return x_;
}
inline void InitRequestPose::set_x(::google::protobuf::int32 value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.InitRequestPose.x)
}

// int32 y = 2;
inline void InitRequestPose::clear_y() {
  y_ = 0;
}
inline ::google::protobuf::int32 InitRequestPose::y() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.InitRequestPose.y)
  return y_;
}
inline void InitRequestPose::set_y(::google::protobuf::int32 value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.InitRequestPose.y)
}

// int32 yaw = 3;
inline void InitRequestPose::clear_yaw() {
  yaw_ = 0;
}
inline ::google::protobuf::int32 InitRequestPose::yaw() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.InitRequestPose.yaw)
  return yaw_;
}
inline void InitRequestPose::set_yaw(::google::protobuf::int32 value) {
  
  yaw_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.InitRequestPose.yaw)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2flocalisation_5finit_5frequest_2eproto__INCLUDED
