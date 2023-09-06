// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/map_sync_request.proto

#ifndef PROTOBUF_business_2fmap_5fsync_5frequest_2eproto__INCLUDED
#define PROTOBUF_business_2fmap_5fsync_5frequest_2eproto__INCLUDED

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

namespace protobuf_business_2fmap_5fsync_5frequest_2eproto {
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
void InitDefaultsMapSyncRequestImpl();
void InitDefaultsMapSyncRequest();
inline void InitDefaults() {
  InitDefaultsMapSyncRequest();
}
}  // namespace protobuf_business_2fmap_5fsync_5frequest_2eproto
namespace xsproto {
namespace communication {
class MapSyncRequest;
class MapSyncRequestDefaultTypeInternal;
extern MapSyncRequestDefaultTypeInternal _MapSyncRequest_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class MapSyncRequest : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.MapSyncRequest) */ {
 public:
  MapSyncRequest();
  virtual ~MapSyncRequest();

  MapSyncRequest(const MapSyncRequest& from);

  inline MapSyncRequest& operator=(const MapSyncRequest& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MapSyncRequest(MapSyncRequest&& from) noexcept
    : MapSyncRequest() {
    *this = ::std::move(from);
  }

  inline MapSyncRequest& operator=(MapSyncRequest&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const MapSyncRequest& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MapSyncRequest* internal_default_instance() {
    return reinterpret_cast<const MapSyncRequest*>(
               &_MapSyncRequest_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(MapSyncRequest* other);
  friend void swap(MapSyncRequest& a, MapSyncRequest& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MapSyncRequest* New() const PROTOBUF_FINAL { return New(NULL); }

  MapSyncRequest* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const MapSyncRequest& from);
  void MergeFrom(const MapSyncRequest& from);
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
  void InternalSwap(MapSyncRequest* other);
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

  // string map_server_url = 5;
  void clear_map_server_url();
  static const int kMapServerUrlFieldNumber = 5;
  const ::std::string& map_server_url() const;
  void set_map_server_url(const ::std::string& value);
  #if LANG_CXX11
  void set_map_server_url(::std::string&& value);
  #endif
  void set_map_server_url(const char* value);
  void set_map_server_url(const char* value, size_t size);
  ::std::string* mutable_map_server_url();
  ::std::string* release_map_server_url();
  void set_allocated_map_server_url(::std::string* map_server_url);

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // .google.protobuf.Any msg_content = 6;
  bool has_msg_content() const;
  void clear_msg_content();
  static const int kMsgContentFieldNumber = 6;
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

  // @@protoc_insertion_point(class_scope:xsproto.communication.MapSyncRequest)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr map_server_url_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::Any* msg_content_;
  ::google::protobuf::int64 timestamp_;
  ::google::protobuf::int32 seq_num_;
  ::google::protobuf::int32 msg_code_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fmap_5fsync_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fmap_5fsync_5frequest_2eproto::InitDefaultsMapSyncRequestImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MapSyncRequest

// .xsproto.base.Header header = 1;
inline bool MapSyncRequest::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& MapSyncRequest::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.MapSyncRequest.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* MapSyncRequest::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MapSyncRequest.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* MapSyncRequest::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MapSyncRequest.header)
  return header_;
}
inline void MapSyncRequest::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MapSyncRequest.header)
}

// int64 timestamp = 2;
inline void MapSyncRequest::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 MapSyncRequest::timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapSyncRequest.timestamp)
  return timestamp_;
}
inline void MapSyncRequest::set_timestamp(::google::protobuf::int64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapSyncRequest.timestamp)
}

// int32 seq_num = 3;
inline void MapSyncRequest::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 MapSyncRequest::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapSyncRequest.seq_num)
  return seq_num_;
}
inline void MapSyncRequest::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapSyncRequest.seq_num)
}

// int32 msg_code = 4;
inline void MapSyncRequest::clear_msg_code() {
  msg_code_ = 0;
}
inline ::google::protobuf::int32 MapSyncRequest::msg_code() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapSyncRequest.msg_code)
  return msg_code_;
}
inline void MapSyncRequest::set_msg_code(::google::protobuf::int32 value) {
  
  msg_code_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MapSyncRequest.msg_code)
}

// string map_server_url = 5;
inline void MapSyncRequest::clear_map_server_url() {
  map_server_url_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& MapSyncRequest::map_server_url() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MapSyncRequest.map_server_url)
  return map_server_url_.GetNoArena();
}
inline void MapSyncRequest::set_map_server_url(const ::std::string& value) {
  
  map_server_url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.communication.MapSyncRequest.map_server_url)
}
#if LANG_CXX11
inline void MapSyncRequest::set_map_server_url(::std::string&& value) {
  
  map_server_url_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.communication.MapSyncRequest.map_server_url)
}
#endif
inline void MapSyncRequest::set_map_server_url(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  map_server_url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.communication.MapSyncRequest.map_server_url)
}
inline void MapSyncRequest::set_map_server_url(const char* value, size_t size) {
  
  map_server_url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.communication.MapSyncRequest.map_server_url)
}
inline ::std::string* MapSyncRequest::mutable_map_server_url() {
  
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MapSyncRequest.map_server_url)
  return map_server_url_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* MapSyncRequest::release_map_server_url() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MapSyncRequest.map_server_url)
  
  return map_server_url_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void MapSyncRequest::set_allocated_map_server_url(::std::string* map_server_url) {
  if (map_server_url != NULL) {
    
  } else {
    
  }
  map_server_url_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), map_server_url);
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MapSyncRequest.map_server_url)
}

// .google.protobuf.Any msg_content = 6;
inline bool MapSyncRequest::has_msg_content() const {
  return this != internal_default_instance() && msg_content_ != NULL;
}
inline const ::google::protobuf::Any& MapSyncRequest::msg_content() const {
  const ::google::protobuf::Any* p = msg_content_;
  // @@protoc_insertion_point(field_get:xsproto.communication.MapSyncRequest.msg_content)
  return p != NULL ? *p : *reinterpret_cast<const ::google::protobuf::Any*>(
      &::google::protobuf::_Any_default_instance_);
}
inline ::google::protobuf::Any* MapSyncRequest::release_msg_content() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MapSyncRequest.msg_content)
  
  ::google::protobuf::Any* temp = msg_content_;
  msg_content_ = NULL;
  return temp;
}
inline ::google::protobuf::Any* MapSyncRequest::mutable_msg_content() {
  
  if (msg_content_ == NULL) {
    msg_content_ = new ::google::protobuf::Any;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MapSyncRequest.msg_content)
  return msg_content_;
}
inline void MapSyncRequest::set_allocated_msg_content(::google::protobuf::Any* msg_content) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MapSyncRequest.msg_content)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fmap_5fsync_5frequest_2eproto__INCLUDED