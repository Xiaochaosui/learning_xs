// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/rubbish_detector_control_response.proto

#ifndef PROTOBUF_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto__INCLUDED
#define PROTOBUF_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto__INCLUDED

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

namespace protobuf_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto {
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
void InitDefaultsRubbishDetectorControlResponseImpl();
void InitDefaultsRubbishDetectorControlResponse();
inline void InitDefaults() {
  InitDefaultsRubbishDetectorControlResponse();
}
}  // namespace protobuf_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto
namespace xsproto {
namespace communication {
class RubbishDetectorControlResponse;
class RubbishDetectorControlResponseDefaultTypeInternal;
extern RubbishDetectorControlResponseDefaultTypeInternal _RubbishDetectorControlResponse_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class RubbishDetectorControlResponse : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.RubbishDetectorControlResponse) */ {
 public:
  RubbishDetectorControlResponse();
  virtual ~RubbishDetectorControlResponse();

  RubbishDetectorControlResponse(const RubbishDetectorControlResponse& from);

  inline RubbishDetectorControlResponse& operator=(const RubbishDetectorControlResponse& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  RubbishDetectorControlResponse(RubbishDetectorControlResponse&& from) noexcept
    : RubbishDetectorControlResponse() {
    *this = ::std::move(from);
  }

  inline RubbishDetectorControlResponse& operator=(RubbishDetectorControlResponse&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const RubbishDetectorControlResponse& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const RubbishDetectorControlResponse* internal_default_instance() {
    return reinterpret_cast<const RubbishDetectorControlResponse*>(
               &_RubbishDetectorControlResponse_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(RubbishDetectorControlResponse* other);
  friend void swap(RubbishDetectorControlResponse& a, RubbishDetectorControlResponse& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline RubbishDetectorControlResponse* New() const PROTOBUF_FINAL { return New(NULL); }

  RubbishDetectorControlResponse* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const RubbishDetectorControlResponse& from);
  void MergeFrom(const RubbishDetectorControlResponse& from);
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
  void InternalSwap(RubbishDetectorControlResponse* other);
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

  // string error_msg = 3;
  void clear_error_msg();
  static const int kErrorMsgFieldNumber = 3;
  const ::std::string& error_msg() const;
  void set_error_msg(const ::std::string& value);
  #if LANG_CXX11
  void set_error_msg(::std::string&& value);
  #endif
  void set_error_msg(const char* value);
  void set_error_msg(const char* value, size_t size);
  ::std::string* mutable_error_msg();
  ::std::string* release_error_msg();
  void set_allocated_error_msg(::std::string* error_msg);

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // int32 result = 2;
  void clear_result();
  static const int kResultFieldNumber = 2;
  ::google::protobuf::int32 result() const;
  void set_result(::google::protobuf::int32 value);

  // int32 seq_num = 4;
  void clear_seq_num();
  static const int kSeqNumFieldNumber = 4;
  ::google::protobuf::int32 seq_num() const;
  void set_seq_num(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.RubbishDetectorControlResponse)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr error_msg_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::int32 result_;
  ::google::protobuf::int32 seq_num_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto::TableStruct;
  friend void ::protobuf_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto::InitDefaultsRubbishDetectorControlResponseImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// RubbishDetectorControlResponse

// .xsproto.base.Header header = 1;
inline bool RubbishDetectorControlResponse::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& RubbishDetectorControlResponse::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.RubbishDetectorControlResponse.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* RubbishDetectorControlResponse::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.RubbishDetectorControlResponse.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* RubbishDetectorControlResponse::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.RubbishDetectorControlResponse.header)
  return header_;
}
inline void RubbishDetectorControlResponse::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.RubbishDetectorControlResponse.header)
}

// int32 result = 2;
inline void RubbishDetectorControlResponse::clear_result() {
  result_ = 0;
}
inline ::google::protobuf::int32 RubbishDetectorControlResponse::result() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.RubbishDetectorControlResponse.result)
  return result_;
}
inline void RubbishDetectorControlResponse::set_result(::google::protobuf::int32 value) {
  
  result_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.RubbishDetectorControlResponse.result)
}

// string error_msg = 3;
inline void RubbishDetectorControlResponse::clear_error_msg() {
  error_msg_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& RubbishDetectorControlResponse::error_msg() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.RubbishDetectorControlResponse.error_msg)
  return error_msg_.GetNoArena();
}
inline void RubbishDetectorControlResponse::set_error_msg(const ::std::string& value) {
  
  error_msg_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.communication.RubbishDetectorControlResponse.error_msg)
}
#if LANG_CXX11
inline void RubbishDetectorControlResponse::set_error_msg(::std::string&& value) {
  
  error_msg_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.communication.RubbishDetectorControlResponse.error_msg)
}
#endif
inline void RubbishDetectorControlResponse::set_error_msg(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  error_msg_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.communication.RubbishDetectorControlResponse.error_msg)
}
inline void RubbishDetectorControlResponse::set_error_msg(const char* value, size_t size) {
  
  error_msg_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.communication.RubbishDetectorControlResponse.error_msg)
}
inline ::std::string* RubbishDetectorControlResponse::mutable_error_msg() {
  
  // @@protoc_insertion_point(field_mutable:xsproto.communication.RubbishDetectorControlResponse.error_msg)
  return error_msg_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* RubbishDetectorControlResponse::release_error_msg() {
  // @@protoc_insertion_point(field_release:xsproto.communication.RubbishDetectorControlResponse.error_msg)
  
  return error_msg_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void RubbishDetectorControlResponse::set_allocated_error_msg(::std::string* error_msg) {
  if (error_msg != NULL) {
    
  } else {
    
  }
  error_msg_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), error_msg);
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.RubbishDetectorControlResponse.error_msg)
}

// int32 seq_num = 4;
inline void RubbishDetectorControlResponse::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 RubbishDetectorControlResponse::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.RubbishDetectorControlResponse.seq_num)
  return seq_num_;
}
inline void RubbishDetectorControlResponse::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.RubbishDetectorControlResponse.seq_num)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2frubbish_5fdetector_5fcontrol_5fresponse_2eproto__INCLUDED
