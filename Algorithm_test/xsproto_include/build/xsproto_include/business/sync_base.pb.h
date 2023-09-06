// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/sync_base.proto

#ifndef PROTOBUF_business_2fsync_5fbase_2eproto__INCLUDED
#define PROTOBUF_business_2fsync_5fbase_2eproto__INCLUDED

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
// @@protoc_insertion_point(includes)

namespace protobuf_business_2fsync_5fbase_2eproto {
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
void InitDefaultsSyncBaseImpl();
void InitDefaultsSyncBase();
inline void InitDefaults() {
  InitDefaultsSyncBase();
}
}  // namespace protobuf_business_2fsync_5fbase_2eproto
namespace xsproto {
namespace communication {
class SyncBase;
class SyncBaseDefaultTypeInternal;
extern SyncBaseDefaultTypeInternal _SyncBase_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class SyncBase : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.SyncBase) */ {
 public:
  SyncBase();
  virtual ~SyncBase();

  SyncBase(const SyncBase& from);

  inline SyncBase& operator=(const SyncBase& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SyncBase(SyncBase&& from) noexcept
    : SyncBase() {
    *this = ::std::move(from);
  }

  inline SyncBase& operator=(SyncBase&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const SyncBase& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SyncBase* internal_default_instance() {
    return reinterpret_cast<const SyncBase*>(
               &_SyncBase_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SyncBase* other);
  friend void swap(SyncBase& a, SyncBase& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SyncBase* New() const PROTOBUF_FINAL { return New(NULL); }

  SyncBase* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SyncBase& from);
  void MergeFrom(const SyncBase& from);
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
  void InternalSwap(SyncBase* other);
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

  // string map_server_url = 3;
  void clear_map_server_url();
  static const int kMapServerUrlFieldNumber = 3;
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

  // int64 time = 1;
  void clear_time();
  static const int kTimeFieldNumber = 1;
  ::google::protobuf::int64 time() const;
  void set_time(::google::protobuf::int64 value);

  // int32 seq_num = 2;
  void clear_seq_num();
  static const int kSeqNumFieldNumber = 2;
  ::google::protobuf::int32 seq_num() const;
  void set_seq_num(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.SyncBase)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr map_server_url_;
  ::google::protobuf::int64 time_;
  ::google::protobuf::int32 seq_num_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fsync_5fbase_2eproto::TableStruct;
  friend void ::protobuf_business_2fsync_5fbase_2eproto::InitDefaultsSyncBaseImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SyncBase

// int64 time = 1;
inline void SyncBase::clear_time() {
  time_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 SyncBase::time() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SyncBase.time)
  return time_;
}
inline void SyncBase::set_time(::google::protobuf::int64 value) {
  
  time_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.SyncBase.time)
}

// int32 seq_num = 2;
inline void SyncBase::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 SyncBase::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SyncBase.seq_num)
  return seq_num_;
}
inline void SyncBase::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.SyncBase.seq_num)
}

// string map_server_url = 3;
inline void SyncBase::clear_map_server_url() {
  map_server_url_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& SyncBase::map_server_url() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SyncBase.map_server_url)
  return map_server_url_.GetNoArena();
}
inline void SyncBase::set_map_server_url(const ::std::string& value) {
  
  map_server_url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.communication.SyncBase.map_server_url)
}
#if LANG_CXX11
inline void SyncBase::set_map_server_url(::std::string&& value) {
  
  map_server_url_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.communication.SyncBase.map_server_url)
}
#endif
inline void SyncBase::set_map_server_url(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  map_server_url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.communication.SyncBase.map_server_url)
}
inline void SyncBase::set_map_server_url(const char* value, size_t size) {
  
  map_server_url_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.communication.SyncBase.map_server_url)
}
inline ::std::string* SyncBase::mutable_map_server_url() {
  
  // @@protoc_insertion_point(field_mutable:xsproto.communication.SyncBase.map_server_url)
  return map_server_url_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SyncBase::release_map_server_url() {
  // @@protoc_insertion_point(field_release:xsproto.communication.SyncBase.map_server_url)
  
  return map_server_url_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SyncBase::set_allocated_map_server_url(::std::string* map_server_url) {
  if (map_server_url != NULL) {
    
  } else {
    
  }
  map_server_url_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), map_server_url);
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.SyncBase.map_server_url)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fsync_5fbase_2eproto__INCLUDED
