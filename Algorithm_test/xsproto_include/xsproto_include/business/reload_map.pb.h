// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/reload_map.proto

#ifndef PROTOBUF_business_2freload_5fmap_2eproto__INCLUDED
#define PROTOBUF_business_2freload_5fmap_2eproto__INCLUDED

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
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
#include "base/header.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_business_2freload_5fmap_2eproto {
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
void InitDefaultsReloadMapImpl();
void InitDefaultsReloadMap();
inline void InitDefaults() {
  InitDefaultsReloadMap();
}
}  // namespace protobuf_business_2freload_5fmap_2eproto
namespace xsproto {
namespace communication {
class ReloadMap;
class ReloadMapDefaultTypeInternal;
extern ReloadMapDefaultTypeInternal _ReloadMap_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

enum ReloadMap_LoadType {
  ReloadMap_LoadType_Pause = 0,
  ReloadMap_LoadType_Reload = 1,
  ReloadMap_LoadType_Resume = 2,
  ReloadMap_LoadType_ReloadMap_LoadType_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  ReloadMap_LoadType_ReloadMap_LoadType_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool ReloadMap_LoadType_IsValid(int value);
const ReloadMap_LoadType ReloadMap_LoadType_LoadType_MIN = ReloadMap_LoadType_Pause;
const ReloadMap_LoadType ReloadMap_LoadType_LoadType_MAX = ReloadMap_LoadType_Resume;
const int ReloadMap_LoadType_LoadType_ARRAYSIZE = ReloadMap_LoadType_LoadType_MAX + 1;

const ::google::protobuf::EnumDescriptor* ReloadMap_LoadType_descriptor();
inline const ::std::string& ReloadMap_LoadType_Name(ReloadMap_LoadType value) {
  return ::google::protobuf::internal::NameOfEnum(
    ReloadMap_LoadType_descriptor(), value);
}
inline bool ReloadMap_LoadType_Parse(
    const ::std::string& name, ReloadMap_LoadType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<ReloadMap_LoadType>(
    ReloadMap_LoadType_descriptor(), name, value);
}
enum ReloadMap_ProcessType {
  ReloadMap_ProcessType_GlobalPathTask = 0,
  ReloadMap_ProcessType_LocalizerInit = 1,
  ReloadMap_ProcessType_MapLocalizer3D_xs = 2,
  ReloadMap_ProcessType_MapLocalizer2D = 3,
  ReloadMap_ProcessType_TrafficMapDetector = 4,
  ReloadMap_ProcessType_PlanMonitor = 5,
  ReloadMap_ProcessType_ReloadMap_ProcessType_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  ReloadMap_ProcessType_ReloadMap_ProcessType_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool ReloadMap_ProcessType_IsValid(int value);
const ReloadMap_ProcessType ReloadMap_ProcessType_ProcessType_MIN = ReloadMap_ProcessType_GlobalPathTask;
const ReloadMap_ProcessType ReloadMap_ProcessType_ProcessType_MAX = ReloadMap_ProcessType_PlanMonitor;
const int ReloadMap_ProcessType_ProcessType_ARRAYSIZE = ReloadMap_ProcessType_ProcessType_MAX + 1;

const ::google::protobuf::EnumDescriptor* ReloadMap_ProcessType_descriptor();
inline const ::std::string& ReloadMap_ProcessType_Name(ReloadMap_ProcessType value) {
  return ::google::protobuf::internal::NameOfEnum(
    ReloadMap_ProcessType_descriptor(), value);
}
inline bool ReloadMap_ProcessType_Parse(
    const ::std::string& name, ReloadMap_ProcessType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<ReloadMap_ProcessType>(
    ReloadMap_ProcessType_descriptor(), name, value);
}
// ===================================================================

class ReloadMap : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.ReloadMap) */ {
 public:
  ReloadMap();
  virtual ~ReloadMap();

  ReloadMap(const ReloadMap& from);

  inline ReloadMap& operator=(const ReloadMap& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ReloadMap(ReloadMap&& from) noexcept
    : ReloadMap() {
    *this = ::std::move(from);
  }

  inline ReloadMap& operator=(ReloadMap&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const ReloadMap& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ReloadMap* internal_default_instance() {
    return reinterpret_cast<const ReloadMap*>(
               &_ReloadMap_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ReloadMap* other);
  friend void swap(ReloadMap& a, ReloadMap& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ReloadMap* New() const PROTOBUF_FINAL { return New(NULL); }

  ReloadMap* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ReloadMap& from);
  void MergeFrom(const ReloadMap& from);
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
  void InternalSwap(ReloadMap* other);
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

  typedef ReloadMap_LoadType LoadType;
  static const LoadType Pause =
    ReloadMap_LoadType_Pause;
  static const LoadType Reload =
    ReloadMap_LoadType_Reload;
  static const LoadType Resume =
    ReloadMap_LoadType_Resume;
  static inline bool LoadType_IsValid(int value) {
    return ReloadMap_LoadType_IsValid(value);
  }
  static const LoadType LoadType_MIN =
    ReloadMap_LoadType_LoadType_MIN;
  static const LoadType LoadType_MAX =
    ReloadMap_LoadType_LoadType_MAX;
  static const int LoadType_ARRAYSIZE =
    ReloadMap_LoadType_LoadType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  LoadType_descriptor() {
    return ReloadMap_LoadType_descriptor();
  }
  static inline const ::std::string& LoadType_Name(LoadType value) {
    return ReloadMap_LoadType_Name(value);
  }
  static inline bool LoadType_Parse(const ::std::string& name,
      LoadType* value) {
    return ReloadMap_LoadType_Parse(name, value);
  }

  typedef ReloadMap_ProcessType ProcessType;
  static const ProcessType GlobalPathTask =
    ReloadMap_ProcessType_GlobalPathTask;
  static const ProcessType LocalizerInit =
    ReloadMap_ProcessType_LocalizerInit;
  static const ProcessType MapLocalizer3D_xs =
    ReloadMap_ProcessType_MapLocalizer3D_xs;
  static const ProcessType MapLocalizer2D =
    ReloadMap_ProcessType_MapLocalizer2D;
  static const ProcessType TrafficMapDetector =
    ReloadMap_ProcessType_TrafficMapDetector;
  static const ProcessType PlanMonitor =
    ReloadMap_ProcessType_PlanMonitor;
  static inline bool ProcessType_IsValid(int value) {
    return ReloadMap_ProcessType_IsValid(value);
  }
  static const ProcessType ProcessType_MIN =
    ReloadMap_ProcessType_ProcessType_MIN;
  static const ProcessType ProcessType_MAX =
    ReloadMap_ProcessType_ProcessType_MAX;
  static const int ProcessType_ARRAYSIZE =
    ReloadMap_ProcessType_ProcessType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  ProcessType_descriptor() {
    return ReloadMap_ProcessType_descriptor();
  }
  static inline const ::std::string& ProcessType_Name(ProcessType value) {
    return ReloadMap_ProcessType_Name(value);
  }
  static inline bool ProcessType_Parse(const ::std::string& name,
      ProcessType* value) {
    return ReloadMap_ProcessType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // string map_path = 6;
  void clear_map_path();
  static const int kMapPathFieldNumber = 6;
  const ::std::string& map_path() const;
  void set_map_path(const ::std::string& value);
  #if LANG_CXX11
  void set_map_path(::std::string&& value);
  #endif
  void set_map_path(const char* value);
  void set_map_path(const char* value, size_t size);
  ::std::string* mutable_map_path();
  ::std::string* release_map_path();
  void set_allocated_map_path(::std::string* map_path);

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

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

  // .xsproto.communication.ReloadMap.LoadType load_type = 4;
  void clear_load_type();
  static const int kLoadTypeFieldNumber = 4;
  ::xsproto::communication::ReloadMap_LoadType load_type() const;
  void set_load_type(::xsproto::communication::ReloadMap_LoadType value);

  // .xsproto.communication.ReloadMap.ProcessType process_type = 5;
  void clear_process_type();
  static const int kProcessTypeFieldNumber = 5;
  ::xsproto::communication::ReloadMap_ProcessType process_type() const;
  void set_process_type(::xsproto::communication::ReloadMap_ProcessType value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.ReloadMap)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr map_path_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::int64 timestamp_;
  ::google::protobuf::int32 seq_num_;
  int load_type_;
  int process_type_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2freload_5fmap_2eproto::TableStruct;
  friend void ::protobuf_business_2freload_5fmap_2eproto::InitDefaultsReloadMapImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ReloadMap

// .xsproto.base.Header header = 1;
inline bool ReloadMap::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& ReloadMap::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.ReloadMap.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* ReloadMap::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.ReloadMap.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* ReloadMap::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.ReloadMap.header)
  return header_;
}
inline void ReloadMap::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.ReloadMap.header)
}

// int64 timestamp = 2;
inline void ReloadMap::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 ReloadMap::timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ReloadMap.timestamp)
  return timestamp_;
}
inline void ReloadMap::set_timestamp(::google::protobuf::int64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ReloadMap.timestamp)
}

// int32 seq_num = 3;
inline void ReloadMap::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 ReloadMap::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ReloadMap.seq_num)
  return seq_num_;
}
inline void ReloadMap::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ReloadMap.seq_num)
}

// .xsproto.communication.ReloadMap.LoadType load_type = 4;
inline void ReloadMap::clear_load_type() {
  load_type_ = 0;
}
inline ::xsproto::communication::ReloadMap_LoadType ReloadMap::load_type() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ReloadMap.load_type)
  return static_cast< ::xsproto::communication::ReloadMap_LoadType >(load_type_);
}
inline void ReloadMap::set_load_type(::xsproto::communication::ReloadMap_LoadType value) {
  
  load_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ReloadMap.load_type)
}

// .xsproto.communication.ReloadMap.ProcessType process_type = 5;
inline void ReloadMap::clear_process_type() {
  process_type_ = 0;
}
inline ::xsproto::communication::ReloadMap_ProcessType ReloadMap::process_type() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ReloadMap.process_type)
  return static_cast< ::xsproto::communication::ReloadMap_ProcessType >(process_type_);
}
inline void ReloadMap::set_process_type(::xsproto::communication::ReloadMap_ProcessType value) {
  
  process_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ReloadMap.process_type)
}

// string map_path = 6;
inline void ReloadMap::clear_map_path() {
  map_path_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& ReloadMap::map_path() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ReloadMap.map_path)
  return map_path_.GetNoArena();
}
inline void ReloadMap::set_map_path(const ::std::string& value) {
  
  map_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.communication.ReloadMap.map_path)
}
#if LANG_CXX11
inline void ReloadMap::set_map_path(::std::string&& value) {
  
  map_path_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.communication.ReloadMap.map_path)
}
#endif
inline void ReloadMap::set_map_path(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  map_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.communication.ReloadMap.map_path)
}
inline void ReloadMap::set_map_path(const char* value, size_t size) {
  
  map_path_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.communication.ReloadMap.map_path)
}
inline ::std::string* ReloadMap::mutable_map_path() {
  
  // @@protoc_insertion_point(field_mutable:xsproto.communication.ReloadMap.map_path)
  return map_path_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* ReloadMap::release_map_path() {
  // @@protoc_insertion_point(field_release:xsproto.communication.ReloadMap.map_path)
  
  return map_path_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void ReloadMap::set_allocated_map_path(::std::string* map_path) {
  if (map_path != NULL) {
    
  } else {
    
  }
  map_path_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), map_path);
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.ReloadMap.map_path)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::xsproto::communication::ReloadMap_LoadType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::communication::ReloadMap_LoadType>() {
  return ::xsproto::communication::ReloadMap_LoadType_descriptor();
}
template <> struct is_proto_enum< ::xsproto::communication::ReloadMap_ProcessType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::communication::ReloadMap_ProcessType>() {
  return ::xsproto::communication::ReloadMap_ProcessType_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2freload_5fmap_2eproto__INCLUDED
