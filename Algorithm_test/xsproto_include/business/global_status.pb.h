// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/global_status.proto

#ifndef PROTOBUF_business_2fglobal_5fstatus_2eproto__INCLUDED
#define PROTOBUF_business_2fglobal_5fstatus_2eproto__INCLUDED

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

namespace protobuf_business_2fglobal_5fstatus_2eproto {
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
void InitDefaultsGlobalStatusImpl();
void InitDefaultsGlobalStatus();
inline void InitDefaults() {
  InitDefaultsGlobalStatus();
}
}  // namespace protobuf_business_2fglobal_5fstatus_2eproto
namespace xsproto {
namespace communication {
class GlobalStatus;
class GlobalStatusDefaultTypeInternal;
extern GlobalStatusDefaultTypeInternal _GlobalStatus_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class GlobalStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.GlobalStatus) */ {
 public:
  GlobalStatus();
  virtual ~GlobalStatus();

  GlobalStatus(const GlobalStatus& from);

  inline GlobalStatus& operator=(const GlobalStatus& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  GlobalStatus(GlobalStatus&& from) noexcept
    : GlobalStatus() {
    *this = ::std::move(from);
  }

  inline GlobalStatus& operator=(GlobalStatus&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const GlobalStatus& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const GlobalStatus* internal_default_instance() {
    return reinterpret_cast<const GlobalStatus*>(
               &_GlobalStatus_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(GlobalStatus* other);
  friend void swap(GlobalStatus& a, GlobalStatus& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline GlobalStatus* New() const PROTOBUF_FINAL { return New(NULL); }

  GlobalStatus* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const GlobalStatus& from);
  void MergeFrom(const GlobalStatus& from);
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
  void InternalSwap(GlobalStatus* other);
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

  // int32 zone = 1;
  void clear_zone();
  static const int kZoneFieldNumber = 1;
  ::google::protobuf::int32 zone() const;
  void set_zone(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.GlobalStatus)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 zone_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fglobal_5fstatus_2eproto::TableStruct;
  friend void ::protobuf_business_2fglobal_5fstatus_2eproto::InitDefaultsGlobalStatusImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// GlobalStatus

// int32 zone = 1;
inline void GlobalStatus::clear_zone() {
  zone_ = 0;
}
inline ::google::protobuf::int32 GlobalStatus::zone() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.GlobalStatus.zone)
  return zone_;
}
inline void GlobalStatus::set_zone(::google::protobuf::int32 value) {
  
  zone_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.GlobalStatus.zone)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fglobal_5fstatus_2eproto__INCLUDED