// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: base/sys_data.proto

#ifndef PROTOBUF_base_2fsys_5fdata_2eproto__INCLUDED
#define PROTOBUF_base_2fsys_5fdata_2eproto__INCLUDED

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
#include "base/sys_info.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_base_2fsys_5fdata_2eproto {
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
void InitDefaultsSysDataImpl();
void InitDefaultsSysData();
inline void InitDefaults() {
  InitDefaultsSysData();
}
}  // namespace protobuf_base_2fsys_5fdata_2eproto
namespace xsproto {
namespace base {
class SysData;
class SysDataDefaultTypeInternal;
extern SysDataDefaultTypeInternal _SysData_default_instance_;
}  // namespace base
}  // namespace xsproto
namespace xsproto {
namespace base {

// ===================================================================

class SysData : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.base.SysData) */ {
 public:
  SysData();
  virtual ~SysData();

  SysData(const SysData& from);

  inline SysData& operator=(const SysData& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SysData(SysData&& from) noexcept
    : SysData() {
    *this = ::std::move(from);
  }

  inline SysData& operator=(SysData&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const SysData& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SysData* internal_default_instance() {
    return reinterpret_cast<const SysData*>(
               &_SysData_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SysData* other);
  friend void swap(SysData& a, SysData& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SysData* New() const PROTOBUF_FINAL { return New(NULL); }

  SysData* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SysData& from);
  void MergeFrom(const SysData& from);
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
  void InternalSwap(SysData* other);
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

  // repeated .xsproto.base.SysInfo sys_infos = 2;
  int sys_infos_size() const;
  void clear_sys_infos();
  static const int kSysInfosFieldNumber = 2;
  const ::xsproto::base::SysInfo& sys_infos(int index) const;
  ::xsproto::base::SysInfo* mutable_sys_infos(int index);
  ::xsproto::base::SysInfo* add_sys_infos();
  ::google::protobuf::RepeatedPtrField< ::xsproto::base::SysInfo >*
      mutable_sys_infos();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::base::SysInfo >&
      sys_infos() const;

  // .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // @@protoc_insertion_point(class_scope:xsproto.base.SysData)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::base::SysInfo > sys_infos_;
  ::xsproto::base::Header* header_;
  mutable int _cached_size_;
  friend struct ::protobuf_base_2fsys_5fdata_2eproto::TableStruct;
  friend void ::protobuf_base_2fsys_5fdata_2eproto::InitDefaultsSysDataImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SysData

// .xsproto.base.Header header = 1;
inline bool SysData::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& SysData::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.base.SysData.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* SysData::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.base.SysData.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* SysData::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.base.SysData.header)
  return header_;
}
inline void SysData::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.base.SysData.header)
}

// repeated .xsproto.base.SysInfo sys_infos = 2;
inline int SysData::sys_infos_size() const {
  return sys_infos_.size();
}
inline const ::xsproto::base::SysInfo& SysData::sys_infos(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.base.SysData.sys_infos)
  return sys_infos_.Get(index);
}
inline ::xsproto::base::SysInfo* SysData::mutable_sys_infos(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.base.SysData.sys_infos)
  return sys_infos_.Mutable(index);
}
inline ::xsproto::base::SysInfo* SysData::add_sys_infos() {
  // @@protoc_insertion_point(field_add:xsproto.base.SysData.sys_infos)
  return sys_infos_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::base::SysInfo >*
SysData::mutable_sys_infos() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.base.SysData.sys_infos)
  return &sys_infos_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::base::SysInfo >&
SysData::sys_infos() const {
  // @@protoc_insertion_point(field_list:xsproto.base.SysData.sys_infos)
  return sys_infos_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace base
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_base_2fsys_5fdata_2eproto__INCLUDED
