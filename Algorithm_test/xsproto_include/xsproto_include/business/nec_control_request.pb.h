// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/nec_control_request.proto

#ifndef PROTOBUF_business_2fnec_5fcontrol_5frequest_2eproto__INCLUDED
#define PROTOBUF_business_2fnec_5fcontrol_5frequest_2eproto__INCLUDED

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
#include <google/protobuf/any.pb.h>
// @@protoc_insertion_point(includes)

namespace protobuf_business_2fnec_5fcontrol_5frequest_2eproto {
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
void InitDefaultsNecControlImpl();
void InitDefaultsNecControl();
void InitDefaultsNecDeviceInfoListImpl();
void InitDefaultsNecDeviceInfoList();
void InitDefaultsNecDeviceInfoImpl();
void InitDefaultsNecDeviceInfo();
inline void InitDefaults() {
  InitDefaultsNecControl();
  InitDefaultsNecDeviceInfoList();
  InitDefaultsNecDeviceInfo();
}
}  // namespace protobuf_business_2fnec_5fcontrol_5frequest_2eproto
namespace xsproto {
namespace communication {
class NecControl;
class NecControlDefaultTypeInternal;
extern NecControlDefaultTypeInternal _NecControl_default_instance_;
class NecDeviceInfo;
class NecDeviceInfoDefaultTypeInternal;
extern NecDeviceInfoDefaultTypeInternal _NecDeviceInfo_default_instance_;
class NecDeviceInfoList;
class NecDeviceInfoListDefaultTypeInternal;
extern NecDeviceInfoListDefaultTypeInternal _NecDeviceInfoList_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

enum NecControl_ControlType {
  NecControl_ControlType_Set = 0,
  NecControl_ControlType_Get = 1,
  NecControl_ControlType_NecControl_ControlType_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  NecControl_ControlType_NecControl_ControlType_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool NecControl_ControlType_IsValid(int value);
const NecControl_ControlType NecControl_ControlType_ControlType_MIN = NecControl_ControlType_Set;
const NecControl_ControlType NecControl_ControlType_ControlType_MAX = NecControl_ControlType_Get;
const int NecControl_ControlType_ControlType_ARRAYSIZE = NecControl_ControlType_ControlType_MAX + 1;

const ::google::protobuf::EnumDescriptor* NecControl_ControlType_descriptor();
inline const ::std::string& NecControl_ControlType_Name(NecControl_ControlType value) {
  return ::google::protobuf::internal::NameOfEnum(
    NecControl_ControlType_descriptor(), value);
}
inline bool NecControl_ControlType_Parse(
    const ::std::string& name, NecControl_ControlType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<NecControl_ControlType>(
    NecControl_ControlType_descriptor(), name, value);
}
enum NecControl_ControlSource {
  NecControl_ControlSource_CI = 0,
  NecControl_ControlSource_PLANNER = 1,
  NecControl_ControlSource_NecControl_ControlSource_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  NecControl_ControlSource_NecControl_ControlSource_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool NecControl_ControlSource_IsValid(int value);
const NecControl_ControlSource NecControl_ControlSource_ControlSource_MIN = NecControl_ControlSource_CI;
const NecControl_ControlSource NecControl_ControlSource_ControlSource_MAX = NecControl_ControlSource_PLANNER;
const int NecControl_ControlSource_ControlSource_ARRAYSIZE = NecControl_ControlSource_ControlSource_MAX + 1;

const ::google::protobuf::EnumDescriptor* NecControl_ControlSource_descriptor();
inline const ::std::string& NecControl_ControlSource_Name(NecControl_ControlSource value) {
  return ::google::protobuf::internal::NameOfEnum(
    NecControl_ControlSource_descriptor(), value);
}
inline bool NecControl_ControlSource_Parse(
    const ::std::string& name, NecControl_ControlSource* value) {
  return ::google::protobuf::internal::ParseNamedEnum<NecControl_ControlSource>(
    NecControl_ControlSource_descriptor(), name, value);
}
enum NecDeviceInfo_NecDeviceType {
  NecDeviceInfo_NecDeviceType_SweepSwitch = 0,
  NecDeviceInfo_NecDeviceType_SweeperPosition = 1,
  NecDeviceInfo_NecDeviceType_FlushSwitch = 2,
  NecDeviceInfo_NecDeviceType_FlushPosition = 3,
  NecDeviceInfo_NecDeviceType_BlowerSpeed = 4,
  NecDeviceInfo_NecDeviceType_WaterPumpSpeed = 5,
  NecDeviceInfo_NecDeviceType_StartDoWork = 6,
  NecDeviceInfo_NecDeviceType_NECMode = 7,
  NecDeviceInfo_NecDeviceType_CrtDustbin = 8,
  NecDeviceInfo_NecDeviceType_NecDeviceInfo_NecDeviceType_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  NecDeviceInfo_NecDeviceType_NecDeviceInfo_NecDeviceType_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool NecDeviceInfo_NecDeviceType_IsValid(int value);
const NecDeviceInfo_NecDeviceType NecDeviceInfo_NecDeviceType_NecDeviceType_MIN = NecDeviceInfo_NecDeviceType_SweepSwitch;
const NecDeviceInfo_NecDeviceType NecDeviceInfo_NecDeviceType_NecDeviceType_MAX = NecDeviceInfo_NecDeviceType_CrtDustbin;
const int NecDeviceInfo_NecDeviceType_NecDeviceType_ARRAYSIZE = NecDeviceInfo_NecDeviceType_NecDeviceType_MAX + 1;

const ::google::protobuf::EnumDescriptor* NecDeviceInfo_NecDeviceType_descriptor();
inline const ::std::string& NecDeviceInfo_NecDeviceType_Name(NecDeviceInfo_NecDeviceType value) {
  return ::google::protobuf::internal::NameOfEnum(
    NecDeviceInfo_NecDeviceType_descriptor(), value);
}
inline bool NecDeviceInfo_NecDeviceType_Parse(
    const ::std::string& name, NecDeviceInfo_NecDeviceType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<NecDeviceInfo_NecDeviceType>(
    NecDeviceInfo_NecDeviceType_descriptor(), name, value);
}
// ===================================================================

class NecControl : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.NecControl) */ {
 public:
  NecControl();
  virtual ~NecControl();

  NecControl(const NecControl& from);

  inline NecControl& operator=(const NecControl& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  NecControl(NecControl&& from) noexcept
    : NecControl() {
    *this = ::std::move(from);
  }

  inline NecControl& operator=(NecControl&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const NecControl& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NecControl* internal_default_instance() {
    return reinterpret_cast<const NecControl*>(
               &_NecControl_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(NecControl* other);
  friend void swap(NecControl& a, NecControl& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline NecControl* New() const PROTOBUF_FINAL { return New(NULL); }

  NecControl* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const NecControl& from);
  void MergeFrom(const NecControl& from);
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
  void InternalSwap(NecControl* other);
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

  typedef NecControl_ControlType ControlType;
  static const ControlType Set =
    NecControl_ControlType_Set;
  static const ControlType Get =
    NecControl_ControlType_Get;
  static inline bool ControlType_IsValid(int value) {
    return NecControl_ControlType_IsValid(value);
  }
  static const ControlType ControlType_MIN =
    NecControl_ControlType_ControlType_MIN;
  static const ControlType ControlType_MAX =
    NecControl_ControlType_ControlType_MAX;
  static const int ControlType_ARRAYSIZE =
    NecControl_ControlType_ControlType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  ControlType_descriptor() {
    return NecControl_ControlType_descriptor();
  }
  static inline const ::std::string& ControlType_Name(ControlType value) {
    return NecControl_ControlType_Name(value);
  }
  static inline bool ControlType_Parse(const ::std::string& name,
      ControlType* value) {
    return NecControl_ControlType_Parse(name, value);
  }

  typedef NecControl_ControlSource ControlSource;
  static const ControlSource CI =
    NecControl_ControlSource_CI;
  static const ControlSource PLANNER =
    NecControl_ControlSource_PLANNER;
  static inline bool ControlSource_IsValid(int value) {
    return NecControl_ControlSource_IsValid(value);
  }
  static const ControlSource ControlSource_MIN =
    NecControl_ControlSource_ControlSource_MIN;
  static const ControlSource ControlSource_MAX =
    NecControl_ControlSource_ControlSource_MAX;
  static const int ControlSource_ARRAYSIZE =
    NecControl_ControlSource_ControlSource_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  ControlSource_descriptor() {
    return NecControl_ControlSource_descriptor();
  }
  static inline const ::std::string& ControlSource_Name(ControlSource value) {
    return NecControl_ControlSource_Name(value);
  }
  static inline bool ControlSource_Parse(const ::std::string& name,
      ControlSource* value) {
    return NecControl_ControlSource_Parse(name, value);
  }

  // accessors -------------------------------------------------------

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

  // .xsproto.communication.NecControl.ControlType control_type = 4;
  void clear_control_type();
  static const int kControlTypeFieldNumber = 4;
  ::xsproto::communication::NecControl_ControlType control_type() const;
  void set_control_type(::xsproto::communication::NecControl_ControlType value);

  // .xsproto.communication.NecControl.ControlSource control_source = 5;
  void clear_control_source();
  static const int kControlSourceFieldNumber = 5;
  ::xsproto::communication::NecControl_ControlSource control_source() const;
  void set_control_source(::xsproto::communication::NecControl_ControlSource value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.NecControl)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::Any* msg_content_;
  ::google::protobuf::int64 timestamp_;
  ::google::protobuf::int32 seq_num_;
  int control_type_;
  int control_source_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fnec_5fcontrol_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fnec_5fcontrol_5frequest_2eproto::InitDefaultsNecControlImpl();
};
// -------------------------------------------------------------------

class NecDeviceInfoList : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.NecDeviceInfoList) */ {
 public:
  NecDeviceInfoList();
  virtual ~NecDeviceInfoList();

  NecDeviceInfoList(const NecDeviceInfoList& from);

  inline NecDeviceInfoList& operator=(const NecDeviceInfoList& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  NecDeviceInfoList(NecDeviceInfoList&& from) noexcept
    : NecDeviceInfoList() {
    *this = ::std::move(from);
  }

  inline NecDeviceInfoList& operator=(NecDeviceInfoList&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const NecDeviceInfoList& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NecDeviceInfoList* internal_default_instance() {
    return reinterpret_cast<const NecDeviceInfoList*>(
               &_NecDeviceInfoList_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(NecDeviceInfoList* other);
  friend void swap(NecDeviceInfoList& a, NecDeviceInfoList& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline NecDeviceInfoList* New() const PROTOBUF_FINAL { return New(NULL); }

  NecDeviceInfoList* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const NecDeviceInfoList& from);
  void MergeFrom(const NecDeviceInfoList& from);
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
  void InternalSwap(NecDeviceInfoList* other);
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

  // repeated .xsproto.communication.NecDeviceInfo nec_device_infos = 1;
  int nec_device_infos_size() const;
  void clear_nec_device_infos();
  static const int kNecDeviceInfosFieldNumber = 1;
  const ::xsproto::communication::NecDeviceInfo& nec_device_infos(int index) const;
  ::xsproto::communication::NecDeviceInfo* mutable_nec_device_infos(int index);
  ::xsproto::communication::NecDeviceInfo* add_nec_device_infos();
  ::google::protobuf::RepeatedPtrField< ::xsproto::communication::NecDeviceInfo >*
      mutable_nec_device_infos();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::communication::NecDeviceInfo >&
      nec_device_infos() const;

  // @@protoc_insertion_point(class_scope:xsproto.communication.NecDeviceInfoList)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::communication::NecDeviceInfo > nec_device_infos_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fnec_5fcontrol_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fnec_5fcontrol_5frequest_2eproto::InitDefaultsNecDeviceInfoListImpl();
};
// -------------------------------------------------------------------

class NecDeviceInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.NecDeviceInfo) */ {
 public:
  NecDeviceInfo();
  virtual ~NecDeviceInfo();

  NecDeviceInfo(const NecDeviceInfo& from);

  inline NecDeviceInfo& operator=(const NecDeviceInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  NecDeviceInfo(NecDeviceInfo&& from) noexcept
    : NecDeviceInfo() {
    *this = ::std::move(from);
  }

  inline NecDeviceInfo& operator=(NecDeviceInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const NecDeviceInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const NecDeviceInfo* internal_default_instance() {
    return reinterpret_cast<const NecDeviceInfo*>(
               &_NecDeviceInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(NecDeviceInfo* other);
  friend void swap(NecDeviceInfo& a, NecDeviceInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline NecDeviceInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  NecDeviceInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const NecDeviceInfo& from);
  void MergeFrom(const NecDeviceInfo& from);
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
  void InternalSwap(NecDeviceInfo* other);
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

  typedef NecDeviceInfo_NecDeviceType NecDeviceType;
  static const NecDeviceType SweepSwitch =
    NecDeviceInfo_NecDeviceType_SweepSwitch;
  static const NecDeviceType SweeperPosition =
    NecDeviceInfo_NecDeviceType_SweeperPosition;
  static const NecDeviceType FlushSwitch =
    NecDeviceInfo_NecDeviceType_FlushSwitch;
  static const NecDeviceType FlushPosition =
    NecDeviceInfo_NecDeviceType_FlushPosition;
  static const NecDeviceType BlowerSpeed =
    NecDeviceInfo_NecDeviceType_BlowerSpeed;
  static const NecDeviceType WaterPumpSpeed =
    NecDeviceInfo_NecDeviceType_WaterPumpSpeed;
  static const NecDeviceType StartDoWork =
    NecDeviceInfo_NecDeviceType_StartDoWork;
  static const NecDeviceType NECMode =
    NecDeviceInfo_NecDeviceType_NECMode;
  static const NecDeviceType CrtDustbin =
    NecDeviceInfo_NecDeviceType_CrtDustbin;
  static inline bool NecDeviceType_IsValid(int value) {
    return NecDeviceInfo_NecDeviceType_IsValid(value);
  }
  static const NecDeviceType NecDeviceType_MIN =
    NecDeviceInfo_NecDeviceType_NecDeviceType_MIN;
  static const NecDeviceType NecDeviceType_MAX =
    NecDeviceInfo_NecDeviceType_NecDeviceType_MAX;
  static const int NecDeviceType_ARRAYSIZE =
    NecDeviceInfo_NecDeviceType_NecDeviceType_ARRAYSIZE;
  static inline const ::google::protobuf::EnumDescriptor*
  NecDeviceType_descriptor() {
    return NecDeviceInfo_NecDeviceType_descriptor();
  }
  static inline const ::std::string& NecDeviceType_Name(NecDeviceType value) {
    return NecDeviceInfo_NecDeviceType_Name(value);
  }
  static inline bool NecDeviceType_Parse(const ::std::string& name,
      NecDeviceType* value) {
    return NecDeviceInfo_NecDeviceType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  // .xsproto.communication.NecDeviceInfo.NecDeviceType device_type = 1;
  void clear_device_type();
  static const int kDeviceTypeFieldNumber = 1;
  ::xsproto::communication::NecDeviceInfo_NecDeviceType device_type() const;
  void set_device_type(::xsproto::communication::NecDeviceInfo_NecDeviceType value);

  // int32 device_state = 2;
  void clear_device_state();
  static const int kDeviceStateFieldNumber = 2;
  ::google::protobuf::int32 device_state() const;
  void set_device_state(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.NecDeviceInfo)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  int device_type_;
  ::google::protobuf::int32 device_state_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fnec_5fcontrol_5frequest_2eproto::TableStruct;
  friend void ::protobuf_business_2fnec_5fcontrol_5frequest_2eproto::InitDefaultsNecDeviceInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// NecControl

// .xsproto.base.Header header = 1;
inline bool NecControl::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& NecControl::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.NecControl.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* NecControl::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.NecControl.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* NecControl::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.NecControl.header)
  return header_;
}
inline void NecControl::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.NecControl.header)
}

// int64 timestamp = 2;
inline void NecControl::clear_timestamp() {
  timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 NecControl::timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecControl.timestamp)
  return timestamp_;
}
inline void NecControl::set_timestamp(::google::protobuf::int64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.NecControl.timestamp)
}

// int32 seq_num = 3;
inline void NecControl::clear_seq_num() {
  seq_num_ = 0;
}
inline ::google::protobuf::int32 NecControl::seq_num() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecControl.seq_num)
  return seq_num_;
}
inline void NecControl::set_seq_num(::google::protobuf::int32 value) {
  
  seq_num_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.NecControl.seq_num)
}

// .xsproto.communication.NecControl.ControlType control_type = 4;
inline void NecControl::clear_control_type() {
  control_type_ = 0;
}
inline ::xsproto::communication::NecControl_ControlType NecControl::control_type() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecControl.control_type)
  return static_cast< ::xsproto::communication::NecControl_ControlType >(control_type_);
}
inline void NecControl::set_control_type(::xsproto::communication::NecControl_ControlType value) {
  
  control_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.NecControl.control_type)
}

// .xsproto.communication.NecControl.ControlSource control_source = 5;
inline void NecControl::clear_control_source() {
  control_source_ = 0;
}
inline ::xsproto::communication::NecControl_ControlSource NecControl::control_source() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecControl.control_source)
  return static_cast< ::xsproto::communication::NecControl_ControlSource >(control_source_);
}
inline void NecControl::set_control_source(::xsproto::communication::NecControl_ControlSource value) {
  
  control_source_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.NecControl.control_source)
}

// .google.protobuf.Any msg_content = 6;
inline bool NecControl::has_msg_content() const {
  return this != internal_default_instance() && msg_content_ != NULL;
}
inline const ::google::protobuf::Any& NecControl::msg_content() const {
  const ::google::protobuf::Any* p = msg_content_;
  // @@protoc_insertion_point(field_get:xsproto.communication.NecControl.msg_content)
  return p != NULL ? *p : *reinterpret_cast<const ::google::protobuf::Any*>(
      &::google::protobuf::_Any_default_instance_);
}
inline ::google::protobuf::Any* NecControl::release_msg_content() {
  // @@protoc_insertion_point(field_release:xsproto.communication.NecControl.msg_content)
  
  ::google::protobuf::Any* temp = msg_content_;
  msg_content_ = NULL;
  return temp;
}
inline ::google::protobuf::Any* NecControl::mutable_msg_content() {
  
  if (msg_content_ == NULL) {
    msg_content_ = new ::google::protobuf::Any;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.NecControl.msg_content)
  return msg_content_;
}
inline void NecControl::set_allocated_msg_content(::google::protobuf::Any* msg_content) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.NecControl.msg_content)
}

// -------------------------------------------------------------------

// NecDeviceInfoList

// repeated .xsproto.communication.NecDeviceInfo nec_device_infos = 1;
inline int NecDeviceInfoList::nec_device_infos_size() const {
  return nec_device_infos_.size();
}
inline void NecDeviceInfoList::clear_nec_device_infos() {
  nec_device_infos_.Clear();
}
inline const ::xsproto::communication::NecDeviceInfo& NecDeviceInfoList::nec_device_infos(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecDeviceInfoList.nec_device_infos)
  return nec_device_infos_.Get(index);
}
inline ::xsproto::communication::NecDeviceInfo* NecDeviceInfoList::mutable_nec_device_infos(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.communication.NecDeviceInfoList.nec_device_infos)
  return nec_device_infos_.Mutable(index);
}
inline ::xsproto::communication::NecDeviceInfo* NecDeviceInfoList::add_nec_device_infos() {
  // @@protoc_insertion_point(field_add:xsproto.communication.NecDeviceInfoList.nec_device_infos)
  return nec_device_infos_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::communication::NecDeviceInfo >*
NecDeviceInfoList::mutable_nec_device_infos() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.communication.NecDeviceInfoList.nec_device_infos)
  return &nec_device_infos_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::communication::NecDeviceInfo >&
NecDeviceInfoList::nec_device_infos() const {
  // @@protoc_insertion_point(field_list:xsproto.communication.NecDeviceInfoList.nec_device_infos)
  return nec_device_infos_;
}

// -------------------------------------------------------------------

// NecDeviceInfo

// .xsproto.communication.NecDeviceInfo.NecDeviceType device_type = 1;
inline void NecDeviceInfo::clear_device_type() {
  device_type_ = 0;
}
inline ::xsproto::communication::NecDeviceInfo_NecDeviceType NecDeviceInfo::device_type() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecDeviceInfo.device_type)
  return static_cast< ::xsproto::communication::NecDeviceInfo_NecDeviceType >(device_type_);
}
inline void NecDeviceInfo::set_device_type(::xsproto::communication::NecDeviceInfo_NecDeviceType value) {
  
  device_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.NecDeviceInfo.device_type)
}

// int32 device_state = 2;
inline void NecDeviceInfo::clear_device_state() {
  device_state_ = 0;
}
inline ::google::protobuf::int32 NecDeviceInfo::device_state() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.NecDeviceInfo.device_state)
  return device_state_;
}
inline void NecDeviceInfo::set_device_state(::google::protobuf::int32 value) {
  
  device_state_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.NecDeviceInfo.device_state)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::xsproto::communication::NecControl_ControlType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::communication::NecControl_ControlType>() {
  return ::xsproto::communication::NecControl_ControlType_descriptor();
}
template <> struct is_proto_enum< ::xsproto::communication::NecControl_ControlSource> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::communication::NecControl_ControlSource>() {
  return ::xsproto::communication::NecControl_ControlSource_descriptor();
}
template <> struct is_proto_enum< ::xsproto::communication::NecDeviceInfo_NecDeviceType> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::communication::NecDeviceInfo_NecDeviceType>() {
  return ::xsproto::communication::NecDeviceInfo_NecDeviceType_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fnec_5fcontrol_5frequest_2eproto__INCLUDED
