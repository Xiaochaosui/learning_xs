// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/selfcheck.proto

#ifndef PROTOBUF_business_2fselfcheck_2eproto__INCLUDED
#define PROTOBUF_business_2fselfcheck_2eproto__INCLUDED

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
// @@protoc_insertion_point(includes)

namespace protobuf_business_2fselfcheck_2eproto {
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
void InitDefaultsSelfCheckImpl();
void InitDefaultsSelfCheck();
void InitDefaultsSelfCheckInfoImpl();
void InitDefaultsSelfCheckInfo();
inline void InitDefaults() {
  InitDefaultsSelfCheck();
  InitDefaultsSelfCheckInfo();
}
}  // namespace protobuf_business_2fselfcheck_2eproto
namespace xsproto {
namespace communication {
class SelfCheck;
class SelfCheckDefaultTypeInternal;
extern SelfCheckDefaultTypeInternal _SelfCheck_default_instance_;
class SelfCheckInfo;
class SelfCheckInfoDefaultTypeInternal;
extern SelfCheckInfoDefaultTypeInternal _SelfCheckInfo_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

enum SelfCheckState {
  CHECKING = 0,
  SUCCESS = 1,
  FAIL = 2,
  SelfCheckState_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  SelfCheckState_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool SelfCheckState_IsValid(int value);
const SelfCheckState SelfCheckState_MIN = CHECKING;
const SelfCheckState SelfCheckState_MAX = FAIL;
const int SelfCheckState_ARRAYSIZE = SelfCheckState_MAX + 1;

const ::google::protobuf::EnumDescriptor* SelfCheckState_descriptor();
inline const ::std::string& SelfCheckState_Name(SelfCheckState value) {
  return ::google::protobuf::internal::NameOfEnum(
    SelfCheckState_descriptor(), value);
}
inline bool SelfCheckState_Parse(
    const ::std::string& name, SelfCheckState* value) {
  return ::google::protobuf::internal::ParseNamedEnum<SelfCheckState>(
    SelfCheckState_descriptor(), name, value);
}
// ===================================================================

class SelfCheck : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.SelfCheck) */ {
 public:
  SelfCheck();
  virtual ~SelfCheck();

  SelfCheck(const SelfCheck& from);

  inline SelfCheck& operator=(const SelfCheck& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SelfCheck(SelfCheck&& from) noexcept
    : SelfCheck() {
    *this = ::std::move(from);
  }

  inline SelfCheck& operator=(SelfCheck&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const SelfCheck& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SelfCheck* internal_default_instance() {
    return reinterpret_cast<const SelfCheck*>(
               &_SelfCheck_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(SelfCheck* other);
  friend void swap(SelfCheck& a, SelfCheck& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SelfCheck* New() const PROTOBUF_FINAL { return New(NULL); }

  SelfCheck* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SelfCheck& from);
  void MergeFrom(const SelfCheck& from);
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
  void InternalSwap(SelfCheck* other);
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

  // repeated .xsproto.communication.SelfCheckInfo check_detail = 2;
  int check_detail_size() const;
  void clear_check_detail();
  static const int kCheckDetailFieldNumber = 2;
  const ::xsproto::communication::SelfCheckInfo& check_detail(int index) const;
  ::xsproto::communication::SelfCheckInfo* mutable_check_detail(int index);
  ::xsproto::communication::SelfCheckInfo* add_check_detail();
  ::google::protobuf::RepeatedPtrField< ::xsproto::communication::SelfCheckInfo >*
      mutable_check_detail();
  const ::google::protobuf::RepeatedPtrField< ::xsproto::communication::SelfCheckInfo >&
      check_detail() const;

  // .xsproto.communication.SelfCheckState state = 1;
  void clear_state();
  static const int kStateFieldNumber = 1;
  ::xsproto::communication::SelfCheckState state() const;
  void set_state(::xsproto::communication::SelfCheckState value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.SelfCheck)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::xsproto::communication::SelfCheckInfo > check_detail_;
  int state_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fselfcheck_2eproto::TableStruct;
  friend void ::protobuf_business_2fselfcheck_2eproto::InitDefaultsSelfCheckImpl();
};
// -------------------------------------------------------------------

class SelfCheckInfo : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.SelfCheckInfo) */ {
 public:
  SelfCheckInfo();
  virtual ~SelfCheckInfo();

  SelfCheckInfo(const SelfCheckInfo& from);

  inline SelfCheckInfo& operator=(const SelfCheckInfo& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  SelfCheckInfo(SelfCheckInfo&& from) noexcept
    : SelfCheckInfo() {
    *this = ::std::move(from);
  }

  inline SelfCheckInfo& operator=(SelfCheckInfo&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const SelfCheckInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SelfCheckInfo* internal_default_instance() {
    return reinterpret_cast<const SelfCheckInfo*>(
               &_SelfCheckInfo_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(SelfCheckInfo* other);
  friend void swap(SelfCheckInfo& a, SelfCheckInfo& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline SelfCheckInfo* New() const PROTOBUF_FINAL { return New(NULL); }

  SelfCheckInfo* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SelfCheckInfo& from);
  void MergeFrom(const SelfCheckInfo& from);
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
  void InternalSwap(SelfCheckInfo* other);
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

  // string message = 2;
  void clear_message();
  static const int kMessageFieldNumber = 2;
  const ::std::string& message() const;
  void set_message(const ::std::string& value);
  #if LANG_CXX11
  void set_message(::std::string&& value);
  #endif
  void set_message(const char* value);
  void set_message(const char* value, size_t size);
  ::std::string* mutable_message();
  ::std::string* release_message();
  void set_allocated_message(::std::string* message);

  // int32 code = 1;
  void clear_code();
  static const int kCodeFieldNumber = 1;
  ::google::protobuf::int32 code() const;
  void set_code(::google::protobuf::int32 value);

  // .xsproto.communication.SelfCheckState state = 3;
  void clear_state();
  static const int kStateFieldNumber = 3;
  ::xsproto::communication::SelfCheckState state() const;
  void set_state(::xsproto::communication::SelfCheckState value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.SelfCheckInfo)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr message_;
  ::google::protobuf::int32 code_;
  int state_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fselfcheck_2eproto::TableStruct;
  friend void ::protobuf_business_2fselfcheck_2eproto::InitDefaultsSelfCheckInfoImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SelfCheck

// .xsproto.communication.SelfCheckState state = 1;
inline void SelfCheck::clear_state() {
  state_ = 0;
}
inline ::xsproto::communication::SelfCheckState SelfCheck::state() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SelfCheck.state)
  return static_cast< ::xsproto::communication::SelfCheckState >(state_);
}
inline void SelfCheck::set_state(::xsproto::communication::SelfCheckState value) {
  
  state_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.SelfCheck.state)
}

// repeated .xsproto.communication.SelfCheckInfo check_detail = 2;
inline int SelfCheck::check_detail_size() const {
  return check_detail_.size();
}
inline void SelfCheck::clear_check_detail() {
  check_detail_.Clear();
}
inline const ::xsproto::communication::SelfCheckInfo& SelfCheck::check_detail(int index) const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SelfCheck.check_detail)
  return check_detail_.Get(index);
}
inline ::xsproto::communication::SelfCheckInfo* SelfCheck::mutable_check_detail(int index) {
  // @@protoc_insertion_point(field_mutable:xsproto.communication.SelfCheck.check_detail)
  return check_detail_.Mutable(index);
}
inline ::xsproto::communication::SelfCheckInfo* SelfCheck::add_check_detail() {
  // @@protoc_insertion_point(field_add:xsproto.communication.SelfCheck.check_detail)
  return check_detail_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::xsproto::communication::SelfCheckInfo >*
SelfCheck::mutable_check_detail() {
  // @@protoc_insertion_point(field_mutable_list:xsproto.communication.SelfCheck.check_detail)
  return &check_detail_;
}
inline const ::google::protobuf::RepeatedPtrField< ::xsproto::communication::SelfCheckInfo >&
SelfCheck::check_detail() const {
  // @@protoc_insertion_point(field_list:xsproto.communication.SelfCheck.check_detail)
  return check_detail_;
}

// -------------------------------------------------------------------

// SelfCheckInfo

// int32 code = 1;
inline void SelfCheckInfo::clear_code() {
  code_ = 0;
}
inline ::google::protobuf::int32 SelfCheckInfo::code() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SelfCheckInfo.code)
  return code_;
}
inline void SelfCheckInfo::set_code(::google::protobuf::int32 value) {
  
  code_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.SelfCheckInfo.code)
}

// string message = 2;
inline void SelfCheckInfo::clear_message() {
  message_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& SelfCheckInfo::message() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SelfCheckInfo.message)
  return message_.GetNoArena();
}
inline void SelfCheckInfo::set_message(const ::std::string& value) {
  
  message_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.communication.SelfCheckInfo.message)
}
#if LANG_CXX11
inline void SelfCheckInfo::set_message(::std::string&& value) {
  
  message_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.communication.SelfCheckInfo.message)
}
#endif
inline void SelfCheckInfo::set_message(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  message_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.communication.SelfCheckInfo.message)
}
inline void SelfCheckInfo::set_message(const char* value, size_t size) {
  
  message_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.communication.SelfCheckInfo.message)
}
inline ::std::string* SelfCheckInfo::mutable_message() {
  
  // @@protoc_insertion_point(field_mutable:xsproto.communication.SelfCheckInfo.message)
  return message_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* SelfCheckInfo::release_message() {
  // @@protoc_insertion_point(field_release:xsproto.communication.SelfCheckInfo.message)
  
  return message_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void SelfCheckInfo::set_allocated_message(::std::string* message) {
  if (message != NULL) {
    
  } else {
    
  }
  message_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), message);
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.SelfCheckInfo.message)
}

// .xsproto.communication.SelfCheckState state = 3;
inline void SelfCheckInfo::clear_state() {
  state_ = 0;
}
inline ::xsproto::communication::SelfCheckState SelfCheckInfo::state() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.SelfCheckInfo.state)
  return static_cast< ::xsproto::communication::SelfCheckState >(state_);
}
inline void SelfCheckInfo::set_state(::xsproto::communication::SelfCheckState value) {
  
  state_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.SelfCheckInfo.state)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::xsproto::communication::SelfCheckState> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::xsproto::communication::SelfCheckState>() {
  return ::xsproto::communication::SelfCheckState_descriptor();
}

}  // namespace protobuf
}  // namespace google

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fselfcheck_2eproto__INCLUDED
