// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/motion_cmd.proto

#ifndef PROTOBUF_business_2fmotion_5fcmd_2eproto__INCLUDED
#define PROTOBUF_business_2fmotion_5fcmd_2eproto__INCLUDED

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

namespace protobuf_business_2fmotion_5fcmd_2eproto {
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
void InitDefaultsMotionCmdImpl();
void InitDefaultsMotionCmd();
void InitDefaultsControlMsgImpl();
void InitDefaultsControlMsg();
inline void InitDefaults() {
  InitDefaultsMotionCmd();
  InitDefaultsControlMsg();
}
}  // namespace protobuf_business_2fmotion_5fcmd_2eproto
namespace xsproto {
namespace communication {
class ControlMsg;
class ControlMsgDefaultTypeInternal;
extern ControlMsgDefaultTypeInternal _ControlMsg_default_instance_;
class MotionCmd;
class MotionCmdDefaultTypeInternal;
extern MotionCmdDefaultTypeInternal _MotionCmd_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class MotionCmd : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.MotionCmd) */ {
 public:
  MotionCmd();
  virtual ~MotionCmd();

  MotionCmd(const MotionCmd& from);

  inline MotionCmd& operator=(const MotionCmd& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  MotionCmd(MotionCmd&& from) noexcept
    : MotionCmd() {
    *this = ::std::move(from);
  }

  inline MotionCmd& operator=(MotionCmd&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const MotionCmd& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const MotionCmd* internal_default_instance() {
    return reinterpret_cast<const MotionCmd*>(
               &_MotionCmd_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(MotionCmd* other);
  friend void swap(MotionCmd& a, MotionCmd& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline MotionCmd* New() const PROTOBUF_FINAL { return New(NULL); }

  MotionCmd* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const MotionCmd& from);
  void MergeFrom(const MotionCmd& from);
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
  void InternalSwap(MotionCmd* other);
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

  // .google.protobuf.Any data = 5;
  bool has_data() const;
  void clear_data();
  static const int kDataFieldNumber = 5;
  const ::google::protobuf::Any& data() const;
  ::google::protobuf::Any* release_data();
  ::google::protobuf::Any* mutable_data();
  void set_allocated_data(::google::protobuf::Any* data);

  // int32 tag = 2;
  void clear_tag();
  static const int kTagFieldNumber = 2;
  ::google::protobuf::int32 tag() const;
  void set_tag(::google::protobuf::int32 value);

  // int32 version = 3;
  void clear_version();
  static const int kVersionFieldNumber = 3;
  ::google::protobuf::int32 version() const;
  void set_version(::google::protobuf::int32 value);

  // int32 valid = 4;
  void clear_valid();
  static const int kValidFieldNumber = 4;
  ::google::protobuf::int32 valid() const;
  void set_valid(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.MotionCmd)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::xsproto::base::Header* header_;
  ::google::protobuf::Any* data_;
  ::google::protobuf::int32 tag_;
  ::google::protobuf::int32 version_;
  ::google::protobuf::int32 valid_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fmotion_5fcmd_2eproto::TableStruct;
  friend void ::protobuf_business_2fmotion_5fcmd_2eproto::InitDefaultsMotionCmdImpl();
};
// -------------------------------------------------------------------

class ControlMsg : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.ControlMsg) */ {
 public:
  ControlMsg();
  virtual ~ControlMsg();

  ControlMsg(const ControlMsg& from);

  inline ControlMsg& operator=(const ControlMsg& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ControlMsg(ControlMsg&& from) noexcept
    : ControlMsg() {
    *this = ::std::move(from);
  }

  inline ControlMsg& operator=(ControlMsg&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const ControlMsg& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ControlMsg* internal_default_instance() {
    return reinterpret_cast<const ControlMsg*>(
               &_ControlMsg_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(ControlMsg* other);
  friend void swap(ControlMsg& a, ControlMsg& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ControlMsg* New() const PROTOBUF_FINAL { return New(NULL); }

  ControlMsg* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ControlMsg& from);
  void MergeFrom(const ControlMsg& from);
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
  void InternalSwap(ControlMsg* other);
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

  // int64 video_timestamp = 1;
  void clear_video_timestamp();
  static const int kVideoTimestampFieldNumber = 1;
  ::google::protobuf::int64 video_timestamp() const;
  void set_video_timestamp(::google::protobuf::int64 value);

  // int64 control_timestamp = 2;
  void clear_control_timestamp();
  static const int kControlTimestampFieldNumber = 2;
  ::google::protobuf::int64 control_timestamp() const;
  void set_control_timestamp(::google::protobuf::int64 value);

  // int32 steer_angle = 3;
  void clear_steer_angle();
  static const int kSteerAngleFieldNumber = 3;
  ::google::protobuf::int32 steer_angle() const;
  void set_steer_angle(::google::protobuf::int32 value);

  // int32 speed = 4;
  void clear_speed();
  static const int kSpeedFieldNumber = 4;
  ::google::protobuf::int32 speed() const;
  void set_speed(::google::protobuf::int32 value);

  // int32 curve = 5;
  void clear_curve();
  static const int kCurveFieldNumber = 5;
  ::google::protobuf::int32 curve() const;
  void set_curve(::google::protobuf::int32 value);

  // int32 drive_mode = 6;
  void clear_drive_mode();
  static const int kDriveModeFieldNumber = 6;
  ::google::protobuf::int32 drive_mode() const;
  void set_drive_mode(::google::protobuf::int32 value);

  // int32 handbrake_status = 7;
  void clear_handbrake_status();
  static const int kHandbrakeStatusFieldNumber = 7;
  ::google::protobuf::int32 handbrake_status() const;
  void set_handbrake_status(::google::protobuf::int32 value);

  // int32 oil = 8;
  void clear_oil();
  static const int kOilFieldNumber = 8;
  ::google::protobuf::int32 oil() const;
  void set_oil(::google::protobuf::int32 value);

  // int32 brake = 9;
  void clear_brake();
  static const int kBrakeFieldNumber = 9;
  ::google::protobuf::int32 brake() const;
  void set_brake(::google::protobuf::int32 value);

  // int32 shift = 10;
  void clear_shift();
  static const int kShiftFieldNumber = 10;
  ::google::protobuf::int32 shift() const;
  void set_shift(::google::protobuf::int32 value);

  // int32 engine_status = 11;
  void clear_engine_status();
  static const int kEngineStatusFieldNumber = 11;
  ::google::protobuf::int32 engine_status() const;
  void set_engine_status(::google::protobuf::int32 value);

  // int32 light = 12;
  void clear_light();
  static const int kLightFieldNumber = 12;
  ::google::protobuf::int32 light() const;
  void set_light(::google::protobuf::int32 value);

  // int32 avoid_type = 13;
  void clear_avoid_type();
  static const int kAvoidTypeFieldNumber = 13;
  ::google::protobuf::int32 avoid_type() const;
  void set_avoid_type(::google::protobuf::int32 value);

  // int32 urgent_stop = 14;
  void clear_urgent_stop();
  static const int kUrgentStopFieldNumber = 14;
  ::google::protobuf::int32 urgent_stop() const;
  void set_urgent_stop(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.ControlMsg)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int64 video_timestamp_;
  ::google::protobuf::int64 control_timestamp_;
  ::google::protobuf::int32 steer_angle_;
  ::google::protobuf::int32 speed_;
  ::google::protobuf::int32 curve_;
  ::google::protobuf::int32 drive_mode_;
  ::google::protobuf::int32 handbrake_status_;
  ::google::protobuf::int32 oil_;
  ::google::protobuf::int32 brake_;
  ::google::protobuf::int32 shift_;
  ::google::protobuf::int32 engine_status_;
  ::google::protobuf::int32 light_;
  ::google::protobuf::int32 avoid_type_;
  ::google::protobuf::int32 urgent_stop_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fmotion_5fcmd_2eproto::TableStruct;
  friend void ::protobuf_business_2fmotion_5fcmd_2eproto::InitDefaultsControlMsgImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// MotionCmd

// .xsproto.base.Header header = 1;
inline bool MotionCmd::has_header() const {
  return this != internal_default_instance() && header_ != NULL;
}
inline const ::xsproto::base::Header& MotionCmd::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.communication.MotionCmd.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* MotionCmd::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MotionCmd.header)
  
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* MotionCmd::mutable_header() {
  
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MotionCmd.header)
  return header_;
}
inline void MotionCmd::set_allocated_header(::xsproto::base::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MotionCmd.header)
}

// int32 tag = 2;
inline void MotionCmd::clear_tag() {
  tag_ = 0;
}
inline ::google::protobuf::int32 MotionCmd::tag() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MotionCmd.tag)
  return tag_;
}
inline void MotionCmd::set_tag(::google::protobuf::int32 value) {
  
  tag_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MotionCmd.tag)
}

// int32 version = 3;
inline void MotionCmd::clear_version() {
  version_ = 0;
}
inline ::google::protobuf::int32 MotionCmd::version() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MotionCmd.version)
  return version_;
}
inline void MotionCmd::set_version(::google::protobuf::int32 value) {
  
  version_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MotionCmd.version)
}

// int32 valid = 4;
inline void MotionCmd::clear_valid() {
  valid_ = 0;
}
inline ::google::protobuf::int32 MotionCmd::valid() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.MotionCmd.valid)
  return valid_;
}
inline void MotionCmd::set_valid(::google::protobuf::int32 value) {
  
  valid_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.MotionCmd.valid)
}

// .google.protobuf.Any data = 5;
inline bool MotionCmd::has_data() const {
  return this != internal_default_instance() && data_ != NULL;
}
inline const ::google::protobuf::Any& MotionCmd::data() const {
  const ::google::protobuf::Any* p = data_;
  // @@protoc_insertion_point(field_get:xsproto.communication.MotionCmd.data)
  return p != NULL ? *p : *reinterpret_cast<const ::google::protobuf::Any*>(
      &::google::protobuf::_Any_default_instance_);
}
inline ::google::protobuf::Any* MotionCmd::release_data() {
  // @@protoc_insertion_point(field_release:xsproto.communication.MotionCmd.data)
  
  ::google::protobuf::Any* temp = data_;
  data_ = NULL;
  return temp;
}
inline ::google::protobuf::Any* MotionCmd::mutable_data() {
  
  if (data_ == NULL) {
    data_ = new ::google::protobuf::Any;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.communication.MotionCmd.data)
  return data_;
}
inline void MotionCmd::set_allocated_data(::google::protobuf::Any* data) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(data_);
  }
  if (data) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      data = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, data, submessage_arena);
    }
    
  } else {
    
  }
  data_ = data;
  // @@protoc_insertion_point(field_set_allocated:xsproto.communication.MotionCmd.data)
}

// -------------------------------------------------------------------

// ControlMsg

// int64 video_timestamp = 1;
inline void ControlMsg::clear_video_timestamp() {
  video_timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 ControlMsg::video_timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.video_timestamp)
  return video_timestamp_;
}
inline void ControlMsg::set_video_timestamp(::google::protobuf::int64 value) {
  
  video_timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.video_timestamp)
}

// int64 control_timestamp = 2;
inline void ControlMsg::clear_control_timestamp() {
  control_timestamp_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 ControlMsg::control_timestamp() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.control_timestamp)
  return control_timestamp_;
}
inline void ControlMsg::set_control_timestamp(::google::protobuf::int64 value) {
  
  control_timestamp_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.control_timestamp)
}

// int32 steer_angle = 3;
inline void ControlMsg::clear_steer_angle() {
  steer_angle_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::steer_angle() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.steer_angle)
  return steer_angle_;
}
inline void ControlMsg::set_steer_angle(::google::protobuf::int32 value) {
  
  steer_angle_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.steer_angle)
}

// int32 speed = 4;
inline void ControlMsg::clear_speed() {
  speed_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::speed() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.speed)
  return speed_;
}
inline void ControlMsg::set_speed(::google::protobuf::int32 value) {
  
  speed_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.speed)
}

// int32 curve = 5;
inline void ControlMsg::clear_curve() {
  curve_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::curve() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.curve)
  return curve_;
}
inline void ControlMsg::set_curve(::google::protobuf::int32 value) {
  
  curve_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.curve)
}

// int32 drive_mode = 6;
inline void ControlMsg::clear_drive_mode() {
  drive_mode_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::drive_mode() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.drive_mode)
  return drive_mode_;
}
inline void ControlMsg::set_drive_mode(::google::protobuf::int32 value) {
  
  drive_mode_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.drive_mode)
}

// int32 handbrake_status = 7;
inline void ControlMsg::clear_handbrake_status() {
  handbrake_status_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::handbrake_status() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.handbrake_status)
  return handbrake_status_;
}
inline void ControlMsg::set_handbrake_status(::google::protobuf::int32 value) {
  
  handbrake_status_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.handbrake_status)
}

// int32 oil = 8;
inline void ControlMsg::clear_oil() {
  oil_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::oil() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.oil)
  return oil_;
}
inline void ControlMsg::set_oil(::google::protobuf::int32 value) {
  
  oil_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.oil)
}

// int32 brake = 9;
inline void ControlMsg::clear_brake() {
  brake_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::brake() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.brake)
  return brake_;
}
inline void ControlMsg::set_brake(::google::protobuf::int32 value) {
  
  brake_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.brake)
}

// int32 shift = 10;
inline void ControlMsg::clear_shift() {
  shift_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::shift() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.shift)
  return shift_;
}
inline void ControlMsg::set_shift(::google::protobuf::int32 value) {
  
  shift_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.shift)
}

// int32 engine_status = 11;
inline void ControlMsg::clear_engine_status() {
  engine_status_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::engine_status() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.engine_status)
  return engine_status_;
}
inline void ControlMsg::set_engine_status(::google::protobuf::int32 value) {
  
  engine_status_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.engine_status)
}

// int32 light = 12;
inline void ControlMsg::clear_light() {
  light_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::light() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.light)
  return light_;
}
inline void ControlMsg::set_light(::google::protobuf::int32 value) {
  
  light_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.light)
}

// int32 avoid_type = 13;
inline void ControlMsg::clear_avoid_type() {
  avoid_type_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::avoid_type() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.avoid_type)
  return avoid_type_;
}
inline void ControlMsg::set_avoid_type(::google::protobuf::int32 value) {
  
  avoid_type_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.avoid_type)
}

// int32 urgent_stop = 14;
inline void ControlMsg::clear_urgent_stop() {
  urgent_stop_ = 0;
}
inline ::google::protobuf::int32 ControlMsg::urgent_stop() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.ControlMsg.urgent_stop)
  return urgent_stop_;
}
inline void ControlMsg::set_urgent_stop(::google::protobuf::int32 value) {
  
  urgent_stop_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.ControlMsg.urgent_stop)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fmotion_5fcmd_2eproto__INCLUDED
