// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: hdmap/obstacle_map.proto

#ifndef PROTOBUF_hdmap_2fobstacle_5fmap_2eproto__INCLUDED
#define PROTOBUF_hdmap_2fobstacle_5fmap_2eproto__INCLUDED

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

namespace protobuf_hdmap_2fobstacle_5fmap_2eproto {
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
void InitDefaultsObstacleMapImpl();
void InitDefaultsObstacleMap();
inline void InitDefaults() {
  InitDefaultsObstacleMap();
}
}  // namespace protobuf_hdmap_2fobstacle_5fmap_2eproto
namespace xsproto {
namespace hdmap {
class ObstacleMap;
class ObstacleMapDefaultTypeInternal;
extern ObstacleMapDefaultTypeInternal _ObstacleMap_default_instance_;
}  // namespace hdmap
}  // namespace xsproto
namespace xsproto {
namespace hdmap {

// ===================================================================

class ObstacleMap : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.hdmap.ObstacleMap) */ {
 public:
  ObstacleMap();
  virtual ~ObstacleMap();

  ObstacleMap(const ObstacleMap& from);

  inline ObstacleMap& operator=(const ObstacleMap& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ObstacleMap(ObstacleMap&& from) noexcept
    : ObstacleMap() {
    *this = ::std::move(from);
  }

  inline ObstacleMap& operator=(ObstacleMap&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const ObstacleMap& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ObstacleMap* internal_default_instance() {
    return reinterpret_cast<const ObstacleMap*>(
               &_ObstacleMap_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ObstacleMap* other);
  friend void swap(ObstacleMap& a, ObstacleMap& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ObstacleMap* New() const PROTOBUF_FINAL { return New(NULL); }

  ObstacleMap* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ObstacleMap& from);
  void MergeFrom(const ObstacleMap& from);
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
  void InternalSwap(ObstacleMap* other);
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

  // optional bytes obstacleMap_2cm = 2;
  bool has_obstaclemap_2cm() const;
  void clear_obstaclemap_2cm();
  static const int kObstacleMap2CmFieldNumber = 2;
  const ::std::string& obstaclemap_2cm() const;
  void set_obstaclemap_2cm(const ::std::string& value);
  #if LANG_CXX11
  void set_obstaclemap_2cm(::std::string&& value);
  #endif
  void set_obstaclemap_2cm(const char* value);
  void set_obstaclemap_2cm(const void* value, size_t size);
  ::std::string* mutable_obstaclemap_2cm();
  ::std::string* release_obstaclemap_2cm();
  void set_allocated_obstaclemap_2cm(::std::string* obstaclemap_2cm);

  // optional bytes obstacleMap_10cm = 3;
  bool has_obstaclemap_10cm() const;
  void clear_obstaclemap_10cm();
  static const int kObstacleMap10CmFieldNumber = 3;
  const ::std::string& obstaclemap_10cm() const;
  void set_obstaclemap_10cm(const ::std::string& value);
  #if LANG_CXX11
  void set_obstaclemap_10cm(::std::string&& value);
  #endif
  void set_obstaclemap_10cm(const char* value);
  void set_obstaclemap_10cm(const void* value, size_t size);
  ::std::string* mutable_obstaclemap_10cm();
  ::std::string* release_obstaclemap_10cm();
  void set_allocated_obstaclemap_10cm(::std::string* obstaclemap_10cm);

  // optional .xsproto.base.Header header = 1;
  bool has_header() const;
  void clear_header();
  static const int kHeaderFieldNumber = 1;
  const ::xsproto::base::Header& header() const;
  ::xsproto::base::Header* release_header();
  ::xsproto::base::Header* mutable_header();
  void set_allocated_header(::xsproto::base::Header* header);

  // @@protoc_insertion_point(class_scope:xsproto.hdmap.ObstacleMap)
 private:
  void set_has_header();
  void clear_has_header();
  void set_has_obstaclemap_2cm();
  void clear_has_obstaclemap_2cm();
  void set_has_obstaclemap_10cm();
  void clear_has_obstaclemap_10cm();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::google::protobuf::internal::ArenaStringPtr obstaclemap_2cm_;
  ::google::protobuf::internal::ArenaStringPtr obstaclemap_10cm_;
  ::xsproto::base::Header* header_;
  friend struct ::protobuf_hdmap_2fobstacle_5fmap_2eproto::TableStruct;
  friend void ::protobuf_hdmap_2fobstacle_5fmap_2eproto::InitDefaultsObstacleMapImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ObstacleMap

// optional .xsproto.base.Header header = 1;
inline bool ObstacleMap::has_header() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ObstacleMap::set_has_header() {
  _has_bits_[0] |= 0x00000004u;
}
inline void ObstacleMap::clear_has_header() {
  _has_bits_[0] &= ~0x00000004u;
}
inline const ::xsproto::base::Header& ObstacleMap::header() const {
  const ::xsproto::base::Header* p = header_;
  // @@protoc_insertion_point(field_get:xsproto.hdmap.ObstacleMap.header)
  return p != NULL ? *p : *reinterpret_cast<const ::xsproto::base::Header*>(
      &::xsproto::base::_Header_default_instance_);
}
inline ::xsproto::base::Header* ObstacleMap::release_header() {
  // @@protoc_insertion_point(field_release:xsproto.hdmap.ObstacleMap.header)
  clear_has_header();
  ::xsproto::base::Header* temp = header_;
  header_ = NULL;
  return temp;
}
inline ::xsproto::base::Header* ObstacleMap::mutable_header() {
  set_has_header();
  if (header_ == NULL) {
    header_ = new ::xsproto::base::Header;
  }
  // @@protoc_insertion_point(field_mutable:xsproto.hdmap.ObstacleMap.header)
  return header_;
}
inline void ObstacleMap::set_allocated_header(::xsproto::base::Header* header) {
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
    set_has_header();
  } else {
    clear_has_header();
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:xsproto.hdmap.ObstacleMap.header)
}

// optional bytes obstacleMap_2cm = 2;
inline bool ObstacleMap::has_obstaclemap_2cm() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ObstacleMap::set_has_obstaclemap_2cm() {
  _has_bits_[0] |= 0x00000001u;
}
inline void ObstacleMap::clear_has_obstaclemap_2cm() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void ObstacleMap::clear_obstaclemap_2cm() {
  obstaclemap_2cm_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_obstaclemap_2cm();
}
inline const ::std::string& ObstacleMap::obstaclemap_2cm() const {
  // @@protoc_insertion_point(field_get:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
  return obstaclemap_2cm_.GetNoArena();
}
inline void ObstacleMap::set_obstaclemap_2cm(const ::std::string& value) {
  set_has_obstaclemap_2cm();
  obstaclemap_2cm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
}
#if LANG_CXX11
inline void ObstacleMap::set_obstaclemap_2cm(::std::string&& value) {
  set_has_obstaclemap_2cm();
  obstaclemap_2cm_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
}
#endif
inline void ObstacleMap::set_obstaclemap_2cm(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_obstaclemap_2cm();
  obstaclemap_2cm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
}
inline void ObstacleMap::set_obstaclemap_2cm(const void* value, size_t size) {
  set_has_obstaclemap_2cm();
  obstaclemap_2cm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
}
inline ::std::string* ObstacleMap::mutable_obstaclemap_2cm() {
  set_has_obstaclemap_2cm();
  // @@protoc_insertion_point(field_mutable:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
  return obstaclemap_2cm_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* ObstacleMap::release_obstaclemap_2cm() {
  // @@protoc_insertion_point(field_release:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
  clear_has_obstaclemap_2cm();
  return obstaclemap_2cm_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void ObstacleMap::set_allocated_obstaclemap_2cm(::std::string* obstaclemap_2cm) {
  if (obstaclemap_2cm != NULL) {
    set_has_obstaclemap_2cm();
  } else {
    clear_has_obstaclemap_2cm();
  }
  obstaclemap_2cm_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), obstaclemap_2cm);
  // @@protoc_insertion_point(field_set_allocated:xsproto.hdmap.ObstacleMap.obstacleMap_2cm)
}

// optional bytes obstacleMap_10cm = 3;
inline bool ObstacleMap::has_obstaclemap_10cm() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ObstacleMap::set_has_obstaclemap_10cm() {
  _has_bits_[0] |= 0x00000002u;
}
inline void ObstacleMap::clear_has_obstaclemap_10cm() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void ObstacleMap::clear_obstaclemap_10cm() {
  obstaclemap_10cm_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
  clear_has_obstaclemap_10cm();
}
inline const ::std::string& ObstacleMap::obstaclemap_10cm() const {
  // @@protoc_insertion_point(field_get:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
  return obstaclemap_10cm_.GetNoArena();
}
inline void ObstacleMap::set_obstaclemap_10cm(const ::std::string& value) {
  set_has_obstaclemap_10cm();
  obstaclemap_10cm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
}
#if LANG_CXX11
inline void ObstacleMap::set_obstaclemap_10cm(::std::string&& value) {
  set_has_obstaclemap_10cm();
  obstaclemap_10cm_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
}
#endif
inline void ObstacleMap::set_obstaclemap_10cm(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  set_has_obstaclemap_10cm();
  obstaclemap_10cm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
}
inline void ObstacleMap::set_obstaclemap_10cm(const void* value, size_t size) {
  set_has_obstaclemap_10cm();
  obstaclemap_10cm_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
}
inline ::std::string* ObstacleMap::mutable_obstaclemap_10cm() {
  set_has_obstaclemap_10cm();
  // @@protoc_insertion_point(field_mutable:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
  return obstaclemap_10cm_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* ObstacleMap::release_obstaclemap_10cm() {
  // @@protoc_insertion_point(field_release:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
  clear_has_obstaclemap_10cm();
  return obstaclemap_10cm_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void ObstacleMap::set_allocated_obstaclemap_10cm(::std::string* obstaclemap_10cm) {
  if (obstaclemap_10cm != NULL) {
    set_has_obstaclemap_10cm();
  } else {
    clear_has_obstaclemap_10cm();
  }
  obstaclemap_10cm_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), obstaclemap_10cm);
  // @@protoc_insertion_point(field_set_allocated:xsproto.hdmap.ObstacleMap.obstacleMap_10cm)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_hdmap_2fobstacle_5fmap_2eproto__INCLUDED
