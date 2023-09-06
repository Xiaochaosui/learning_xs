// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: business/ci_common.proto

#ifndef PROTOBUF_business_2fci_5fcommon_2eproto__INCLUDED
#define PROTOBUF_business_2fci_5fcommon_2eproto__INCLUDED

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

namespace protobuf_business_2fci_5fcommon_2eproto {
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
void InitDefaultsPointGPSImpl();
void InitDefaultsPointGPS();
void InitDefaultsPointGaussImpl();
void InitDefaultsPointGauss();
void InitDefaultsPointImpl();
void InitDefaultsPoint();
inline void InitDefaults() {
  InitDefaultsPointGPS();
  InitDefaultsPointGauss();
  InitDefaultsPoint();
}
}  // namespace protobuf_business_2fci_5fcommon_2eproto
namespace xsproto {
namespace communication {
class Point;
class PointDefaultTypeInternal;
extern PointDefaultTypeInternal _Point_default_instance_;
class PointGPS;
class PointGPSDefaultTypeInternal;
extern PointGPSDefaultTypeInternal _PointGPS_default_instance_;
class PointGauss;
class PointGaussDefaultTypeInternal;
extern PointGaussDefaultTypeInternal _PointGauss_default_instance_;
}  // namespace communication
}  // namespace xsproto
namespace xsproto {
namespace communication {

// ===================================================================

class PointGPS : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.PointGPS) */ {
 public:
  PointGPS();
  virtual ~PointGPS();

  PointGPS(const PointGPS& from);

  inline PointGPS& operator=(const PointGPS& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PointGPS(PointGPS&& from) noexcept
    : PointGPS() {
    *this = ::std::move(from);
  }

  inline PointGPS& operator=(PointGPS&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const PointGPS& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PointGPS* internal_default_instance() {
    return reinterpret_cast<const PointGPS*>(
               &_PointGPS_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(PointGPS* other);
  friend void swap(PointGPS& a, PointGPS& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PointGPS* New() const PROTOBUF_FINAL { return New(NULL); }

  PointGPS* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const PointGPS& from);
  void MergeFrom(const PointGPS& from);
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
  void InternalSwap(PointGPS* other);
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

  // double longitude = 1;
  void clear_longitude();
  static const int kLongitudeFieldNumber = 1;
  double longitude() const;
  void set_longitude(double value);

  // double latitude = 2;
  void clear_latitude();
  static const int kLatitudeFieldNumber = 2;
  double latitude() const;
  void set_latitude(double value);

  // double azimuth = 3;
  void clear_azimuth();
  static const int kAzimuthFieldNumber = 3;
  double azimuth() const;
  void set_azimuth(double value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.PointGPS)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double longitude_;
  double latitude_;
  double azimuth_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fci_5fcommon_2eproto::TableStruct;
  friend void ::protobuf_business_2fci_5fcommon_2eproto::InitDefaultsPointGPSImpl();
};
// -------------------------------------------------------------------

class PointGauss : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.PointGauss) */ {
 public:
  PointGauss();
  virtual ~PointGauss();

  PointGauss(const PointGauss& from);

  inline PointGauss& operator=(const PointGauss& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  PointGauss(PointGauss&& from) noexcept
    : PointGauss() {
    *this = ::std::move(from);
  }

  inline PointGauss& operator=(PointGauss&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const PointGauss& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const PointGauss* internal_default_instance() {
    return reinterpret_cast<const PointGauss*>(
               &_PointGauss_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(PointGauss* other);
  friend void swap(PointGauss& a, PointGauss& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline PointGauss* New() const PROTOBUF_FINAL { return New(NULL); }

  PointGauss* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const PointGauss& from);
  void MergeFrom(const PointGauss& from);
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
  void InternalSwap(PointGauss* other);
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

  // double gauss_x = 1;
  void clear_gauss_x();
  static const int kGaussXFieldNumber = 1;
  double gauss_x() const;
  void set_gauss_x(double value);

  // double gauss_y = 2;
  void clear_gauss_y();
  static const int kGaussYFieldNumber = 2;
  double gauss_y() const;
  void set_gauss_y(double value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.PointGauss)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double gauss_x_;
  double gauss_y_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fci_5fcommon_2eproto::TableStruct;
  friend void ::protobuf_business_2fci_5fcommon_2eproto::InitDefaultsPointGaussImpl();
};
// -------------------------------------------------------------------

class Point : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:xsproto.communication.Point) */ {
 public:
  Point();
  virtual ~Point();

  Point(const Point& from);

  inline Point& operator=(const Point& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Point(Point&& from) noexcept
    : Point() {
    *this = ::std::move(from);
  }

  inline Point& operator=(Point&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Point& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Point* internal_default_instance() {
    return reinterpret_cast<const Point*>(
               &_Point_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(Point* other);
  friend void swap(Point& a, Point& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Point* New() const PROTOBUF_FINAL { return New(NULL); }

  Point* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Point& from);
  void MergeFrom(const Point& from);
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
  void InternalSwap(Point* other);
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

  // double x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // double y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // double yaw = 3;
  void clear_yaw();
  static const int kYawFieldNumber = 3;
  double yaw() const;
  void set_yaw(double value);

  // @@protoc_insertion_point(class_scope:xsproto.communication.Point)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double x_;
  double y_;
  double yaw_;
  mutable int _cached_size_;
  friend struct ::protobuf_business_2fci_5fcommon_2eproto::TableStruct;
  friend void ::protobuf_business_2fci_5fcommon_2eproto::InitDefaultsPointImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// PointGPS

// double longitude = 1;
inline void PointGPS::clear_longitude() {
  longitude_ = 0;
}
inline double PointGPS::longitude() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.PointGPS.longitude)
  return longitude_;
}
inline void PointGPS::set_longitude(double value) {
  
  longitude_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.PointGPS.longitude)
}

// double latitude = 2;
inline void PointGPS::clear_latitude() {
  latitude_ = 0;
}
inline double PointGPS::latitude() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.PointGPS.latitude)
  return latitude_;
}
inline void PointGPS::set_latitude(double value) {
  
  latitude_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.PointGPS.latitude)
}

// double azimuth = 3;
inline void PointGPS::clear_azimuth() {
  azimuth_ = 0;
}
inline double PointGPS::azimuth() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.PointGPS.azimuth)
  return azimuth_;
}
inline void PointGPS::set_azimuth(double value) {
  
  azimuth_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.PointGPS.azimuth)
}

// -------------------------------------------------------------------

// PointGauss

// double gauss_x = 1;
inline void PointGauss::clear_gauss_x() {
  gauss_x_ = 0;
}
inline double PointGauss::gauss_x() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.PointGauss.gauss_x)
  return gauss_x_;
}
inline void PointGauss::set_gauss_x(double value) {
  
  gauss_x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.PointGauss.gauss_x)
}

// double gauss_y = 2;
inline void PointGauss::clear_gauss_y() {
  gauss_y_ = 0;
}
inline double PointGauss::gauss_y() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.PointGauss.gauss_y)
  return gauss_y_;
}
inline void PointGauss::set_gauss_y(double value) {
  
  gauss_y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.PointGauss.gauss_y)
}

// -------------------------------------------------------------------

// Point

// double x = 1;
inline void Point::clear_x() {
  x_ = 0;
}
inline double Point::x() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.Point.x)
  return x_;
}
inline void Point::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.Point.x)
}

// double y = 2;
inline void Point::clear_y() {
  y_ = 0;
}
inline double Point::y() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.Point.y)
  return y_;
}
inline void Point::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.Point.y)
}

// double yaw = 3;
inline void Point::clear_yaw() {
  yaw_ = 0;
}
inline double Point::yaw() const {
  // @@protoc_insertion_point(field_get:xsproto.communication.Point.yaw)
  return yaw_;
}
inline void Point::set_yaw(double value) {
  
  yaw_ = value;
  // @@protoc_insertion_point(field_set:xsproto.communication.Point.yaw)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace communication
}  // namespace xsproto

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_business_2fci_5fcommon_2eproto__INCLUDED