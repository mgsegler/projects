// Generated by gencpp from file bebop_msgs/CommonCommonStateMassStorageContent.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEMASSSTORAGECONTENT_H
#define BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEMASSSTORAGECONTENT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace bebop_msgs
{
template <class ContainerAllocator>
struct CommonCommonStateMassStorageContent_
{
  typedef CommonCommonStateMassStorageContent_<ContainerAllocator> Type;

  CommonCommonStateMassStorageContent_()
    : header()
    , mass_storage_id(0)
    , nbPhotos(0)
    , nbVideos(0)
    , nbPuds(0)
    , nbCrashLogs(0)
    , nbRawPhotos(0)  {
    }
  CommonCommonStateMassStorageContent_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , mass_storage_id(0)
    , nbPhotos(0)
    , nbVideos(0)
    , nbPuds(0)
    , nbCrashLogs(0)
    , nbRawPhotos(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _mass_storage_id_type;
  _mass_storage_id_type mass_storage_id;

   typedef uint16_t _nbPhotos_type;
  _nbPhotos_type nbPhotos;

   typedef uint16_t _nbVideos_type;
  _nbVideos_type nbVideos;

   typedef uint16_t _nbPuds_type;
  _nbPuds_type nbPuds;

   typedef uint16_t _nbCrashLogs_type;
  _nbCrashLogs_type nbCrashLogs;

   typedef uint16_t _nbRawPhotos_type;
  _nbRawPhotos_type nbRawPhotos;





  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> const> ConstPtr;

}; // struct CommonCommonStateMassStorageContent_

typedef ::bebop_msgs::CommonCommonStateMassStorageContent_<std::allocator<void> > CommonCommonStateMassStorageContent;

typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateMassStorageContent > CommonCommonStateMassStorageContentPtr;
typedef boost::shared_ptr< ::bebop_msgs::CommonCommonStateMassStorageContent const> CommonCommonStateMassStorageContentConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace bebop_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'bebop_msgs': ['/home/leader/bebop_ws/src/bebop_autonomy/bebop_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a6f27bb44bd06f7e0a5be4f3c064ceea";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa6f27bb44bd06f7eULL;
  static const uint64_t static_value2 = 0x0a5be4f3c064ceeaULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/CommonCommonStateMassStorageContent";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# CommonCommonStateMassStorageContent\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Mass storage content.\n\
\n\
Header header\n\
\n\
# Mass storage id (unique)\n\
uint8 mass_storage_id\n\
# Number of photos (does not include raw photos)\n\
uint16 nbPhotos\n\
# Number of videos\n\
uint16 nbVideos\n\
# Number of puds\n\
uint16 nbPuds\n\
# Number of crash logs\n\
uint16 nbCrashLogs\n\
# Number of raw photos\n\
uint16 nbRawPhotos\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.mass_storage_id);
      stream.next(m.nbPhotos);
      stream.next(m.nbVideos);
      stream.next(m.nbPuds);
      stream.next(m.nbCrashLogs);
      stream.next(m.nbRawPhotos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CommonCommonStateMassStorageContent_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::CommonCommonStateMassStorageContent_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "mass_storage_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.mass_storage_id);
    s << indent << "nbPhotos: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.nbPhotos);
    s << indent << "nbVideos: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.nbVideos);
    s << indent << "nbPuds: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.nbPuds);
    s << indent << "nbCrashLogs: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.nbCrashLogs);
    s << indent << "nbRawPhotos: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.nbRawPhotos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_COMMONCOMMONSTATEMASSSTORAGECONTENT_H
