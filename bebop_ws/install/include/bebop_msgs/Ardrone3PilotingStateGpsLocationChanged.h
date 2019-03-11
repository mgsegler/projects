// Generated by gencpp from file bebop_msgs/Ardrone3PilotingStateGpsLocationChanged.msg
// DO NOT EDIT!


#ifndef BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEGPSLOCATIONCHANGED_H
#define BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEGPSLOCATIONCHANGED_H


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
struct Ardrone3PilotingStateGpsLocationChanged_
{
  typedef Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> Type;

  Ardrone3PilotingStateGpsLocationChanged_()
    : header()
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , latitude_accuracy(0)
    , longitude_accuracy(0)
    , altitude_accuracy(0)  {
    }
  Ardrone3PilotingStateGpsLocationChanged_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)
    , latitude_accuracy(0)
    , longitude_accuracy(0)
    , altitude_accuracy(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef double _altitude_type;
  _altitude_type altitude;

   typedef int8_t _latitude_accuracy_type;
  _latitude_accuracy_type latitude_accuracy;

   typedef int8_t _longitude_accuracy_type;
  _longitude_accuracy_type longitude_accuracy;

   typedef int8_t _altitude_accuracy_type;
  _altitude_accuracy_type altitude_accuracy;





  typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> const> ConstPtr;

}; // struct Ardrone3PilotingStateGpsLocationChanged_

typedef ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<std::allocator<void> > Ardrone3PilotingStateGpsLocationChanged;

typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged > Ardrone3PilotingStateGpsLocationChangedPtr;
typedef boost::shared_ptr< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged const> Ardrone3PilotingStateGpsLocationChangedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ac3eedcc2193887dd8d2257bf807fbae";
  }

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xac3eedcc2193887dULL;
  static const uint64_t static_value2 = 0xd8d2257bf807fbaeULL;
};

template<class ContainerAllocator>
struct DataType< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bebop_msgs/Ardrone3PilotingStateGpsLocationChanged";
  }

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Ardrone3PilotingStateGpsLocationChanged\n\
# auto-generated from up stream XML files at\n\
#   github.com/Parrot-Developers/libARCommands/tree/master/Xml\n\
# To check upstream commit hash, refer to last_build_info file\n\
# Do not modify this file by hand. Check scripts/meta folder for generator files.\n\
#\n\
# SDK Comment: Drones location changed.\\n This event is meant to replace [PositionChanged](#1-4-4).\n\
\n\
Header header\n\
\n\
# Latitude location in decimal degrees (500.0 if not available)\n\
float64 latitude\n\
# Longitude location in decimal degrees (500.0 if not available)\n\
float64 longitude\n\
# Altitude location in meters.\n\
float64 altitude\n\
# Latitude location error in meters (1 sigma/standard deviation) -1 if not available.\n\
int8 latitude_accuracy\n\
# Longitude location error in meters (1 sigma/standard deviation) -1 if not available.\n\
int8 longitude_accuracy\n\
# Altitude location error in meters (1 sigma/standard deviation) -1 if not available.\n\
int8 altitude_accuracy\n\
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

  static const char* value(const ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
      stream.next(m.latitude_accuracy);
      stream.next(m.longitude_accuracy);
      stream.next(m.altitude_accuracy);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Ardrone3PilotingStateGpsLocationChanged_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bebop_msgs::Ardrone3PilotingStateGpsLocationChanged_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<double>::stream(s, indent + "  ", v.altitude);
    s << indent << "latitude_accuracy: ";
    Printer<int8_t>::stream(s, indent + "  ", v.latitude_accuracy);
    s << indent << "longitude_accuracy: ";
    Printer<int8_t>::stream(s, indent + "  ", v.longitude_accuracy);
    s << indent << "altitude_accuracy: ";
    Printer<int8_t>::stream(s, indent + "  ", v.altitude_accuracy);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEBOP_MSGS_MESSAGE_ARDRONE3PILOTINGSTATEGPSLOCATIONCHANGED_H
