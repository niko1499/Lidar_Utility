// Generated by gencpp from file rslidar_msgs/rslidarScan.msg
// DO NOT EDIT!


#ifndef RSLIDAR_MSGS_MESSAGE_RSLIDARSCAN_H
#define RSLIDAR_MSGS_MESSAGE_RSLIDARSCAN_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <rslidar_msgs/rslidarPacket.h>

namespace rslidar_msgs
{
template <class ContainerAllocator>
struct rslidarScan_
{
  typedef rslidarScan_<ContainerAllocator> Type;

  rslidarScan_()
    : header()
    , packets()  {
    }
  rslidarScan_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , packets(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::rslidar_msgs::rslidarPacket_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::rslidar_msgs::rslidarPacket_<ContainerAllocator> >::other >  _packets_type;
  _packets_type packets;





  typedef boost::shared_ptr< ::rslidar_msgs::rslidarScan_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rslidar_msgs::rslidarScan_<ContainerAllocator> const> ConstPtr;

}; // struct rslidarScan_

typedef ::rslidar_msgs::rslidarScan_<std::allocator<void> > rslidarScan;

typedef boost::shared_ptr< ::rslidar_msgs::rslidarScan > rslidarScanPtr;
typedef boost::shared_ptr< ::rslidar_msgs::rslidarScan const> rslidarScanConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::rslidar_msgs::rslidarScan_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace rslidar_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'rslidar_msgs': ['/home/pc/Lidar_Utility/src/rslidar/rslidar_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::rslidar_msgs::rslidarScan_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::rslidar_msgs::rslidarScan_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::rslidar_msgs::rslidarScan_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff6baa58985b528481871cbaf1bb342f";
  }

  static const char* value(const ::rslidar_msgs::rslidarScan_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff6baa58985b5284ULL;
  static const uint64_t static_value2 = 0x81871cbaf1bb342fULL;
};

template<class ContainerAllocator>
struct DataType< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "rslidar_msgs/rslidarScan";
  }

  static const char* value(const ::rslidar_msgs::rslidarScan_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# LIDAR scan packets.\n\
\n\
Header           header         # standard ROS message header\n\
rslidarPacket[] packets        # vector of raw packets\n\
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
\n\
================================================================================\n\
MSG: rslidar_msgs/rslidarPacket\n\
# Raw LIDAR packet.\n\
\n\
time stamp              # packet timestamp\n\
uint8[1248] data        # packet contents\n\
\n\
";
  }

  static const char* value(const ::rslidar_msgs::rslidarScan_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.packets);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct rslidarScan_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rslidar_msgs::rslidarScan_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::rslidar_msgs::rslidarScan_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "packets[]" << std::endl;
    for (size_t i = 0; i < v.packets.size(); ++i)
    {
      s << indent << "  packets[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::rslidar_msgs::rslidarPacket_<ContainerAllocator> >::stream(s, indent + "    ", v.packets[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // RSLIDAR_MSGS_MESSAGE_RSLIDARSCAN_H
