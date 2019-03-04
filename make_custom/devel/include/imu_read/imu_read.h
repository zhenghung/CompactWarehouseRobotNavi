// Generated by gencpp from file imu_read/imu_read.msg
// DO NOT EDIT!


#ifndef IMU_READ_MESSAGE_IMU_READ_H
#define IMU_READ_MESSAGE_IMU_READ_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace imu_read
{
template <class ContainerAllocator>
struct imu_read_
{
  typedef imu_read_<ContainerAllocator> Type;

  imu_read_()
    : header()
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , gx(0.0)
    , gy(0.0)
    , gz(0.0)
    , ax(0.0)
    , ay(0.0)
    , az(0.0)
    , mx(0.0)
    , my(0.0)
    , mz(0.0)  {
    }
  imu_read_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , x(0.0)
    , y(0.0)
    , theta(0.0)
    , gx(0.0)
    , gy(0.0)
    , gz(0.0)
    , ax(0.0)
    , ay(0.0)
    , az(0.0)
    , mx(0.0)
    , my(0.0)
    , mz(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _theta_type;
  _theta_type theta;

   typedef float _gx_type;
  _gx_type gx;

   typedef float _gy_type;
  _gy_type gy;

   typedef float _gz_type;
  _gz_type gz;

   typedef float _ax_type;
  _ax_type ax;

   typedef float _ay_type;
  _ay_type ay;

   typedef float _az_type;
  _az_type az;

   typedef float _mx_type;
  _mx_type mx;

   typedef float _my_type;
  _my_type my;

   typedef float _mz_type;
  _mz_type mz;





  typedef boost::shared_ptr< ::imu_read::imu_read_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::imu_read::imu_read_<ContainerAllocator> const> ConstPtr;

}; // struct imu_read_

typedef ::imu_read::imu_read_<std::allocator<void> > imu_read;

typedef boost::shared_ptr< ::imu_read::imu_read > imu_readPtr;
typedef boost::shared_ptr< ::imu_read::imu_read const> imu_readConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::imu_read::imu_read_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::imu_read::imu_read_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace imu_read

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'imu_read': ['/home/iman/make_custom/src/imu_read/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::imu_read::imu_read_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::imu_read::imu_read_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::imu_read::imu_read_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::imu_read::imu_read_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::imu_read::imu_read_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::imu_read::imu_read_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::imu_read::imu_read_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6d318de967de5f12514489daed47dfd9";
  }

  static const char* value(const ::imu_read::imu_read_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6d318de967de5f12ULL;
  static const uint64_t static_value2 = 0x514489daed47dfd9ULL;
};

template<class ContainerAllocator>
struct DataType< ::imu_read::imu_read_<ContainerAllocator> >
{
  static const char* value()
  {
    return "imu_read/imu_read";
  }

  static const char* value(const ::imu_read::imu_read_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::imu_read::imu_read_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 x\n\
float32 y\n\
float32 theta\n\
float32 gx\n\
float32 gy\n\
float32 gz\n\
float32 ax\n\
float32 ay\n\
float32 az\n\
float32 mx\n\
float32 my\n\
float32 mz\n\
\n\
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

  static const char* value(const ::imu_read::imu_read_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::imu_read::imu_read_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
      stream.next(m.gx);
      stream.next(m.gy);
      stream.next(m.gz);
      stream.next(m.ax);
      stream.next(m.ay);
      stream.next(m.az);
      stream.next(m.mx);
      stream.next(m.my);
      stream.next(m.mz);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct imu_read_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::imu_read::imu_read_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::imu_read::imu_read_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<float>::stream(s, indent + "  ", v.theta);
    s << indent << "gx: ";
    Printer<float>::stream(s, indent + "  ", v.gx);
    s << indent << "gy: ";
    Printer<float>::stream(s, indent + "  ", v.gy);
    s << indent << "gz: ";
    Printer<float>::stream(s, indent + "  ", v.gz);
    s << indent << "ax: ";
    Printer<float>::stream(s, indent + "  ", v.ax);
    s << indent << "ay: ";
    Printer<float>::stream(s, indent + "  ", v.ay);
    s << indent << "az: ";
    Printer<float>::stream(s, indent + "  ", v.az);
    s << indent << "mx: ";
    Printer<float>::stream(s, indent + "  ", v.mx);
    s << indent << "my: ";
    Printer<float>::stream(s, indent + "  ", v.my);
    s << indent << "mz: ";
    Printer<float>::stream(s, indent + "  ", v.mz);
  }
};

} // namespace message_operations
} // namespace ros

#endif // IMU_READ_MESSAGE_IMU_READ_H
