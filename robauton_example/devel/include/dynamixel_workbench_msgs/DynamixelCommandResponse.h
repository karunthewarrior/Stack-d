// Generated by gencpp from file dynamixel_workbench_msgs/DynamixelCommandResponse.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_DYNAMIXELCOMMANDRESPONSE_H
#define DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_DYNAMIXELCOMMANDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_workbench_msgs
{
template <class ContainerAllocator>
struct DynamixelCommandResponse_
{
  typedef DynamixelCommandResponse_<ContainerAllocator> Type;

  DynamixelCommandResponse_()
    : comm_result(false)  {
    }
  DynamixelCommandResponse_(const ContainerAllocator& _alloc)
    : comm_result(false)  {
  (void)_alloc;
    }



   typedef uint8_t _comm_result_type;
  _comm_result_type comm_result;





  typedef boost::shared_ptr< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> const> ConstPtr;

}; // struct DynamixelCommandResponse_

typedef ::dynamixel_workbench_msgs::DynamixelCommandResponse_<std::allocator<void> > DynamixelCommandResponse;

typedef boost::shared_ptr< ::dynamixel_workbench_msgs::DynamixelCommandResponse > DynamixelCommandResponsePtr;
typedef boost::shared_ptr< ::dynamixel_workbench_msgs::DynamixelCommandResponse const> DynamixelCommandResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dynamixel_workbench_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'dynamixel_workbench_msgs': ['/home/ktw/ws/robauton_example/src/cmu-16662-robot-ctrl/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f12f56e0367ef7d42085bd5f9c478576";
  }

  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf12f56e0367ef7d4ULL;
  static const uint64_t static_value2 = 0x2085bd5f9c478576ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_workbench_msgs/DynamixelCommandResponse";
  }

  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
bool comm_result\n\
\n\
";
  }

  static const char* value(const ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.comm_result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DynamixelCommandResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_workbench_msgs::DynamixelCommandResponse_<ContainerAllocator>& v)
  {
    s << indent << "comm_result: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.comm_result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_DYNAMIXELCOMMANDRESPONSE_H
