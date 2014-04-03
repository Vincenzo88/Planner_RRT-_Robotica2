/* Auto-generated by genmsg_cpp for file /home/valerio/catkin_ws/src/smartfm/smart-ros-pkg/golfcar_navigation/pnc_msgs/msg/local_map.msg */
#ifndef PNC_MSGS_MESSAGE_LOCAL_MAP_H
#define PNC_MSGS_MESSAGE_LOCAL_MAP_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "nav_msgs/OccupancyGrid.h"

namespace pnc_msgs
{
template <class ContainerAllocator>
struct local_map_ {
  typedef local_map_<ContainerAllocator> Type;

  local_map_()
  : occupancy()
  , free_cells()
  , centerline()
  {
  }

  local_map_(const ContainerAllocator& _alloc)
  : occupancy(_alloc)
  , free_cells(_alloc)
  , centerline(_alloc)
  {
  }

  typedef  ::nav_msgs::OccupancyGrid_<ContainerAllocator>  _occupancy_type;
   ::nav_msgs::OccupancyGrid_<ContainerAllocator>  occupancy;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _free_cells_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  free_cells;

  typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _centerline_type;
  std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  centerline;


  typedef boost::shared_ptr< ::pnc_msgs::local_map_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pnc_msgs::local_map_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct local_map
typedef  ::pnc_msgs::local_map_<std::allocator<void> > local_map;

typedef boost::shared_ptr< ::pnc_msgs::local_map> local_mapPtr;
typedef boost::shared_ptr< ::pnc_msgs::local_map const> local_mapConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pnc_msgs::local_map_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pnc_msgs::local_map_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pnc_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pnc_msgs::local_map_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pnc_msgs::local_map_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pnc_msgs::local_map_<ContainerAllocator> > {
  static const char* value() 
  {
    return "37253253ae33268d3732bc89ba9784db";
  }

  static const char* value(const  ::pnc_msgs::local_map_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x37253253ae33268dULL;
  static const uint64_t static_value2 = 0x3732bc89ba9784dbULL;
};

template<class ContainerAllocator>
struct DataType< ::pnc_msgs::local_map_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pnc_msgs/local_map";
  }

  static const char* value(const  ::pnc_msgs::local_map_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pnc_msgs::local_map_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nav_msgs/OccupancyGrid occupancy\n\
int32[] free_cells\n\
int32[] centerline\n\
\n\
================================================================================\n\
MSG: nav_msgs/OccupancyGrid\n\
# This represents a 2-D grid map, in which each cell represents the probability of\n\
# occupancy.\n\
\n\
Header header \n\
\n\
#MetaData for the map\n\
MapMetaData info\n\
\n\
# The map data, in row-major order, starting with (0,0).  Occupancy\n\
# probabilities are in the range [0,100].  Unknown is -1.\n\
int8[] data\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: nav_msgs/MapMetaData\n\
# This hold basic information about the characterists of the OccupancyGrid\n\
\n\
# The time at which the map was loaded\n\
time map_load_time\n\
# The map resolution [m/cell]\n\
float32 resolution\n\
# Map width [cells]\n\
uint32 width\n\
# Map height [cells]\n\
uint32 height\n\
# The origin of the map [m, m, rad].  This is the real-world pose of the\n\
# cell (0,0) in the map.\n\
geometry_msgs/Pose origin\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::pnc_msgs::local_map_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pnc_msgs::local_map_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.occupancy);
    stream.next(m.free_cells);
    stream.next(m.centerline);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct local_map_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pnc_msgs::local_map_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pnc_msgs::local_map_<ContainerAllocator> & v) 
  {
    s << indent << "occupancy: ";
s << std::endl;
    Printer< ::nav_msgs::OccupancyGrid_<ContainerAllocator> >::stream(s, indent + "  ", v.occupancy);
    s << indent << "free_cells[]" << std::endl;
    for (size_t i = 0; i < v.free_cells.size(); ++i)
    {
      s << indent << "  free_cells[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.free_cells[i]);
    }
    s << indent << "centerline[]" << std::endl;
    for (size_t i = 0; i < v.centerline.size(); ++i)
    {
      s << indent << "  centerline[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.centerline[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PNC_MSGS_MESSAGE_LOCAL_MAP_H
