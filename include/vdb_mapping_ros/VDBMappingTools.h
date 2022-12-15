#ifndef VDB_MAPPING_ROS_VDBMAPPINGTOOLS_H_INCLUDED
#define VDB_MAPPING_ROS_VDBMAPPINGTOOLS_H_INCLUDED

#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

/*!
 * \brief Collection of VDBMapping helper functions and tools
 */
template <typename VDBMappingT>
class VDBMappingTools
{
public:
  VDBMappingTools(){};
  virtual ~VDBMappingTools(){};

  /*!
   * \brief Creates output msgs for pointcloud and marker arrays
   *
   * \param grid Map grid
   * \param resolution Resolution of the grid
   * \param frame_id Frame ID of the grid
   * \param marker_msg Output Marker message
   * \param cloud_msg Output Pointcloud message
   * \param create_marker Flag specifying to create a marker message
   * \param create_pointcloud Flag specifying to create a pointcloud message
   */
  static void createMappingOutput(const typename VDBMappingT::GridT::Ptr grid,
                                  const std::string& frame_id,
                                  visualization_msgs::msg::Marker& marker_msg,
                                  sensor_msgs::msg::PointCloud2& cloud_msg,
                                  const bool create_marker,
                                  const bool create_pointcloud,
                                  const double lower_z_limit,
                                  const double upper_z_limit,
                                  rclcpp::Clock clock);

  /*!
   * \brief Calculates a height correlating color coding using HSV color space
   *
   * \param height Gridcell height relativ to the min and max height of the complete grid. Parameter
   * can take values between 0 and 1
   *
   * \returns RGBA color of the grid cell
   */
  static std_msgs::msg::ColorRGBA heightColorCoding(const double height);
};

#include "VDBMappingTools.hpp"

#endif /* VDB_MAPPING_ROS_VDBMAPPINGTOOLS_H_INCLUDED */
