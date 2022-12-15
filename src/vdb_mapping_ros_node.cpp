#include <vdb_mapping/OccupancyVDBMapping.h>
#include <vdb_mapping_ros/VDBMappingROS.h>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VDBMappingROS<vdb_mapping::OccupancyVDBMapping>>());
  rclcpp::shutdown();
  return 0;
}
