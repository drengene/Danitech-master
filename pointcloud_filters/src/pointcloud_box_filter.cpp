#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <rclcpp/rclcpp.hpp>

// Example taken from https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html#

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width  = 5;
  cloud->height = 5;
  cloud->points.resize (cloud->width * cloud->height);

  for (auto& point: *cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (const auto& point: *cloud)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> passZ;
  passZ.setInputCloud (cloud);
  passZ.setFilterFieldName ("z");
  passZ.setFilterLimits (0.0, 1.0);
  passZ.setFilterLimitsNegative (true);
  passZ.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  for (const auto& point: *cloud_filtered)
    std::cerr << "    " << point.x << " "
                        << point.y << " "
                        << point.z << std::endl;

  return (0);
}