#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <iostream>



int main ()

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_transformed (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> use_cloud;
  pcl::PointCloud<pcl::PointXYZ> pc_transformed;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/aims/engineer_fac.pcd1709227398312705.pcd", *cloud) == -1) //* load the file

  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  for (const auto& point: *cloud)

    std::cout << "    " << point.x

              << " "    << point.y

              << " "    << point.z << std::endl;

    use_cloud = *cloud;

    Eigen::Matrix4f trans;
        trans<< 0,   0,  1, 0,
                1,   0,  0, 0,
                0,   1,  0, 0,
                0,   0,  0, 1;
                
    pcl::transformPointCloud(use_cloud, *ptr_transformed, trans);
    pc_transformed = *ptr_transformed;

    pcl::io::savePCDFileASCII ("/home/aims/en_trans.pcd", pc_transformed);


  return (0);

}