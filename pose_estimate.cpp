#include <iostream>
#include <thread>
#include <vector>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;

using namespace std::chrono_literals;

using namespace std;
using namespace pcl;

int
main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("color_1.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  viewer->addPointCloud (cloud,"cloud");

  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;

////////////////////////////////////////////////////////
  
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  Eigen::Vector3f ob_major_vector, ob_middle_vector, ob_minor_vector;
  Eigen::Vector3f ob_mass_center;

  feature_extractor.getEigenVectors (ob_major_vector, ob_middle_vector, ob_minor_vector);
  feature_extractor.getMassCenter (ob_mass_center);

  pcl::PointXYZ ob_center (ob_mass_center (0), ob_mass_center (1), ob_mass_center (2));
  pcl::PointXYZ ob_z_axis (ob_major_vector (0) + ob_mass_center (0), ob_major_vector (1) + ob_mass_center (1), ob_major_vector (2) + ob_mass_center (2));
  pcl::PointXYZ ob_y_axis (ob_middle_vector (0) + ob_mass_center (0), ob_middle_vector (1) + ob_mass_center (1), ob_middle_vector (2) + ob_mass_center (2));
  pcl::PointXYZ ob_x_axis (ob_minor_vector (0) + ob_mass_center (0), ob_minor_vector (1) + ob_mass_center (1), ob_minor_vector (2) + ob_mass_center (2));

  Eigen::Matrix4f transformation;
  transformation (0,0) = ob_center.x-ob_x_axis.x, transformation (0,1) = ob_center.x-ob_y_axis.x, transformation (0,2) = ob_center.x-ob_z_axis.x, transformation (0,3) = ob_center.x;
  transformation (1,0) = ob_center.y-ob_x_axis.y, transformation (1,1) = ob_center.y-ob_y_axis.y , transformation (1,2) = ob_center.y-ob_z_axis.y, transformation (1,3) = ob_center.y;
  transformation (2,0) = ob_center.z-ob_x_axis.z, transformation (2,1) = ob_center.z-ob_y_axis.z , transformation (2,2) = ob_center.z-ob_z_axis.z, transformation (2,3) = ob_center.z;
  transformation (3,0) = 0.0, transformation (3,1) = 0.0 , transformation (3,2) = 0, transformation (3,3) = 1.0;

  std::cout << ob_center << std::endl;

  Eigen::Affine3f t;
  t.matrix() = transformation;
  viewer->addCoordinateSystem (0.1,t);
  // viewer->addCoordinateSystem (0.2);
     
  


  // pcl::PointCloud <pcl::PointXYZ>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::visualization::CloudViewer viewer ("Cluster viewer");
  // viewer.showCloud (colored_cloud);
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
  return 0;
}