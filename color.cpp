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

typedef pcl::PointXYZRGB PointType;

using namespace std::chrono_literals;

using namespace std;
using namespace pcl;

int
main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("overseg.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (cloud_filtered);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (5);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (3500);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "color_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());

  string addr = "/home/jinglan/pcl_tutorial/plytopcd/build/";
  string filename;

  int num = 10;

  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer2(new pcl::visualization::PCLVisualizer("Viewer2"));
  viewer2->setBackgroundColor(0, 0, 0);
 
    // 创建窗口
  int vp;
  viewer2->createViewPort(0.0, 0.0, 0.5, 1.0, vp);
 
    for(int i = 0;i <= num; i++){
        //filename 为点云文件名
        filename = addr + "color_" + to_string(i) + ".pcd";
        //读取点云
        pcl::io::loadPCDFile(filename, *object_cloud);

        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(object_cloud);
        std::cout << i << std::endl;
        // viewer->removePointCloud(std::to_string(i));  
        viewer2->addPointCloud<pcl::PointXYZRGB> (object_cloud, rgb, std::to_string(i));
        viewer2->spinOnce ();

        pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;

      ////////////////////////////////////////////////////////
        
        feature_extractor.setInputCloud (object_cloud);
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

        Eigen::Affine3f t;
        t.matrix() = transformation;
        viewer2->addCoordinateSystem (0.1,t);
     
    }

  int vc;
  viewer2->createViewPort(0.5, 0.0, 1.0, 1.0, vc);
  viewer2->setBackgroundColor(0, 0, 0, vc);

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
  viewer2->addPointCloud<pcl::PointXYZRGB> (colored_cloud, rgb, "Color viewer", vc);

  // pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  // pcl::visualization::CloudViewer viewer ("Cluster viewer");
  // viewer.showCloud (colored_cloud);
  while (!viewer2->wasStopped()) {
    viewer2->spinOnce(100);
  }
  return 0;
}