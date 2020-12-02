#include <iostream>
#include <thread>
#include <vector>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
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
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/lccp_segmentation.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

#define Random(x) (rand() % x)

// typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;

using namespace std::chrono_literals;

using namespace std;
using namespace pcl;
void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer);


int
main (int argc, char** argv)
{
  pcl::search::Search <pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  pcl::PointCloud <pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGBA>),cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGBA> ("test.pcd", *cloud) == -1 )
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  float voxel_resolution = 0.001f; // the voxel size, which determines the leaf size of the underlying octree structure (in meters)
  float seed_resolution = 0.02f; // the seeding size, which determines how big the supervoxels will be (in meters)
  float color_importance = 0.2f;
  float spatial_importance = 0.2f; // the weight for spatial term - higher values will result in supervoxels with very regular shapes
  float normal_importance = 1.0f; // the weight for normal - how much surface normals will influence the shape of the supervoxels

  pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
  super.setInputCloud(cloud);
  super.setColorImportance(color_importance);
  super.setSpatialImportance(spatial_importance);
  super.setNormalImportance(normal_importance);

  std::map <uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  pcl::console::print_highlight("Extracting supervoxels!\n");
  super.setUseSingleCameraTransform(false);
  super.extract(supervoxel_clusters);

  
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");
 
  pcl::io::savePCDFileBinary("scene_voxel.pcd", *labeled_voxel_cloud);

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
  for ( ; label_itr != supervoxel_adjacency.end (); )
  {
    //First get the label
    uint32_t supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);
 
    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
    for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }
    //Now we make a name for this polygon
    std::stringstream ss;
    ss << "supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }
//////////////////vccs_segmentation_label_coclored///////////////////////////////
  pcl::PointCloud<pcl::PointXYZL>::Ptr overseg = super.getLabeledCloud();
	ofstream outFile1("overseg.txt", std::ios_base::out);
	for (int i = 0; i < overseg->size(); i++) {
		outFile1 << overseg->points[i].x << "\t" << overseg->points[i].y << "\t" << overseg->points[i].z << "\t" << overseg->points[i].label << endl;
	}
	int label_max1 = 0;
	for (int i = 0;i< overseg->size(); i++) {
		if (overseg->points[i].label>label_max1)
			label_max1 = overseg->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud1->height = 1;
	ColoredCloud1->width = overseg->size();
	ColoredCloud1->resize(overseg->size());
	for (int i = 0; i < label_max1; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);
		
		for (int j = 0; j < overseg->size(); j++) {
			if (overseg->points[j].label == i) {
				ColoredCloud1->points[j].x = overseg->points[j].x;
				ColoredCloud1->points[j].y = overseg->points[j].y;
				ColoredCloud1->points[j].z = overseg->points[j].z;
				ColoredCloud1->points[j].r = color_R;
				ColoredCloud1->points[j].g = color_G;
				ColoredCloud1->points[j].b = color_B;
			}
		}
	}
	pcl::io::savePCDFileASCII("overseg.pcd", *ColoredCloud1);
//////////////////////////////////////////////////////////////////////////////

///////////////////////lccp/////////////////////////////////
  float concavity_tolerance_threshold = 2;
	float smoothness_threshold = 0.2;
	uint32_t min_segment_size = 0;
	unsigned int k_factor = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");

	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);

	ofstream outFile2("occulsion.txt", std::ios_base::out);
	int a = 0;
	for (int i = 0; i < lccp_labeled_cloud->size();i++) {
		outFile2 << lccp_labeled_cloud->points[i].x << "\t" << lccp_labeled_cloud->points[i].y << "\t" << lccp_labeled_cloud->points[i].z << "\t" << lccp_labeled_cloud->points[i].label << endl;
	}

	int label_max2 = 0;
	
	for (int i = 0; i< lccp_labeled_cloud->size(); i++) {
		if (lccp_labeled_cloud->points[i].label>label_max2)
			label_max2 = lccp_labeled_cloud->points[i].label;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColoredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
	ColoredCloud2->height = 1;
	ColoredCloud2->width = lccp_labeled_cloud->size();
	ColoredCloud2->is_dense = true;
	ColoredCloud2->resize(lccp_labeled_cloud->size());

	for (int i = 0; i < label_max2; i++) {
		int color_R = Random(255);
		int color_G = Random(255);
		int color_B = Random(255);
		
        
		for (int j = 0; j < lccp_labeled_cloud->size(); j++) {
			if (lccp_labeled_cloud->points[j].label == i) {
				ColoredCloud2->points[j].x = lccp_labeled_cloud->points[j].x;
				ColoredCloud2->points[j].y = lccp_labeled_cloud->points[j].y;
				ColoredCloud2->points[j].z = lccp_labeled_cloud->points[j].z;
				ColoredCloud2->points[j].r = color_R;
				ColoredCloud2->points[j].g = color_G;
				ColoredCloud2->points[j].b = color_B;
				

			}
		}
	}
	pcl::io::savePCDFileASCII("occulsion.pcd", *ColoredCloud2);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
  return 0;
}

void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
 
  //Iterate through all adjacent points, and add a center point to adjacent point pair
  PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
  for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}

