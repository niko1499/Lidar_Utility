/**
 * @file don_segmentation.cpp
 * Difference of Normals Example for PCL Segmentation Tutorials.
 *
 * @author Yani Ioannou
 * @date 2012-09-24
 */
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/passthrough.h>

using namespace pcl;
using namespace std;

	int
main (int argc, char *argv[])
{


	if (argc < 4)
	{
		cerr << "usage: " << argv[0] << " inputfile1.pcd inputfile2.pcd outputfile.pcd xShift yShift zShift" << endl;
		exit (EXIT_FAILURE);
	}

	/// the file to read from.
	string infile= argv[1];
	string outfile =argv[2];
	double xMin=-10;
	double yMin=-10;
	double zMin=-10;
	double xMax=10;
	double yMax=10;
	double zMax=10;
	if(argc>=3){
		xMin = atof (argv[3]);
	}
	if(argc>=4){
xMax = atof (argv[4]);
	}
if(argc>=5){
		yMin = atof (argv[5]);
	}
if(argc>=6){
		yMax = atof (argv[6]);
	}
if(argc>=7){
		zMin = atof (argv[7]);
	}
if(argc>=8){
		zMax = atof (argv[8]);
	}



	std::cout << "b\n";
	// Load cloud in blob format
	pcl::PCLPointCloud2 blob1;
	pcl::io::loadPCDFile (infile.c_str (), blob1);
	pcl::PointCloud<PointNormal>::Ptr cloud (new pcl::PointCloud<PointNormal>);
	pcl::fromPCLPointCloud2 (blob1, *cloud);

	pcl::PointIndices::Ptr indices_x (new pcl::PointIndices);
	pcl::PointIndices::Ptr indices_xy (new pcl::PointIndices);

	///pcl::PCLPointX cloud_filtered_x;
	//pcl::PCLPointCloud2 cloud_filtered_xz;

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered_x (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered_xy (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered_xyz (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::PassThrough<pcl::PointNormal> ptfilter (true); // Initializing with true will allow us to extract the removed indices
	ptfilter.setInputCloud (cloud);
	ptfilter.setFilterFieldName ("x");
	ptfilter.setFilterLimits (xMin,xMax);
	ptfilter.filter (*cloud_filtered_x);

	ptfilter.setInputCloud(cloud_filtered_x);
	ptfilter.setFilterFieldName ("y");
	ptfilter.setFilterLimits (yMin,yMax);
	ptfilter.filter (*cloud_filtered_xy);

	//float zMid = zMaxf -((zMaxf-zMinf)/2);

	ptfilter.setInputCloud(cloud_filtered_xy);
	ptfilter.setFilterFieldName ("z");
	ptfilter.setFilterLimits (zMin,zMax);//SETTING
	ptfilter.setNegative (false);
	ptfilter.filter (*cloud_filtered_xyz);

	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal> (outfile, *cloud_filtered_xyz, false); 


}

