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

using namespace pcl;
using namespace std;

	int
main (int argc, char *argv[])
{


	if (argc < 3)
	{
		cerr << "usage: " << argv[0] << " inputfile.pcd outputfile.pcd" << endl;
		exit (EXIT_FAILURE);
	}

	/// the file to read from.
	string infile = argv[1];
	string outfile =argv[2];
	if(outfile=="same"||outfile=="overwrite"||outfile=="o"){
		outfile=infile;
	}
	// Load cloud in blob format
	pcl::PCLPointCloud2 blob;
	pcl::io::loadPCDFile (infile.c_str (), blob);
	pcl::PointCloud<PointNormal>::Ptr cloud (new pcl::PointCloud<PointNormal>);
	pcl::fromPCLPointCloud2 (blob, *cloud);

	pcl::PointNormal pMin,pMax;
	pcl::getMinMax3D (*cloud,pMin,pMax);

	float xCenter=(abs(pMax.x-pMin.x)/2)+pMin.x;
	float yCenter=(abs(pMax.y-pMin.y)/2)+pMin.y;
	float zCenter=(abs(pMax.z-pMin.z)/2)+pMin.z;

	//float theta =M_PI/2;
	float theta = 0;// define rotation
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0,0)= cos(theta);
	transform(0,1)= -sin(theta);
	transform(1,0)= sin(theta);
	transform(1,1)= cos(theta);


	transform (0,3) = -xCenter; // x shift.
	transform(1,3)=-yCenter; //y shift
	printf ("Transform: Matrix4f\n");
	std::cout << transform << std::endl;

	pcl::PointCloud<PointNormal>::Ptr result (new pcl::PointCloud<PointNormal>);

	pcl::transformPointCloudWithNormals(*cloud,*result,transform);

	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal> (outfile, *result, false); 


}

