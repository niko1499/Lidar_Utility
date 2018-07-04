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


	if (argc < 4)
	{
		cerr << "usage: " << argv[0] << " inputfile1.pcd inputfile2.pcd outputfile.pcd xShift yShift zShift" << endl;
		exit (EXIT_FAILURE);
	}

	/// the file to read from.
	string infile1 = argv[1];
	string infile2 = argv[2];
	string outfile =argv[3];
	double xShift=0;
	double yShift=0;
	double zShift=0;

	if(argc>4){
		sscanf(argv[4],"%lf",&xShift);
		sscanf(argv[5],"%lf",&yShift);
		sscanf(argv[6],"%lf",&zShift);
	}
	std::cout << "b\n";
	// Load cloud in blob format
	pcl::PCLPointCloud2 blob1;
	pcl::io::loadPCDFile (infile1.c_str (), blob1);
	pcl::PointCloud<PointNormal> cloud1;
	pcl::fromPCLPointCloud2 (blob1, cloud1);

	// Load cloud in blob format
	pcl::PCLPointCloud2 blob2;
	pcl::io::loadPCDFile (infile2.c_str (), blob2);
	pcl::PointCloud<PointNormal> cloud2;
	pcl::fromPCLPointCloud2 (blob2, cloud2);

	//float theta =M_PI/2;
	float theta = 0;// define rotation
	Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
	transform(0,0)= cos(theta);
	transform(0,1)= -sin(theta);
	transform(1,0)= sin(theta);
	transform(1,1)= cos(theta);


	transform (0,3) = xShift; // x shift
	transform(1,3)=yShift; //y shift
	transform(2,3)=zShift; 
	printf ("Transform: Matrix4f\n");
	std::cout << transform << std::endl;

	pcl::PointCloud<PointNormal> input2Shifted ;

	pcl::transformPointCloudWithNormals(cloud2,input2Shifted,transform);

	pcl::PointCloud<PointNormal>::Ptr result (new pcl::PointCloud<PointNormal>);

	cloud1+=input2Shifted;

	pcl::PCDWriter writer;
	writer.write<pcl::PointNormal> (outfile, cloud1, false); 


}

