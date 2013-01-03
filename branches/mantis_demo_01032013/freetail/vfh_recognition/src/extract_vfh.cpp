#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>


//typedef std::pair<std::string, std::vector<float> > vfh_model;


int
main (int argc, char** argv)
{
  if(argc != 3)
  {
    std::cout << "build_data requires two aruments. Usage build_data [object name] [num instances]" << std::endl;
  return(-1);
  }

  //Parse command line:
  std::string objectName = argv[1];
  int numInstances = atoi(argv[2]);
  std::stringstream fileName, outFileName;
  fileName.str("");

  //Point cloud with 1 point that contains the VFHSignature308.

  for(unsigned int i = 1; i < numInstances; i++)
  {
    //Read .pcd file.
    fileName << objectName <<  "_" << i << ".pcd";
    std::cout << "reading file: " << fileName.str() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (fileName.str(), *cloud);
  
    //if (pcl::io::loadPCDFile (fileName.str(), *cloud) == -1)
      //std::cout << "error loading matched model." << std::endl;
    if (cloud->points.size () == 0)
      std::cout << "model cloud has no points!" << std::endl;


    //Extract VFH signature: /
    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);

    //Estimate normals:
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.03);
    ne.compute (*cloud_normals);
    vfh.setInputNormals (cloud_normals);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    vfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);

    //Write to file
    outFileName << objectName << "_" << i <<"_vfh.pcd";
    pcl::io::savePCDFile(outFileName.str(), *vfhs);
    // vfhs->points.size () should be of size 1*
    fileName.str("");
    outFileName.str("");
  }


  return (0);
}
