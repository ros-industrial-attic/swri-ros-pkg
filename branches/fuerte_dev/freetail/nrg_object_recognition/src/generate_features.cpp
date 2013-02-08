#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <fstream>
#include "cph.h"
#include <sys/time.h>
#include <boost/filesystem.hpp>

std::stringstream outFileName;
std::ofstream outFile, timeFile;

  //Feature timing:
  int start, stop, numRuns=0;;
  double runTime = 0;

void extractFeatures (const boost::filesystem::path &base_dir, const std::string &extension)
{
  int writeIndex = 0;
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;
  
  timeFile.open("time.csv");
  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
//       std::stringstream ss;
//       ss << it->path ();
      extractFeatures (it->path (), extension);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      //pcl::PointCloud<pcl::PointXYZ>::Ptr  raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZ>);
      std::stringstream inFile;
      inFile.str("");
      inFile << "temp2/" << it->path().filename().c_str();
      pcl::io::loadPCDFile (inFile.str().c_str(), *cloud);
      
       //Write vfh feature to file: 
      std::stringstream fileName_ss;
      pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
      vfh.setInputCloud (cloud);
      
      //Estimate normals:
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (cloud);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch (0.03);
      
      runTime = 0;
      start = clock();
      ne.compute (*cloud_normals);
      stop = clock();
      runTime += (double)(stop-start)/CLOCKS_PER_SEC;
      
      vfh.setInputNormals (cloud_normals);
      //Estimate vfh:
      vfh.setSearchMethod (tree);
      pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
      
      // Compute the feature
      start = clock();
      vfh.compute (*vfhs);
      stop = clock();
      runTime += (double)(stop-start)/CLOCKS_PER_SEC;
      timeFile << cloud->points.size() << " " << runTime*1000.0 << " ";
      std::cout << runTime << " ";
      runTime = 0;
      
      //Write to file: (objectName_angle_vfh.pcd)
      fileName_ss.str("");
      //std::cout << "writing vfh descriptor to file...\n";
      fileName_ss << "temp/" << it->path().filename().c_str();
      std::string vfhOutString(fileName_ss.str());
      vfhOutString.insert(vfhOutString.rfind("."), "_vfh");
      pcl::io::savePCDFile(vfhOutString, *vfhs);
      //std::cout << "done.\n";
    
      //Extract CPH signature: 
      std::vector<float> feature;
      CPHEstimation cph(5,72);
      cph.setInputCloud(cloud);
      
      // Compute the feature
      start = clock();
      cph.compute(feature);
      stop = clock();
      runTime += (double)(stop-start)/CLOCKS_PER_SEC;
      timeFile << runTime*1000.0 << std::endl;
      std::cout << runTime*1e6 << std::endl;
      runTime=0;
      
      //Write feature to file
      outFileName << "temp/" << it->path().filename().c_str();
      //std::cout << "Read file: " << outFileName.str() << std::endl;;
      std::string outString(outFileName.str());
      outString.replace(outString.end()-3, outString.end(), "csv");
      outFile.open(outString.c_str());
      for(unsigned int j=0; j<feature.size(); j++){
	outFile << feature.at(j) << " "; 
      }
      outFile.close();
      outFileName.str("");
      writeIndex++;
    }
  }
  timeFile.close();
}

int main (int argc, char** argv)
{
  outFileName.str("");
  
  extractFeatures(argv[1], ".pcd");
  
  return(0);
}
 
  
 
 
  
