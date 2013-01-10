#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include "cph.h"
#include <sys/time.h>


//typedef std::pair<std::string, std::vector<float> > vfh_model;


int
main (int argc, char** argv)
{
  if(argc != 3)
  {
    std::cout << "generate_features requires two aruments. Usage generate_features [object name] [num instances]" << std::endl;
  return(-1);
  }

  //Parse command line:
  std::string objectName = argv[1];
  int numInstances = atoi(argv[2]); 
  std::stringstream fileName, outFileName;
  std::ofstream outFile;
  fileName.str("");
  outFileName.str(""); 

  for(unsigned int i=0; i<numInstances; i++)
  { 
    //Read .pcd file
    fileName << "data/" << objectName <<  "_" << i << ".pcd";
    std::cout << "reading file: " << fileName.str() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile (fileName.str(), *cloud);
   
    //Extract CPH signature: 
    std::vector<float> feature;
    CPHEstimation cph(2,16);
    cph.setInputCloud(cloud);
    
    // Compute the feature
    cph.compute(feature);
    
    //Write feature to file
    outFileName << "data/" << objectName << "_" << i << ".csv";
    outFile.open(outFileName.str().c_str());
    for(unsigned int j=0; j<feature.size(); j++){
     outFile << feature.at(j) << " "; 
    }
    outFile.close();
    fileName.str("");
    outFileName.str("");
    
  }

  return (0);
}
