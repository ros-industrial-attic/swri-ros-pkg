//cph feature.h
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
//#include <flann/io/hdf5.h>
#include <math.h>

#define PI 3.14159265

class CPHEstimation{
  
public:
  
  CPHEstimation(){
   num_ybins = 5; 
   num_cbins = 180;
   hist.resize(num_ybins * num_cbins,0);
   centroid.resize(3,0);
  };
  
  CPHEstimation(int num_y, int num_c){
   num_ybins = num_y;
   num_cbins = num_c;
   hist.resize(num_ybins * num_cbins,0);
   centroid.resize(3,0);
  };
  
  bool setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){
   cloud = inputCloud;
   return 1;
  };
  
int compute(std::vector<float> &result){
  //compute bouding box size:
  float x_max=0, x_min=100, y_max=0, y_min = 100, z_max=0, z_min=100;
  for(unsigned int i=0; i<cloud->size(); i++){
    if(cloud->points.at(i).x > x_max)
      x_max = cloud->points.at(i).x;
    if(cloud->points.at(i).x < x_min)
      x_min = cloud->points.at(i).x;
    if(cloud->points.at(i).y > y_max)
      y_max = cloud->points.at(i).y;
    if(cloud->points.at(i).y < y_min)
      y_min = cloud->points.at(i).y;
    if(cloud->points.at(i).z > z_max)
      z_max = cloud->points.at(i).z;
    if(cloud->points.at(i).z < z_min)
      z_min = cloud->points.at(i).z;
  }
  //y_max+=.01;
  //x_max+=.01;
  //z_max+=.01;
  
  x_size = (x_max - x_min);
  y_size = (y_max - y_min);
  z_size = (z_max - z_min);
  
  float max_size = x_size;
  if(y_size > max_size)
    max_size = y_size;
  if(z_size > max_size)
    max_size = z_size;
  max_size*=100;
  
  centroid.at(0) = x_min + x_size/2;
  centroid.at(1) = y_min + y_size/2;
  centroid.at(2) = z_min + z_size/2;

  //compute feature
  hist.clear();
  hist.resize(num_cbins*num_ybins, 0.0f);
  int y,c;
  float dy = y_size/num_ybins;
  float dc = 2*PI/num_cbins;
  //std::cout << "dy: " << dy << "  dc: " << dc << std::endl;
  
  for(unsigned int i=0; i<cloud->size(); i++){
    //bin z component
    y = floor((cloud->points.at(i).y-y_min)/dy);
    c = floor((PI+atan2(cloud->points.at(i).z-centroid.at(2),cloud->points.at(i).x-centroid.at(0)))/dc);
    //std::cout << "binning point " << i << ". y: " << y << "  c: " << c << std::endl;
    if(y*num_cbins+c < hist.size())
      hist.at(y*num_cbins+c)+=1.0f;  
  }
  
  //Find tallest peak
  float max_peak = 0;
  for(unsigned int i=0; i<hist.size(); i++){
    if(hist.at(i) > max_peak)
     max_peak = (float)hist.at(i); 
  }
  
  //Rescale cph to largest spatial extent:
  float scaleFactor = max_size/max_peak;
  for(unsigned int i=0; i<hist.size(); i++){
    hist.at(i)*=scaleFactor;
  }
    
  //Stick size onto the end and BAM! scale variance.
  hist.push_back(x_size*100);
  hist.push_back(y_size*100); 
  hist.push_back(z_size*100);
  
  result.clear();
  result = hist;
  return result.size();
};

private:
  
  std::vector<float> hist;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, result;
  int num_ybins, num_cbins;
  float x_size, y_size, z_size;
  std::vector<float> centroid;  
};